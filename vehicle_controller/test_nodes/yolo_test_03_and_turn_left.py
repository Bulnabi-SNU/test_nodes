__author__ = "Juyong Shin"
__contact__ = "juyong3393@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import GimbalManagerSetManualControl
# add by chaewon
from my_bboxes_msg.msg import VehiclePhase
from my_bboxes_msg.msg import YoloObstacle # label, x, y

# import math, numpy
import math
import numpy as np

class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        """
        0. Configure QoS profile for publishing and subscribing
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        """
        1. Constants
        """
        self.mc_acceptance_radius = 0.3
        self.acceptance_heading_angle = np.radians(0.5)

        """
        2. Set waypoints
        """
        self.WP = [np.array([0.0, 0.0, 0.0])]
        self.declare_parameters(
            namespace='',
            parameters=[
                ('WP1', None),
                ('WP2', None),
                ('WP3', None),
                ('WP4', None),
            ])

        for i in range(1, 5):
            wp_position = self.get_parameter(f'WP{i}').value
            self.WP.append(np.array(wp_position))

        """
        3. State variables
        """
        # phase description
        # -2 : after flight
        # -1 : before flight
        # 0 : takeoff and arm
        # i >= 1 : moving toward WP_i
        self.phase = -1
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.home_position = np.array([0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        
        self.previous_goal = None
        self.current_goal = None

        self.time_checker = 0
        self.step_count = 0

        # add by chaewon
        self.obstacle_label = ''
        self.obstacle_x = 0
        self.obstacle_y = 0
        self.obstacle_orientation = ''

        """
        4. Create Subscribers
        """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        # add by chaewon
        self.yolo_obstacle_subscriber = self.create_subscription(
            YoloObstacle, '/yolo_obstacle', self.yolo_obstacle_callback, qos_profile
        )

        """
        5. Create Publishers
        """
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.gimbal_publisher = self.create_publisher(
            GimbalManagerSetManualControl, '/fmu/in/gimbal_manager_set_manual_control', qos_profile
        )
        # add by chaewon.
        self.vehicle_phase_publisher = self.create_publisher(
            VehiclePhase, '/vehicle_phase', qos_profile
        )

        """
        6. timer setup
        """
        self.offboard_heartbeat = self.create_timer(0.1, self.offboard_heartbeat_callback)
        self.takeoff_timer = self.create_timer(0.5, self.takeoff_and_arm_callback)
        self.main_timer = self.create_timer(0.5, self.main_timer_callback)
        # add by chaewon
        self.vehicle_phase_publisher_timer = self.create_timer(0.5, self.vehicle_phase_publisher_callback)
        
        print("Successfully executed: vehicle_controller")
        print("Please switch to offboard mode.")
        
    
    """
    Services
    """   
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.phase = -2
    
    # 천천히 비행
    def make_setpoint_list(self, start, finish, n):
        self.step_count = 0
        start = np.array(start)
        finish = np.array(finish)
        points = np.linspace(start, finish, num=n+1, endpoint=True)[1:]
        return points.tolist()

    def step_by_step(self, setpoints):
        if self.step_count == len(setpoints):
            pass
        elif np.linalg.norm(self.pos - previous_goal) < self.mc_acceptance_radius:
            self.step_count += 1
        else:
            previous_goal = setpoints[self.step_count]
            self.publish_trajectory_setpoint(position_sp=previous_goal)

    """
    Callback functions for the timers
    """
    # add by chaewon
    def vehicle_phase_publisher_callback(self):
        msg = VehiclePhase()
        msg.phase = str(self.phase)
        self.vehicle_phase_publisher.publish(msg)

    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True)

    def takeoff_and_arm_callback(self):
        if self.phase == -1 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print("Takeoff and arm")
            # takeoff and arm only if the vehicle is in offboard mode by RC switch
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.home_position = self.pos # set home position
            self.phase = 0
    
    def main_timer_callback(self):
        if self.phase == 0:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                    param1=1.0, # main mode
                    param2=6.0  # offboard
                )
                self.phase = 0.3

        elif self.phase == 0.3:
            self.publish_gimbal_control(pitch=-math.pi/6, yaw=self.yaw)
            self.current_goal = np.array([10.0 * math.cos(self.yaw), 10.0 * math.sin(self.yaw), -5.0])
            self.phase = 0.5
            self.ladder_detect_attempts = 0  # Initialize detection attempts
            self.ladder_detected = 0  # Initialize successful detections
            self.first_ladder_detected = False  # Initialize the first detection flag
            
        elif self.phase == 0.5:
            self.publish_trajectory_setpoint(position_sp=self.current_goal)

            # Check for the first detection of ladder-truck
            if not self.first_ladder_detected and self.obstacle_label == 'ladder-truck':
                self.first_ladder_detected = True
                print('First detection of ladder-truck.')

            # Start counting attempts only after the first detection
            if self.first_ladder_detected:
                if self.ladder_detect_attempts < 5:
                    self.ladder_detect_attempts += 1

                    if self.obstacle_label == 'ladder-truck':
                        print(f'Detected obstacle: {self.obstacle_label}')
                        self.ladder_detected += 1
                    else:
                        print(f'Detected obstacle: {self.obstacle_label}')

                if self.ladder_detect_attempts >= 5:
                    if self.ladder_detected >= 3:
                        print('Ladder-truck detected. Changing phase to 1.')
                        self.phase = 1
                        self.time_checker = 0
                    else:
                        print('Ladder-truck not sufficiently detected. Resetting attempts.')
                        self.ladder_detect_attempts = 0
                        self.ladder_detected = 0
                        self.first_ladder_detected = False


        elif self.phase == 1:
            self.publish_trajectory_setpoint(position_sp=self.pos)
            print(f'Direction of obstacle: {self.obstacle_orientation}')
            self.time_checker += 1
            if  self.time_checker >= 12:
                self.time_checker = 0
                self.phase = 1.5
        elif self.phase == 1.5:
            if self.obstacle_orientation == 'left':
                self.current_goal = self.pos + np.array([(3.0)*math.cos(self.yaw+(math.pi/2)), (3.0)*math.sin(self.yaw+(math.pi/2)), -5.0])
            else: # right
                self.current_goal = self.pos + np.array([(3.0)*math.cos(self.yaw-(math.pi/2)), (3.0)*math.sin(self.yaw-(math.pi/2)), -5.0])
            self.time_checker = 0
            self.phase = 2
        elif self.phase == 2:
            self.publish_trajectory_setpoint(position_sp = self.current_goal)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.phase = 3
        elif self.phase == 3:
            self.land()
        print(self.phase)

    """
    Callback functions for subscribers.
    """        
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.yaw = msg.heading
        if self.phase != -1:
            # set position relative to the home position after takeoff
            self.pos = self.pos - self.home_position
    
    # add by chaewon. size of picture is 640*480. x 320 기준으로 판단 
    def yolo_obstacle_callback(self, msg):
        self.obstacle_label = msg.label
        self.obstacle_x = int(msg.x)
        self.obstacle_y = int(msg.y)
        if self.obstacle_x < 320:
            self.obstacle_orientation = 'left'
        else:
            self.obstacle_orientation = 'right'

    """
    Functions for publishing topics.
    """
    def publish_vehicle_command(self, command, **kwargs):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", float('nan'))
        msg.param2 = kwargs.get("param2", float('nan'))
        msg.param3 = kwargs.get("param3", float('nan'))
        msg.param4 = kwargs.get("param4", float('nan'))
        msg.param5 = kwargs.get("param5", float('nan'))
        msg.param6 = kwargs.get("param6", float('nan'))
        msg.param7 = kwargs.get("param7", float('nan'))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def publish_offboard_control_mode(self, **kwargs):
        msg = OffboardControlMode()
        msg.position = kwargs.get("position", False)
        msg.velocity = kwargs.get("velocity", False)
        msg.acceleration = kwargs.get("acceleration", False)
        msg.attitude = kwargs.get("attitude", False)
        msg.body_rate = kwargs.get("body_rate", False)
        msg.thrust_and_torque = kwargs.get("thrust_and_torque", False)
        msg.direct_actuator = kwargs.get("direct_actuator", False)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_trajectory_setpoint(self, **kwargs):
        msg = TrajectorySetpoint()
        # position setpoint is relative to the home position
        msg.position = list( kwargs.get("position_sp", np.nan * np.zeros(3)) + self.home_position )
        msg.velocity = list( kwargs.get("velocity_sp", np.nan * np.zeros(3)) )
        msg.yaw = kwargs.get("yaw_sp", float('nan'))
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {setposition}")
    
    def publish_gimbal_control(self, **kwargs) :
        msg = GimbalManagerSetManualControl()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.origin_sysid = 0
        msg.origin_compid = 0
        msg.flags = GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_ROLL_LOCK \
                    + GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_PITCH_LOCK \
                    + GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_YAW_LOCK
        msg.pitch = kwargs.get("pitch", float('nan'))
        msg.yaw = kwargs.get("yaw", float('nan'))
        msg.pitch_rate = float('nan')
        msg.yaw_rate = float('nan')
        self.gimbal_publisher.publish(msg)
    
def main(args = None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
