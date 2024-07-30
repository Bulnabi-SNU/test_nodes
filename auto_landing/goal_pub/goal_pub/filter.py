import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import deque
import numpy as np

class Filter(Node):

    def __init__(self):
        super().__init__('filter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bezier_waypoint/raw',
            self.tag_callback,
            10
        )
        self.publisher = self.create_publisher(Float32MultiArray, 'bezier_waypoint', 10)
        self.raw_values = deque([], maxlen=30)  # 최대 크기를 지정하여 자동으로 관리
        self.raw_values_2 = deque([], maxlen=30)

    def tag_callback(self, msg):
        tag_world = np.array(msg.data)
        self.raw_values.append(tag_world)

        if len(self.raw_values) == 30:
            average = np.mean(self.raw_values, axis=0)
            self.raw_values_2.append(average)
            self.raw_values.clear()
            if len(self.raw_values_2) > 0:
                average_2 = np.mean(self.raw_values_2, axis=0)
                avg_msg = Float32MultiArray()
                avg_msg.data = average_2.tolist()
                self.publisher.publish(avg_msg)
        
        

def main(args=None):
    rclpy.init(args=args)
    filter = Filter()
    rclpy.spin(filter)

    sub.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()