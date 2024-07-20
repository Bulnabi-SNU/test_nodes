import os
import rclpy
import sys
# from pathlib import Path # for yolov5 local

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from my_bboxes_msg.msg import YoloObstacle, YoloTarget, VehiclePhase

import cv2
import torch
from cv_bridge import CvBridge
import time
import numpy as np


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # define model path and load the model
        model_path = os.path.join(os.getcwd(), 'src/yolo_detection/config/best.pt')
        self.model = torch.hub.load(os.path.expanduser('~/yolov5'), 'custom', path=model_path, source='local')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(type(self.model))

        
        # create publishers
        self.publisher_obstacle = self.create_publisher(YoloObstacle, '/yolo_obstacle', qos_profile)
        # create subscribe
        self.subscriber_phase = self.create_subscription(VehiclePhase, '/vehicle_phase', self.phase_callback, qos_profile)
        # create phase
        self.phase = '0'
        
        # create cv_bridge instance
        self.bridge = CvBridge()
        
        # create target_capture folder, which is used to save target images
        self.target_capture_folder = os.path.join(os.getcwd(), 'src/yolo_detection/config/target_capture')
        os.makedirs(self.target_capture_folder, exist_ok=True)
        
        # timer for publishing target image
        self.timer_period = 10.0  # seconds
        self.last_capture_time = time.time()
        
        # create a subscriber for the v4l2 image topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)



    def image_callback(self, msg):
        # convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # send frame to YOLOv5 model
        results = self.model(frame)
        # extract bounding box, labels
        labels, cords = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]


        # draw bounding box in frame & publish obstacle message
        for i in range(len(labels)):
            row = cords[i]
            if row[4] >= 0.3:
                x1, y1, x2, y2 = int(row[0] * frame.shape[1]), int(row[1] * frame.shape[0]), int(row[2] * frame.shape[1]), int(row[3] * frame.shape[0])
                label = self.model.names[int(labels[i])]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                
                # publish obstacle message
                if (self.phase == '8' or '8.1' or '8.2' or '8.3' or '8.4') or True:
                    obstacle_msg = YoloObstacle()
                    obstacle_msg.label = label
                    obstacle_msg.x = x_center
                    obstacle_msg.y = y_center
                    self.publisher_obstacle.publish(obstacle_msg)
                
                # draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{label} {row[4]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


        # check if it's time to publish target image
        if (self.phase == '3'):
            current_time = time.time()
            if current_time - self.last_capture_time >= self.timer_period:
                self.publish_target_image(frame)
                self.last_capture_time = current_time


        # display frame
        cv2.imshow('YOLOv5 Detection', frame)
        

        '''
        MJPG streamer
        '''
        # 프레임을 파일로 저장 (MJPG-streamer가 읽을 수 있도록)
        tmp_file = '/tmp/stream.jpg'
        tmp_lock_file = '/tmp/stream.lock'
        # 잠금 파일 생성
        with open(tmp_lock_file, 'w') as lock_file:
            lock_file.write('1')
        # 이미지 파일 저장
        cv2.imwrite(tmp_file, frame)
        # 잠금 파일 삭제
        os.remove(tmp_lock_file)
        # break whan keyinturrupt occurs
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            
            
        # break whan keyinturrupt occurs
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            
            
            
    
    def publish_target_image(self, frame):
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        target_msg = YoloTarget()
        target_msg.image = image_msg
        
        timestamp = int(time.time())
        file_path = os.path.join(self.target_capture_folder, f'target_{timestamp}.jpg')
        cv2.imwrite(file_path, frame)
        
        # publish the target image
        #self.publisher_target.publish(target_msg)
        self.get_logger().info(f"Target image published and saved to {file_path}")
    
    

    def phase_callback(self, msg):
        # get vehicle phase
        self.phase = msg.phase




def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
