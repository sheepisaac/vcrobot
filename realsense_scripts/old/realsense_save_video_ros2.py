import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys
import numpy as np
import os
from datetime import datetime
import time

class VideoSaver(Node):

    def __init__(self, frame_count, width, height, fps):
        super().__init__('video_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.frame_count = frame_count
        self.current_frame = 0
        self.width = width
        self.height = height
        self.fps = fps
        self.yuv_file = None

        # Create directory based on the current date
        today = datetime.today().strftime('%Y%m%d')
        directory = f'./Results/{today}/'
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Create and open the YUV file with the specified name
        yuv_filename = f'{directory}output_video_{width}x{height}_yuv420.yuv'
        self.yuv_file = open(yuv_filename, 'wb')
        self.get_logger().info(f'Opened "{yuv_filename}" for writing.')

        # Set the delay between frames based on the FPS
        self.frame_delay = 1.0 / fps
        self.last_frame_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time()
        if self.current_frame < self.frame_count and (current_time - self.last_frame_time) >= self.frame_delay:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize the image to the desired resolution
            cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Convert BGR to YUV
            yuv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV_I420)
            
            # Write YUV image to file
            self.yuv_file.write(yuv_image.tobytes())
            self.get_logger().info(f'Saved frame {self.current_frame+1} of {self.frame_count}')
            
            self.current_frame += 1
            self.last_frame_time = current_time

        if self.current_frame >= self.frame_count:
            self.yuv_file.close()
            self.get_logger().info('Finished saving video. Closing file.')
            sys.exit(0)  # Exit the program after saving the specified number of frames

def main(args=None):
    rclpy.init(args=args)

    # Get frame count, resolution, and FPS from the user
    frame_count = int(input("Enter the number of frames to capture: "))
    width = int(input("Enter the desired video width: "))
    height = int(input("Enter the desired video height: "))
    fps = int(input("Enter the desired FPS: "))

    video_saver = VideoSaver(frame_count, width, height, fps)

    try:
        rclpy.spin(video_saver)
    except KeyboardInterrupt:
        pass
    finally:
        if video_saver.yuv_file and not video_saver.yuv_file.closed:
            video_saver.yuv_file.close()
        video_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
