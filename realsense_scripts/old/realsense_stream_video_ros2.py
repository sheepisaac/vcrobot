import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys
import os
from datetime import datetime

class VideoSaver(Node):
    def __init__(self, width, height):
        super().__init__('video_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.width = width
        self.height = height
        self.yuv_file = None

        # 현재 날짜 기반 디렉토리 생성
        today = datetime.today().strftime('%Y%m%d')
        directory = f'./Results/{today}/'
        if not os.path.exists(directory):
            os.makedirs(directory)

        # 지정된 이름으로 YUV 파일 생성 및 열기
        yuv_filename = f'{directory}output_video_{width}x{height}_yuv420.yuv'
        self.yuv_file = open(yuv_filename, 'wb')
        self.get_logger().info(f'Opened "{yuv_filename}" for writing.')

        # OpenCV 창 설정
        cv2.namedWindow("Live Stream", cv2.WINDOW_NORMAL)

    def listener_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 원하는 해상도로 이미지 크기 조정
        cv_image = cv2.resize(cv_image, (self.width, self.height))

        # BGR에서 YUV420으로 변환
        yuv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV_I420)

        # YUV 이미지 파일에 저장
        self.yuv_file.write(yuv_image.tobytes())
        self.get_logger().info('Saved a frame.')

        # OpenCV 창에 이미지 표시
        cv2.imshow("Live Stream", cv_image)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC 키를 누르면 종료
            self.get_logger().info('ESC pressed. Closing file and exiting.')
            self.yuv_file.close()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)

    def destroy_node(self):
        if self.yuv_file and not self.yuv_file.closed:
            self.yuv_file.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # 사용자로부터 해상도 입력 받기
    try:
        width = int(input("Enter the desired video width: "))
        height = int(input("Enter the desired video height: "))
    except ValueError:
        print("Invalid input. Please enter integer values for width and height.")
        sys.exit(1)

    video_saver = VideoSaver(width, height)

    try:
        rclpy.spin(video_saver)
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt (CTRL+C) received. Exiting...")
    finally:
        if video_saver.yuv_file and not video_saver.yuv_file.closed:
            video_saver.yuv_file.close()
        cv2.destroyAllWindows()
        video_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
