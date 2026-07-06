import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys

class ImageSaver(Node):

    def __init__(self, frame_number):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.frame_number = frame_number
        self.width = 1280
        self.height = 720
        self.saved = False

    def listener_callback(self, msg):
        if not self.saved:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize the image to the desired resolution
            cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Save the image with the specified frame number
            image_filename = f'./Results/image_{self.frame_number:04d}.png'
            if cv2.imwrite(image_filename, cv_image):
                self.get_logger().info(f'Successfully saved "{image_filename}"')
                self.saved = True
                # Force exit the program
                sys.exit(0)
            else:
                self.get_logger().error(f'Failed to save "{image_filename}"')

def main(args=None):
    rclpy.init(args=args)

    frame_number = int(input("Enter the frame number: "))

    image_saver = ImageSaver(frame_number)

    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
