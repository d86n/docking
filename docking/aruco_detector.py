import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Đăng ký Subscriber lắng nghe topic camera
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw', 
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # CÚ PHÁP CŨ: Khởi tạo dictionary và thông số detector trực tiếp
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create() # Thay đổi ở đây

        self.get_logger().info('Node nhận diện ArUco (Cú pháp cũ) đã khởi động...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Không thể chuyển đổi hình ảnh: {e}')
            return

        # CÚ PHÁP CŨ: Gọi hàm detectMarkers trực tiếp từ module cv2.aruco
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i in range(len(ids)):
                self.get_logger().info(f'Phát hiện ArUco ID: {ids[i][0]}')
        
        cv2.imshow("ArUco Detection Camera", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()