#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # 1. Khởi tạo bộ chuyển đổi CvBridge
        self.bridge = CvBridge()
        
        # 2. Đăng ký nhận dữ liệu (Subscribe) từ topic camera của TurtleBot 4
        self.camera_topic = '/oakd/rgb/preview/image_raw'
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10 # QoS Profile
        )
        
        # 3. Cấu hình thư viện ArUco (Sử dụng API tương thích OpenCV < 4.7)
        try:
            # Dành cho bản OpenCV rất cũ
            self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            # Fallback nếu hàm _get bị deprecate trong bản giao thời
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_parameters = cv2.aruco.DetectorParameters()
            
        self.get_logger().info(f'Node ArUco Detector đã khởi động, đang lắng nghe topic: {self.camera_topic}')

    def image_callback(self, msg):
        try:
            # Chuyển đổi dữ liệu từ ROS Image sang OpenCV Image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Lỗi chuyển đổi ảnh: {e}')
            return

        # Hàm detectMarkers yêu cầu ảnh xám để tối ưu hóa tính toán
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Tiến hành nhận diện mã ArUco (Sử dụng API cũ)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_image, 
            self.aruco_dictionary, 
            parameters=self.aruco_parameters
        )

        # Nếu phát hiện thấy có mã ArUco trong khung hình
        if ids is not None:
            self.get_logger().info(f'Tìm thấy {len(ids)} mã ArUco! ID: {ids.flatten()}')
            
            # Vẽ khung và ID của ArUco lên bức ảnh màu
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # In tọa độ đỉnh đầu tiên của mã ArUco để theo dõi
            for i in range(len(ids)):
                topLeftCorner = corners[i][0][0]
                self.get_logger().info(f'Mã ID {ids[i][0]} tọa độ đỉnh: {topLeftCorner}')

        # Hiển thị cửa sổ hình ảnh trực quan
        cv2.imshow("TurtleBot 4 Camera Feed - ArUco Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Đang tắt Node...')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()