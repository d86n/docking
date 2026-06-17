import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class AutoDocking(Node):
    def __init__(self):
        super().__init__('auto_docking')
        
        # Subscriber lấy hình ảnh và thông số calibration của camera từ các topic thực tế của bạn
        self.img_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/oakd/rgb/preview/camera_info', self.camera_info_callback, 10)
        
        # Publisher gửi lệnh di chuyển (Dùng TwistStamped do cấu hình hệ thống của bạn yêu cầu stamped:=true)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Khởi tạo ma trận camera
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # --- CẤU HÌNH THÔNG SỐ PHẦN CỨNG ---
        self.marker_length = 0.10  # Kích thước thực tế của mã ArUco (Ví dụ: 0.10 mét = 10cm)
        
        # Tính toán khoảng cách đích (target_distance):
        # Mã ArUco nằm sau dock sạc 10cm (0.1m). 
        # Nếu khi robot chạm vào dock sạc, khoảng cách từ CAMERA đến mã ArUco là bao nhiêu thì điền vào đây.
        # Ví dụ: Nếu camera cách mũi robot 15cm + 10cm khoảng cách đến mã = 0.25m.
        self.target_distance = 0.25  
        
        # Hệ số tỷ lệ bộ điều khiển P (Thay đổi để robot đi mượt hơn)
        self.kp_linear = 0.4
        self.kp_angular = 1.2
        
        # Giới hạn vận tốc tối đa để đảm bảo an toàn
        self.max_linear_vel = 0.15   # m/s
        self.max_angular_vel = 0.5  # rad/s

    def camera_info_callback(self, msg):
        # Lấy thông số cấu hình camera tự động từ topic camera_info
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Đã tải thành công thông số cấu hình Camera.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Lỗi chuyển đổi ảnh: {e}')
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        # Khởi tạo tin nhắn điều khiển có Header (TwistStamped)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        if ids is not None:
            # Ước lượng vị trí 3D của mã ArUco đầu tiên nhận diện được
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            
            # tvecs chứa tọa độ [X, Y, Z] của mã so với Camera frame
            # Trong hệ tọa độ OpenCV camera: Z là trục hướng thẳng phía trước, X là trục ngang phải
            x = tvecs[0][0][0]
            z = tvecs[0][0][2]
            
            # Tính toán sai số khoảng cách và sai số góc độ
            current_distance = math.sqrt(x**2 + z**2)
            distance_error = current_distance - self.target_distance
            angle_error = math.atan2(x, z) # Góc lệch hướng tới mã
            
            # Vòng điều khiển kín P-Controller
            if distance_error > 0.02: # Nếu cách vị trí đích lớn hơn 2cm
                # Tính vận tốc thẳng
                linear_vel = self.kp_linear * distance_error
                linear_vel = min(linear_vel, self.max_linear_vel)
                
                # Tính vận tốc góc để bám tâm mã ArUco
                angular_vel = self.kp_angular * angle_error
                angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
                
                # Gán giá trị điều khiển
                twist_msg.twist.linear.x = linear_vel
                twist_msg.twist.angular.z = angular_vel
                
                self.get_logger().info(f'Đang tiến vào dock - Khoảng cách còn lại: {distance_error:.2f}m, Góc lệch: {math.degrees(angle_error):.1f}°')
            else:
                # Đã đến vị trí sạc an toàn, dừng robot
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.0
                self.get_logger().info('Đã docking thành công vào trạm sạc!')
        else:
            # Nếu mất dấu mã ArUco, dừng robot ngay lập tức để tránh va chạm
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            self.get_logger().warn('Mất dấu mã ArUco! Robot tạm dừng để đảm bảo an toàn.')

        # Publish lệnh điều khiển xuống robot
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDocking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()