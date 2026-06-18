import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class DockingState:
    DETECT = 'DETECT'                       
    PRE_ALIGN_NAV = 'PRE_ALIGN_NAV'         
    PRE_ALIGN_TURN = 'PRE_ALIGN_TURN'       
    FINAL_APPROACH = 'FINAL_APPROACH'       
    STOP = 'STOP'

class AutoDocking(Node):
    def __init__(self):
        super().__init__('auto_docking')
        
        self.img_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/oakd/rgb/preview/camera_info', self.camera_info_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.28
        
        # --- THÔNG SỐ CƠ KHÍ & QUỸ ĐẠO BÙ TRỪ ---
        self.camera_x_offset = 0.10  # Khoảng cách từ tâm robot đến ống kính camera (mét)
        self.pre_align_dist = 0.60   # Điểm neo chờ tiền docking cách dock 60cm
        self.final_dock_dist = 0.45  # Điểm dừng cuối cùng cách dock 25cm
        
        self.kp_linear = 0.4
        self.kp_angular = 1.2
        self.max_linear_vel = 0.15
        self.max_angular_vel = 0.5
        
        self.state = DockingState.DETECT
        self.odom_ready = False
        
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_yaw = 0.0
        
        self.pre_align_x = 0.0
        self.pre_align_y = 0.0
        self.final_dock_x = 0.0
        self.final_dock_y = 0.0
        self.robot_target_yaw = 0.0  

        self.detect_count = 0
        self.sum_x = 0.0
        self.sum_z = 0.0
        self.sum_nx = 0.0  
        self.sum_nz = 0.0  
        self.required_samples = 20 

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.state != DockingState.DETECT: return
        if not self.odom_ready or self.camera_matrix is None: return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            
            x = tvecs[0][0][0]
            z = tvecs[0][0][2]
            
            rmat, _ = cv2.Rodrigues(rvecs[0][0])
            nx = rmat[0, 2]
            nz = rmat[2, 2]
            
            if z <= 0.1 or z > 6.0: return

            self.detect_count += 1
            self.sum_x += x
            self.sum_z += z
            self.sum_nx += nx
            self.sum_nz += nz
            
            if self.detect_count >= self.required_samples:
                avg_x = self.sum_x / self.required_samples
                avg_z = self.sum_z / self.required_samples
                avg_nx = self.sum_nx / self.required_samples
                avg_nz = self.sum_nz / self.required_samples
                
                # Bù sai số cơ khí vị trí đặt camera trên robot
                avg_z += self.camera_x_offset
                
                distance_to_marker = math.hypot(avg_x, avg_z)
                angle_to_marker = -math.atan2(avg_x, avg_z) 
                
                # Xác định vị trí tuyệt đối của mã ArUco trên bản đồ odom
                marker_odom_x = self.current_odom_x + distance_to_marker * math.cos(self.current_odom_yaw + angle_to_marker)
                marker_odom_y = self.current_odom_y + distance_to_marker * math.sin(self.current_odom_yaw + angle_to_marker)
                
                # --- THUẬT TOÁN ĐỒNG BỘ TRỤC PHÁP TUYẾN CHỐNG LỆCH GÓC VUÔNG ---
                psi = self.current_odom_yaw
                # Chiếu vector pháp tuyến từ hệ quang học camera sang hệ base_link của robot
                x_rob_normal = avg_nz
                y_rob_normal = -avg_nx
                
                # Quay hình học hệ base_link sang hệ odom toàn cục toàn phần
                x_odom_normal = x_rob_normal * math.cos(psi) - y_rob_normal * math.sin(psi)
                y_odom_normal = x_rob_normal * math.sin(psi) + y_rob_normal * math.cos(psi)
                
                # Tính góc hướng tuyệt đối của mặt phẳng Dock sạc
                dock_yaw_odom = math.atan2(y_odom_normal, x_odom_normal)
                
                # Robot cần hướng đầu ngược khít với hướng vector pháp tuyến của Dock để đảm bảo vuông góc
                self.robot_target_yaw = self.normalize_angle(dock_yaw_odom + math.pi)
                
                # Khởi tạo tọa độ các Waypoint dọc theo đúng trục pháp tuyến chuẩn vừa tính
                self.pre_align_x = marker_odom_x + self.pre_align_dist * math.cos(dock_yaw_odom)
                self.pre_align_y = marker_odom_y + self.pre_align_dist * math.sin(dock_yaw_odom)
                
                self.final_dock_x = marker_odom_x + self.final_dock_dist * math.cos(dock_yaw_odom)
                self.final_dock_y = marker_odom_y + self.final_dock_dist * math.sin(dock_yaw_odom)
                
                self.get_logger().info('=========================================')
                self.get_logger().info(f'>>> CHỐT QUỸ ĐẠO VUÔNG GÓC HOÀN HẢO!')
                self.get_logger().info(f'>>> Target Yaw chuẩn: {math.degrees(self.robot_target_yaw):.2f} độ')
                self.get_logger().info('=========================================')
                
                self.state = DockingState.PRE_ALIGN_NAV
                self.destroy_subscription(self.img_sub)

    def odom_callback(self, msg):
        self.odom_ready = True
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        linear_vel = 0.0
        angular_vel = 0.0

        if self.state == DockingState.PRE_ALIGN_NAV:
            dx = self.pre_align_x - self.current_odom_x
            dy = self.pre_align_y - self.current_odom_y
            dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_err = self.normalize_angle(target_angle - self.current_odom_yaw)
            
            if dist < 0.04:
                self.state = DockingState.PRE_ALIGN_TURN
                self.get_logger().info('[ODOM] Đã đạt vị trí tiền docking. Chuyển sang pha xoay chỉnh vuông góc...')
            else:
                if abs(math.degrees(angle_err)) > 8.0:
                    angular_vel = self.kp_angular * angle_err  
                else:
                    linear_vel = self.kp_linear * dist         
                    angular_vel = self.kp_angular * angle_err  

        elif self.state == DockingState.PRE_ALIGN_TURN:
            angle_err = self.normalize_angle(self.robot_target_yaw - self.current_odom_yaw)
            
            if abs(math.degrees(angle_err)) < 1.0: 
                self.state = DockingState.FINAL_APPROACH
                self.get_logger().info('[ODOM] Đã vuông góc hoàn toàn. Bắt đầu tiến thẳng ổn định quỹ đạo...')
            else:
                angular_vel = self.kp_angular * angle_err 

        elif self.state == DockingState.FINAL_APPROACH:
            dx = self.final_dock_x - self.current_odom_x
            dy = self.final_dock_y - self.current_odom_y
            dist = math.hypot(dx, dy)
            
            target_angle = math.atan2(dy, dx)
            angle_err = self.normalize_angle(target_angle - self.current_odom_yaw)
            
            if dist < 0.12:
                angle_err = self.normalize_angle(self.robot_target_yaw - self.current_odom_yaw)
            
            if dist < 0.05:
                self.state = DockingState.STOP
                self.get_logger().info('>>> HỆ THỐNG DOCKING THÀNH CÔNG VÀ VUÔNG GÓC TUYỆT ĐỐI! <<<')
            else:
                linear_vel = min(0.06, self.kp_linear * dist) 
                angular_vel = self.kp_angular * angle_err

        elif self.state == DockingState.STOP:
            pass 

        twist_msg.twist.linear.x = min(max(linear_vel, -self.max_linear_vel), self.max_linear_vel)
        twist_msg.twist.angular.z = min(max(angular_vel, -self.max_angular_vel), self.max_angular_vel)
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