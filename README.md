Lệnh build lại package:
colcon build --packages-select [tên package] --symlink-install
source install/setup.bash

Chạy teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
