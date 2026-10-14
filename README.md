# TurtleBot 4 — SLAM và Nav2

Mô phỏng TurtleBot 4 trong world `maze`: tạo bản đồ bằng SLAM Toolbox và
teleop, lưu map, sau đó localization bằng AMCL và điều hướng bằng Nav2.

## 1. Cài đặt và build

Yêu cầu Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Ignition Simulator và Nav2.

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tht4_tb4_nav
source install/setup.bash
```

## 2. Mapping bằng teleop

Terminal 1 — mở Gazebo, RViz và SLAM:

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=mapping
```

Chờ robot xuất hiện và Gazebo đang Play. Terminal 2 — rời đế sạc:

```bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

Chờ `SUCCEEDED`, rồi mở teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Giữ terminal teleop được chọn: `i` tiến, `,` lùi, `j/l` xoay, `k` dừng.
Khảo sát phần lớn maze, theo dõi map trên RViz; Fixed Frame là `map`.
Kiểm tra `/scan` và `/odom` bằng `ros2 topic hz /scan` và `ros2 topic hz /odom`.

## 3. Lưu bản đồ

Nhấn `k`, Ctrl+C teleop; giữ SLAM chạy. Trong terminal 2:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/tht4_tb4_ws/src/tht4_tb4_nav/maps/map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

Kiểm tra có `maps/map.yaml` và `maps/map.pgm`, map không bị nhân đôi tường
hoặc lệch rõ rệt. Sau đó Ctrl+C terminal 1 và đóng Gazebo cũ.

## 4. Localization và navigation

Terminal 1 — khởi động lại với map đã lưu, AMCL và Nav2:

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=navigation \
  map:=$HOME/tht4_tb4_ws/src/tht4_tb4_nav/maps/map.yaml
```
