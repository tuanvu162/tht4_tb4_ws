# TurtleBot 4: mapping, localization và Nav2

Workspace ROS 2 Humble sử dụng world `maze` có sẵn của TurtleBot 4.
Launch có hai chế độ: `mapping` (SLAM + teleop) và `navigation`
(map server + AMCL + Nav2). Cả hai mở Gazebo và RViz với cấu hình đã lưu.
Node `auto_explore` là phần thử nghiệm trước đó, không được launch trong quy trình bài tập.

## Cài đặt và build

Cần Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Ignition simulator và navigation packages.
Cài các dependency đã khai báo trong package rồi build:

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tht4_tb4_nav
source install/setup.bash
```

Mỗi terminal mới cần source ROS; terminal chạy launch cần source workspace.

## 1. Simulation và mapping bằng teleop

Terminal 1:

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=mapping
```

Chờ robot xuất hiện và Gazebo đang Play. Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Chờ undock báo SUCCEEDED trước khi teleop. Dùng phím theo hướng dẫn của teleop,
giữ terminal teleop được chọn; đi chậm qua phần lớn hành lang và khu vực trống.
Theo dõi Map, LaserScan và RobotModel trong RViz (Fixed Frame: `map`).
Nếu robot không hiển thị, đặt RobotModel/Description Topic/Durability Policy
thành Transient Local. Map dùng `/map`; LaserScan dùng `/scan`.

Terminal kiểm tra riêng:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /scan
# Ctrl+C rồi kiểm tra odometry:
ros2 topic hz /odom
```

Xác nhận vận tốc teleop làm robot di chuyển. Kiểm tra map thể hiện tường,
hành lang, vật cản và không bị nhân đôi/lệch rõ rệt.

## 2. Lưu bản đồ

Dừng robot và teleop bằng Ctrl+C, giữ SLAM chạy. Trong terminal 2:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/tht4_tb4_ws/src/tht4_tb4_nav/maps/map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

Kiểm tra có cả `src/tht4_tb4_nav/maps/map.yaml` và `map.pgm`.
Đây phải là map tự tạo, không dùng bản đồ mẫu để thay kết quả mapping.
Sau đó Ctrl+C terminal 1 để dừng SLAM, RViz và mô phỏng; đóng Gazebo nếu cửa sổ còn mở.

## 3. Khởi động lại với localization và Nav2

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=navigation \
  map:=$HOME/tht4_tb4_ws/src/tht4_tb4_nav/maps/map.yaml
```

Chế độ này không chạy SLAM hoặc auto_explore. AMCL định vị trên bản đồ đã lưu;
Nav2 sử dụng map đó qua map server/global costmap.
Trong RViz dùng **2D Pose Estimate** đặt vị trí và hướng ban đầu tương ứng
với robot trong Gazebo. Rời đế bằng lệnh `/undock` ở terminal 2 như trên.
Có thể di chuyển nhẹ bằng teleop trước khi đặt goal để AMCL hội tụ;
sau đó dừng teleop. So sánh vị trí/hướng robot và LaserScan với tường trên map.

Đặt ít nhất **3 Nav2 Goal** tại các vùng trống khác nhau của maze.
Chờ mỗi goal thành công trước khi chọn goal tiếp theo. Không điều khiển thủ công
sau khi đặt goal; quan sát đường đi và xác nhận không va chạm.

## RViz configuration

File nguồn: `src/tht4_tb4_nav/rviz/maze.rviz`.
Trong RViz dùng File > Save Config As để lưu vào file nguồn rồi build lại.
Cũng có thể nạp trực tiếp file nguồn khi launch:

```bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=mapping \
  rviz_config:=$HOME/tht4_tb4_ws/src/tht4_tb4_nav/rviz/maze.rviz
```

## Các thành phần chính

| Thành phần | Vai trò |
| --- | --- |
| turtlebot4_ignition_bringup | Gazebo, robot, bridge sensor và odometry |
| slam_toolbox | Tạo bản đồ trong chế độ mapping |
| teleop_twist_keyboard | Điều khiển thủ công khi mapping |
| nav2_map_server | Load bản đồ đã lưu |
| nav2_amcl | Localization trên bản đồ cố định |
| Nav2 | Planner, controller, costmap và thực thi navigation goal |
| rviz2 | Hiển thị và đặt initial pose/navigation goal |

Topic chính: `/scan`, `/odom`, `/cmd_vel`, `/map`, `/tf`, `/tf_static`,
`/amcl_pose`, `/initialpose`. Action: `/undock`, `/navigate_to_pose`.
Nav2 mặc định của TurtleBot 4 sử dụng NavFn với `use_astar: false` (Dijkstra)
để lập đường và DWB để chọn vận tốc bám đường/tránh vật cản cục bộ.

## Minh chứng và bài nộp

Build/kiểm tra launch không chứng minh robot đã hoàn thành bài tập.
Cần tự chạy và ghi video mapping, map hoàn chỉnh, khởi động lại/load map,
localization và ít nhất 3 navigation goal thành công không va chạm.
Nộp video Drive công khai, repository GitHub Public chứa source/launch/map/config/README,
và PDF mô tả chương trình, node/topic, mapping, localization, Nav2, ảnh kết quả,
câu trả lời phần "Nội dung cần hiểu", cùng link video và repository.

## Lỗi multicast và SLAM chậm

`Exception sending a multicast message: Network is unreachable` là lỗi mạng
Gazebo Transport, không phải thông báo hết RAM. Launch mặc định đặt
`IGN_IP=127.0.0.1` cho các tiến trình Gazebo/bridge cùng máy, để không chọn
địa chỉ Wi-Fi có thể mất kết nối hoặc thay đổi khi đang chạy. Cần dừng và khởi
động lại toàn bộ phiên mô phỏng để môi trường mới có hiệu lực.
Nếu dùng mô phỏng trên nhiều máy, chỉ định `ign_ip:=<IP interface cần dùng>`
và giữ interface đó hoạt động.

Mapping sử dụng async SLAM Toolbox thay vì hàng đợi xử lý sync. Cấu hình
`src/tht4_tb4_nav/config/slam.yaml` đặt ngưỡng chuyển động 0.20 m và góc
0.15 rad (~8.6 độ), cập nhật occupancy map mỗi 2 giây và tắt cache tương tác.
LiDAR vẫn publish liên tục; các ngưỡng này giảm scan được đưa vào SLAM,
không tắt cảm biến khi robot đứng yên. Chúng giúp giảm tải, không đặt giới hạn
cứng cho tổng bộ nhớ pose graph hoặc bảo đảm hết mọi lỗi TF.

Nếu robot dừng, kiểm tra Gazebo đang Play, `/scan`, `/odom`, `/clock` còn
cập nhật và teleop đang gửi `/cmd_vel`. Nếu SLAM báo `Queue size has grown`
hoặc `extrapolation into the past`, dừng robot để xử lý kịp và kiểm tra TF;
không chỉ tăng TF buffer để che hàng đợi tồn đọng.
