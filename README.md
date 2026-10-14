# TurtleBot 4 — SLAM và Nav2

Mô phỏng TurtleBot 4 trong world `maze`: tạo bản đồ bằng SLAM Toolbox (kết hợp điều khiển bằng `teleop` hoặc tự hành `auto_explore`), lưu map, sau đó định vị bằng AMCL và điều hướng bằng Nav2.

## Cấu trúc thư mục

```text
tht4_tb4_ws/
└── src/
    └── tht4_tb4_nav/
        ├── config/          # Cau hinh SLAM (slam.yaml)
        ├── launch/          # File launch tich hop (bringup.launch.py)
        ├── maps/            # Ban do da luu (map.pgm, map.yaml)
        ├── rviz/            # Cau hinh hien thi RViz2 (maze.rviz)
        └── tht4_tb4_nav/    # Node Python (auto_explore.py, avoidance.py)
```

---

## 1. Tải mã nguồn, cài đặt và build

Yêu cầu hệ thống: **Ubuntu 22.04**, **ROS 2 Humble**, **TurtleBot 4 Ignition Simulator** và **Nav2**.

Mở terminal và chạy các lệnh sau để tải dự án từ GitHub về thư mục `~` (Home) và biên dịch:

```bash
# 1. Tai kho chua ve may
cd ~
git clone https://github.com/tuanvu162/tht4_tb4_ws.git

# 2. Di chuyen vao workspace va cai dat cac goi phu thuoc
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3. Build package va nap moi truong
colcon build --packages-select tht4_tb4_nav
source install/setup.bash
```

---

## 2. Mapping (Tạo bản đồ SLAM)

**Bước 2.1 — Khởi chạy Gazebo, RViz2 và SLAM (Terminal 1):**

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=mapping
```

*(Nếu muốn mở cấu hình RViz2 đã lưu sẵn của dự án ở một terminal riêng: `rviz2 -d ~/tht4_tb4_ws/src/tht4_tb4_nav/rviz/maze.rviz`)*

**Bước 2.2 — Rời đế sạc / Undock (Terminal 2):**

Chờ robot xuất hiện và Gazebo đang ở trạng thái Play, chạy lệnh:

```bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

**Bước 2.3 — Điều khiển robot quét bản đồ (Chọn 1 trong 2 cách tại Terminal 2):**

Sau khi terminal báo `SUCCEEDED`, điều khiển robot bằng bàn phím:

* **Điều khiển bằng bàn phím (Teleop):**
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  Giữ terminal teleop được chọn: `i` tiến, `,` lùi, `j/l` xoay, `k` dừng.



Khảo sát phần lớn mê cung, theo dõi map mở rộng trên RViz2 (Fixed Frame là `map`, Durability Policy của `/map` là `Transient Local`).  
Có thể kiểm tra tần số `/scan` và `/odom` bằng `ros2 topic hz /scan` và `ros2 topic hz /odom`.

---

## 3. Lưu bản đồ

Khi bản đồ đã quét kín mê cung, nhấn `k` để dừng robot và bấm `Ctrl + C` tại cửa sổ điều khiển (giữ Terminal 1 đang chạy SLAM). Tại Terminal 2, chạy lệnh lưu bản đồ:

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/tht4_tb4_ws/src/tht4_tb4_nav/maps/map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

Kiểm tra thư mục `maps/` đã có đủ 2 file `map.yaml` và `map.pgm`, bản đồ không bị nhân đôi tường hoặc lệch rõ rệt. Sau đó bấm `Ctrl + C` ở Terminal 1 và đóng Gazebo cũ.

---

## 4. Localization và Navigation (AMCL & Nav2)

**Bước 4.1 — Khởi động lại với bản đồ đã lưu, AMCL và Nav2 (Terminal 1):**

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=navigation \
  map:=$HOME/tht4_tb4_ws/src/tht4_tb4_nav/maps/map.yaml
```

**Bước 4.2 — Đặt vị trí ban đầu và gửi điểm đích trên RViz2:**
1. Mở khóa robot khỏi trạm sạc (nếu cần):
   ```bash
   ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
   ```
2. Trên thanh công cụ RViz2, chọn **2D Pose Estimate**, nhấp và kéo mũi tên tại vị trí thực tế của robot trong mê cung để khởi tạo AMCL.
3. Chọn công cụ **Nav2 Goal**, nhấp vào vị trí đích trên bản đồ để robot tự động lập quỹ đạo và di chuyển tới đích.
