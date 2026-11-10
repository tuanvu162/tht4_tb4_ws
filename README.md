# Bài TH Tuần 5 — TurtleBot 4 Navigation và YOLO

Nhánh **`BaiTHTuan5`** — hướng dẫn riêng cho bài thực hành tuần 5.

Robot định vị trên bản đồ cũ bằng AMCL, tự đi qua ít nhất 3 goal bằng Nav2
và nhận diện vật thể từ camera RGB bằng YOLOv8n.

## 1. Chuẩn bị và build

Yêu cầu Ubuntu 22.04, ROS 2 Humble và TurtleBot 4 Simulation đã cài từ bài trước.
Dùng cả hai package `tht4_tb4_nav` và `tht5_tb4_perception` trong repo này.
Bản đồ world `maze` nằm tại [map.yaml](src/tht4_tb4_nav/maps/map.yaml) và
[map.pgm](src/tht4_tb4_nav/maps/map.pgm).

```bash
sudo apt install ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge python3-pip
/usr/bin/python3 -m pip install --user torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
/usr/bin/python3 -m pip install --user 'numpy<2' 'opencv-python>=4.6,<4.12' ultralytics

cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/tht4_tb4_nav src/tht5_tb4_perception --ignore-src -r -y
PYTHONNOUSERSITE=1 colcon build \
  --packages-select tht4_tb4_nav tht5_tb4_perception --symlink-install

mkdir -p models
cd models
/usr/bin/python3 -c 'from ultralytics import YOLO; YOLO("yolov8n.pt")'
```

Model được lưu tại `~/tht4_tb4_ws/models/yolov8n.pt`, theo `config/yolo.yaml`.
Lần đầu tải model cần mạng.

## 2. Mở mô phỏng và định vị

Chạy phần thiết lập này trong **mỗi terminal ROS**:

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
```

**Terminal 1:**

```bash
ros2 launch tht4_tb4_nav bringup.launch.py mode:=navigation \
  map:=$HOME/tht4_tb4_ws/src/tht4_tb4_nav/maps/map.yaml
```

Trong RViz, đặt **Fixed Frame = map**, dùng **2D Pose Estimate** đặt đúng vị trí
và hướng robot; kiểm tra `/scan` khớp tường. Gazebo phải ở trạng thái Play,
robot sẵn sàng di chuyển.

Nếu dùng GPU NVIDIA, chạy các lệnh sau trước khi mở mô phỏng:

```bash
unset LIBGL_ALWAYS_SOFTWARE
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export QT_QPA_PLATFORM=xcb
```

## 3. Lấy tọa độ và chạy Navigation

Dùng **Publish Point** trên RViz và đọc tọa độ:

```bash
ros2 topic echo /clicked_point
```

Lưu ít nhất 3 đích vào [config/goals.yaml](src/tht5_tb4_perception/config/goals.yaml): `goal_x`,
`goal_y` tính bằng mét trong frame `map`, `goal_yaw` tính bằng radian.
Publish Point chỉ cung cấp vị trí; chọn yaw theo hướng muốn robot tới đích.

**Terminal 2:**

```bash
ros2 run tht5_tb4_perception waypoint_navigation --ros-args \
  --params-file "$PWD/src/tht5_tb4_perception/config/goals.yaml"
```

Node dùng `nav2_simple_commander` gửi lần lượt **Goal 1 → Goal 2 → Goal 3** qua
`NavigateToPose`. Chỉ gửi đích tiếp theo khi đích trước `SUCCEEDED`; dừng tuyến
nếu thất bại, timeout hoặc LiDAR không hợp lệ. Không dùng teleop hoặc chuột đặt
goal khi node đang chạy. Nhấn Ctrl+C để yêu cầu hủy goal.

## 4. Chạy YOLO song song

**Terminal 3:** mở YOLO trước hoặc trong khi chạy waypoint:

```bash
ros2 launch tht5_tb4_perception tht5.launch.py spawn_chair:=false
```

Trong RViz, thêm **Image**, chọn `/yolo/image`. Node đọc
`/oakd/rgb/preview/image_raw`, dùng `cv_bridge` chuyển sang OpenCV, chạy YOLO,
vẽ bounding box/class/confidence và in kết quả ra terminal. Xem JSON bằng:

```bash
ros2 topic echo /yolo/detections std_msgs/msg/String
```

Cần có vật thể như ghế/người trong tầm camera; model COCO không nhận diện tường.
Nếu muốn launch tự thêm ghế mẫu, tải model:

```bash
ign fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair
```

Sửa `chair.model_file` trong [config/scene_objects.yaml](src/tht5_tb4_perception/config/scene_objects.yaml)
cho đúng file `model.sdf` đã tải, rồi chạy launch không có `spawn_chair:=false`.
Tham số camera, model và ngưỡng nhận diện nằm trong [config/yolo.yaml](src/tht5_tb4_perception/config/yolo.yaml).

## 5. Nội dung nộp

- Video Google Drive công khai: load map, localization, 3 goal thành công không
  va chạm và YOLO chạy song song có bounding box/class/confidence.
- Repo Public chứa code, launch, cấu hình, bản đồ và README này.
- PDF ngắn: cấu trúc và luồng chương trình, node/topic/action, cách AMCL/Nav2
  hoạt động, ảnh kết quả và link video/GitHub.

**Video demo:** chưa bổ sung. **Báo cáo PDF:** chưa bổ sung.
