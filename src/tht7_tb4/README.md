# Bài thực hành 7 — Phân vùng bản đồ và điều hướng bằng giọng nói

Package ROS 2 Humble `tht7_tb4`, dùng mô phỏng TurtleBot 4 world `maze` và ảnh
bản đồ của `tht4_tb4_nav`. Không cần YOLO hoặc chạy node waypoint của bài 5.

## Thành phần và kết quả

```text
Micro / file âm thanh → Whisper tiếng Việt → /voice_command (std_msgs/String)
                                              ↓
                        Phân tích câu + config/regions.yaml
                                              ↓
                           /navigate_to_pose (NavigateToPose)
                                              ↓
                           Nav2 → robot tránh vật cản, đến đích

map.pgm + config/map.yaml → phân vùng ô trống → ảnh PNG + nhãn RViz
```

- `config/regions.yaml`: A = Phòng học, B = Phòng thí nghiệm, C = Hành lang;
  các tên là nhãn chức năng minh họa cho maze, có thể đổi tên và aliases.
- `segmentation.py`: BFS nhiều nguồn trên các ô trống, chia theo khoảng cách
  đường đi 4 hướng, không xuyên tường. Ba đích bài 5 là hạt giống. Đây là phân
  vùng có hạt giống, không phải mô hình tự nhận ra loại phòng.
- `results/segmented_map.png`: bản đồ 3 màu, nhãn A/B/C và điểm đích.
- `results/region_labels.png`: ảnh uint16, 0 = tường/chưa biết/chưa gán,
  1/2/3 = A/B/C; lưu toàn bộ ranh giới vùng thay vì chỉ lưu 3 điểm.
- `results/segmentation.yaml`: tên, diện tích, tọa độ đích, khoảng cách tới
  vật cản và SHA256 bản đồ dùng để tạo kết quả.
- `region_markers`: hiển thị các ô vùng, tên và hướng đích trên RViz. Nhãn
  dùng chữ không dấu (`Phong hoc`, `Phong thi nghiem`, `Hanh lang`) để tránh
  lỗi font RViz; ID và tên đặt trên hai dòng, lệch khỏi mũi tên đích.
  Tên tiếng Việt trong YAML và nhận lệnh giọng nói vẫn giữ nguyên.
- `voice_navigation`: xử lý lệnh, theo dõi kết quả, timeout và hủy tuyến.
- `speech_input`: nhấn Enter để thu một câu bằng micro; nhận dạng Whisper
  chạy cục bộ, không cần API key. `send_command` để thử riêng phần điều hướng.

`config/map.pgm` là bản sao ảnh THT4 hiện tại để package cài đặt độc lập.
`config/map.yaml` tham chiếu ảnh này bằng đường dẫn tương đối. Ngưỡng
`free_thresh: 0.196` giữ màu xám 205 là unknown; file bản đồ cũ có ngưỡng 0.25
sẽ coi màu này là free. Launch bài 7 dùng cùng một file map cho Nav2 và phân
vùng. Không sửa ảnh hoặc cấu hình của bài trước.

## 1. Build

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/tht4_tb4_nav src/tht7_tb4 --ignore-src -r -y
PYTHONNOUSERSITE=1 colcon build --packages-select tht4_tb4_nav tht7_tb4 --symlink-install
source install/setup.bash
```

Mỗi terminal ROS cần source `/opt/ros/humble/setup.bash` và
`~/tht4_tb4_ws/install/setup.bash` trước khi dùng các lệnh bên dưới.

## 2. Xem / tạo lại bản đồ phân vùng

Ảnh kết quả có sẵn trong `src/tht7_tb4/results/segmented_map.png`.
Để tạo lại sau khi đổi đích hoặc bản đồ:

```bash
ros2 run tht7_tb4 segment_map \
  --output ~/tht4_tb4_ws/src/tht7_tb4/results
```

Có thể truyền `--map /path/map.yaml --regions /path/regions.yaml`.
Đích phải nằm trong ô trống và cách vật cản ít nhất `minimum_clearance`
(mặc định 0.30 m); ba đích phải cùng một thành phần liên thông của bản đồ.
Phân vùng không gán nhãn cho thành phần trống không nối với hạt giống nào.
Khoảng cách kiểm tra này không thay thế kiểm tra footprint/costmap của Nav2.

Đổi tọa độ bằng cách chọn một Nav2 Goal hợp lệ trên RViz rồi ghi `(x,y,yaw)`
vào YAML. Khi đổi map, cập nhật resolution/origin/image trong `config/map.yaml`
và toàn bộ đích cho đúng bản đồ mới.

Chuyển pixel `(u,v)` sang hệ map, với ảnh cao `H`, độ phân giải `r`, origin
`(ox,oy,theta)`:

```text
lx = (u + 0.5) * r
ly = (H - v - 0.5) * r
x = ox + cos(theta)*lx - sin(theta)*ly
y = oy + sin(theta)*lx + cos(theta)*ly
```

## 3. Chạy mô phỏng, localization và Nav2

Terminal 1:

```bash
ros2 launch tht7_tb4 tht7.launch.py
```

Launch mặc định bật `start_sim:=true`, `use_nvidia:=true` và đọc
`config/regions.yaml` của package. Khi build bằng `--symlink-install`, file
cấu hình này liên kết tới file trong `src/tht7_tb4/config/regions.yaml`.
Vẫn cần source ROS/workspace và đặt `export ROS_DOMAIN_ID=30` trong mỗi terminal.
Trên laptop Intel + NVIDIA, Gazebo
(cả GUI và cảm biến GPU) và RViz kế thừa các biến PRIME render offload.
Qt dùng `xcb` qua XWayland nếu desktop đang chạy Wayland. Cần khởi động lại
phiên mô phỏng để đổi GPU; tiến trình đã chạy không chuyển GPU tại chỗ.
Kiểm tra bằng `nvidia-smi`: khi mô phỏng chạy, danh sách tiến trình phải có
Gazebo và RViz sử dụng VRAM. Trên máy không có driver NVIDIA, dùng:

```bash
ros2 launch tht7_tb4 tht7.launch.py use_nvidia:=false
```

Cấu hình theo [hướng dẫn PRIME render offload của NVIDIA](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/optimus-laptops-and-multi-gpu-desktop-systems.html).

Chờ Gazebo chạy, RViz nhận bản đồ. Dùng **2D Pose Estimate** đặt đúng vị trí
và hướng robot so với Gazebo. Terminal 2, cho robot rời dock:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

Chờ `SUCCEEDED`. Dừng teleop và node waypoint bài trước nếu đang chạy.
Terminal 2 theo dõi trạng thái:

```bash
ros2 topic echo /voice_navigation/status std_msgs/msg/String \
  --qos-durability transient_local
```

Các terminal phải dùng cùng `ROS_DOMAIN_ID` với terminal launch. Trên máy
hiện tại node chạy với `ROS_DOMAIN_ID=30`; dùng `export ROS_DOMAIN_ID=30`
trong terminal theo dõi nếu cần. `transient_local` giúp nhận cả trạng thái gần
nhất đã phát trước khi mở terminal theo dõi. Nếu CLI vẫn báo không thấy topic,
chạy `ros2 daemon stop` rồi thử lại để làm mới discovery.

Nếu đã có Gazebo + AMCL + Nav2, chỉ chạy:

```bash
ros2 launch tht7_tb4 tht7.launch.py start_sim:=false
```

Phiên Nav2 sẵn có phải dùng đúng map bài 7; nếu đang dùng cấu hình bản đồ cũ,
khởi động lại với `start_sim:=true` sau khi đóng phiên trước.
Trong RViz hiện có, thêm display **MarkerArray**, topic `/semantic_regions`,
Durability **Transient Local** để thấy nhãn. Hoặc mở cấu hình đi kèm:

```bash
rviz2 -d ~/tht4_tb4_ws/src/tht7_tb4/rviz/semantic.rviz \
  --ros-args -p use_sim_time:=true
```

`start_sim:=true` dùng mô phỏng và đồng hồ simulation. Khi dùng robot thật,
khởi động Nav2 của robot riêng rồi dùng `start_sim:=false use_sim_time:=false`
và truyền `map:=... regions:=...` phù hợp. Node không tự đặt initial pose.

## 4. Thử bằng văn bản trước

Terminal 3, chạy từng câu và chờ `SUCCEEDED` trước khi gửi câu tiếp:

```bash
ros2 run tht7_tb4 send_command 'Đi đến khu vực A.'
ros2 run tht7_tb4 send_command 'Đi đến phòng thí nghiệm.'
ros2 run tht7_tb4 send_command 'Đi đến hành lang.'
ros2 run tht7_tb4 send_command 'Di chuyển từ khu vực A đến khu vực B.'
```

Lệnh A → B luôn gửi A trước (kể cả robot đang ở vùng khác), chỉ gửi B sau khi
Nav2 xác nhận A thành công. Nếu A trùng B, chỉ gửi một đích.

```bash
ros2 run tht7_tb4 send_command 'Đi đến khu vực Z.'
ros2 run tht7_tb4 send_command 'Di chuyển từ Z đến A.'
ros2 run tht7_tb4 send_command 'Dừng lại.'
```

Hai câu có Z phải trả `INVALID`, không gửi bất kỳ đích nào. `Dừng lại` hoặc
`Hủy điều hướng` hủy goal của node này và xóa phần còn lại của tuyến. Node chờ
kết quả kết thúc của goal trước khi nhận tuyến mới. Lệnh mới khi robot đang
đi sẽ trả `BUSY`, không xếp hàng ngầm.

## 5. Cài và chạy nhận giọng nói tiếng Việt

Dùng Python 3.10 của ROS Humble và venv có quyền đọc các gói ROS hệ thống:

```bash
cd ~/tht4_tb4_ws
sudo apt install python3-venv ffmpeg libportaudio2
/usr/bin/python3 -m venv --system-site-packages .venv-tht7
source .venv-tht7/bin/activate
python -m pip install --upgrade pip 'setuptools<80' wheel packaging
python -m pip install -r src/tht7_tb4/requirements-voice.txt
```

Whisper và PyTorch cần dung lượng đĩa; lần đầu nạp model cần mạng. Dùng model
đa ngôn ngữ `small` cho tiếng Việt; có thể thử `base` nếu máy yếu. Sau khi model
đã tải, nhận dạng chạy cục bộ. Không dùng model có đuôi `.en`.

Terminal 3, vẫn chạy phiên Nav2 ở terminal 1:

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv-tht7/bin/activate
# Dùng python của venv để nhận đúng thư viện âm thanh và Whisper.
python -m tht7_tb4.speech_input \
  --model small --seconds 5
```

Nhấn Enter rồi nói một trong các câu mẫu trong 5 giây. Đợi dòng `Đã gửi` và
trạng thái điều hướng. Gõ `q` để thoát. Nói tên phòng thường dễ phân biệt hơn
chỉ một chữ cái; chương trình cũng hiểu cách đọc `bê`, `bi`, `xê`, `xi`.
Độ chính xác phụ thuộc micro, tiếng ồn và model; cần thử thực tế trước khi quay.

Chọn micro:

```bash
python -m tht7_tb4.speech_input --list-devices
python -m tht7_tb4.speech_input --device 2
```

Thử với một file đã thu (ffmpeg xử lý WAV/MP3/M4A…):

```bash
python -m tht7_tb4.speech_input \
  --model small --audio /path/di_den_phong_hoc.wav
```

Nếu cài Whisper vào đúng Python mà colcon sử dụng, có thể dùng trực tiếp
`ros2 run tht7_tb4 speech_input --model small`; lệnh Python phía trên tránh
trường hợp executable của colcon dùng Python hệ thống thay vì Python venv.

## 6. Trạng thái và xử lý lỗi

| Trạng thái | Ý nghĩa / cách xử lý |
|---|---|
| `NOT_READY` | Thiếu AMCL, scan, TF hoặc Nav2; sửa nguyên nhân rồi nói lại |
| `INVALID` | Không khớp câu lệnh hoặc khu vực không tồn tại; không gửi goal |
| `ROUTE`, `NAVIGATING` | Tuyến được nhận và đích đang gửi |
| `FEEDBACK`, `ARRIVED` | Khoảng cách còn lại / đích vừa hoàn thành |
| `SUCCEEDED` | Nav2 đã báo thành công cho tất cả đích trong tuyến |
| `REJECTED`, `FAILED` | Dừng tuyến ngay, không tự nhảy sang đích kế |
| `CANCELING`, `STOPPED` | Đang hủy / goal đã kết thúc sau yêu cầu dừng |
| `BUSY` | Chưa thể nhận tuyến mới |
| `ERROR` | Lỗi giao tiếp/hủy; đọc log và kiểm tra robot trước khi chạy lại |

Node hủy tuyến nếu scan mất quá 3 giây, dữ liệu scan không hợp lệ, TF/Nav2
không còn sẵn sàng, hoặc quá `goal_timeout` (180 giây mỗi đích). Có thể tăng:

```bash
ros2 launch tht7_tb4 tht7.launch.py goal_timeout:=300.0
```

Nếu chưa nhận được goal handle, node vẫn giữ trạng thái bận và sẽ hủy ngay
khi Nav2 phản hồi; không gửi tuyến mới trong tình trạng chưa xác định goal cũ.
Lệnh dừng giọng nói cần thu âm và nhận dạng nên có độ trễ; khi cần dừng ngay,
dùng chức năng dừng của robot/Nav2. Node này chỉ hủy goal do chính nó gửi.

## 7. Kiểm thử và kịch bản nộp bài

```bash
cd ~/tht4_tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/tht7_tb4 /usr/bin/python3 -m pytest src/tht7_tb4/test -q
```

Kiểm thử gồm câu có/không dấu, tuyến A → B, không nhận một phần câu chứa vùng
lạ, chuyển tọa độ có origin xoay, ô unknown/vật cản, dữ liệu phân vùng thực,
và action server Nav2 giả để kiểm tra thành công/thất bại/hủy. Action server
giả xác minh giao thức và thứ tự goal; không thay thế thử di chuyển trong Gazebo.

Video demo cần cho thấy:

1. Bản đồ màu có đủ A/B/C và nhãn trong RViz.
2. Nói ít nhất ba câu, hiển thị văn bản được nhận dạng và robot đến đúng vùng.
3. Đặt robot ở vùng C, nói “Di chuyển từ khu vực A đến khu vực B”, quan sát
   robot qua A rồi tới B; đối chiếu `ARRIVED` và `SUCCEEDED`.
4. Nói “Đi đến khu vực Z”, nhận thông báo không tồn tại và robot không chạy.
5. Quan sát Nav2 tránh vật cản; lưu log và video thực tế, không dùng kết quả
   action server giả làm bằng chứng robot đã điều hướng thành công.

Tài liệu API: [Nav2 NavigateToPose Humble](https://api.nav2.org/actions/humble/navigatetopose.html),
[Whisper: cài đặt và nhận dạng đa ngôn ngữ](https://github.com/openai/whisper).
