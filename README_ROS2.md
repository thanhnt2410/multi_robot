# ROS 2 Humble + Gazebo Fortress: bầy xe Ackermann tuần tra

Triển khai Kuramoto đảo thời gian, đường Lissajous và MPCC cho **3 xe có vật lý + 2 tác tử ảo**.
Cũng hỗ trợ một xe để kiểm tra cơ sở và năm xe có vật lý.
Mỗi xe có bốn bánh: hai bánh sau chủ động, hai bánh trước quay thụ động và lái Ackermann.

## Môi trường và cài đặt

Đã kiểm tra trên Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 6.17.1,
`gz_ros2_control` 0.7.18 và `ros2_controllers` 2.52.1.
Không dùng Gazebo Classic hoặc plugin truyền động Ackermann gốc cùng ros2_control.

Các gói ROS cần thiết (máy hiện tại đã có các gói chính):

```bash
sudo apt update
sudo apt install python3-venv python3-colcon-common-extensions python3-numpy \
  python3-scipy python3-matplotlib python3-yaml \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-ackermann-steering-controller ros-humble-joint-state-broadcaster \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-tf2-ros \
  ros-humble-rviz2 ros-humble-rosbag2
```

Từ thư mục gốc repo:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements-mpcc.txt
bash scripts/build_ros2.sh
bash scripts/check_ros2.sh
```

Script build dùng setuptools hệ thống của Humble, tránh lỗi `--editable not recognized`
khi máy có setuptools mới trong user site. Script chạy dùng môi trường Python đã ghim
NumPy/SciPy/CasADi; không cần thay phiên bản Python toàn máy.

## Chạy mô phỏng

### Demo có GUI, ba xe và hai tác tử ảo

Khuyến nghị dùng đường **feasible** để kiểm tra xe bám được trong giới hạn lái:

```bash
bash scripts/run_patrol.sh gui:=true rviz:=true \
  scenario:="$PWD/src/multi_robot_bringup/config/feasible.yaml" \
  output_dir:="$PWD/patrol_runs/demo_$(date +%Y%m%d_%H%M%S)"
```

Gazebo hiển thị ba xe vật lý. RViz hiển thị đường tham chiếu, điểm vị trí xe màu xanh
và tác tử ảo màu đỏ. Tác tử ảo không có thân va chạm trong Gazebo.
Các node chờ đủ trạng thái và tất cả controller active, sau đó bắt đầu nhiệm vụ sau 2 giây mô phỏng.
Khi đủ thời lượng, các tác tử nhận lệnh dừng; Ctrl+C để đóng launch tương tác.

### Chạy thí nghiệm tự đóng và ghi rosbag

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
.venv/bin/python scripts/run_experiment.py \
  --scenario src/multi_robot_bringup/config/feasible.yaml \
  --output patrol_runs/feasible_run_01 --record
```

Thư mục đầu ra phải mới. Script giới hạn thời gian chờ theo thời gian thực,
đọc `summary.json` rồi đóng đúng nhóm tiến trình đã khởi động.
Mặc định dùng ROS domain 85 và Gazebo partition riêng cho mỗi lần chạy;
có thể đặt `ROS_DOMAIN_ID` và `IGN_PARTITION` trước lệnh để tự chọn.

### Kiểm tra một xe và mở rộng năm xe

```bash
# Terminal 1: xe đứng yên, chờ lệnh kiểm tra
bash scripts/run_patrol.sh fleet:=single control:=false gui:=true

# Terminal 2: chạy thẳng, rẽ trái/phải, dừng; ghi số đo và kết quả pass/fail
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 .venv/bin/python scripts/validate_vehicle.py

# Năm xe có vật lý, cùng thuật toán
bash scripts/run_patrol.sh fleet:=five gui:=true \
  scenario:="$PWD/src/multi_robot_bringup/config/feasible.yaml" \
  output_dir:="$PWD/patrol_runs/five_$(date +%Y%m%d_%H%M%S)"
```

Giữ cùng ROS domain giữa hai terminal kiểm tra xe. `fleet:=single control:=true`
là chế độ tích hợp một xe và bốn tác tử ảo, không phải bài kiểm tra xe tại gốc tọa độ;
chế độ một xe tại gốc chỉ dùng `control:=false`.

## Hai cấu hình Gazebo và bản đối chiếu

| Cấu hình | Đường Lissajous | Tốc độ pha | Mục đích |
|---|---|---|---|
| `scaled.yaml` — cấu hình của plan gốc | A=7,2 m; B=3,2 m | 0,075 rad/s | Đánh giá sai lệch khi đường còn vượt khả năng lái |
| `feasible.yaml` — mặc định chạy | A=43,2 m; B=19,2 m | 0,0125 rad/s | Biến thể kỹ thuật có độ cong phù hợp xe |
| `paper.yaml` | A=1,8 m; B=0,8 m | 0,3 rad/s | Tham chiếu mô hình số; không tự đổi xe Gazebo thành xe bài báo |

Cả hai cấu hình Gazebo dùng a=3, b=2, N=5, p=2, vòng truyền thông `1–2–5–3–4–1`.
Xe vật lý là 2, 3, 4; tác tử ảo là 1, 5. Trong thí nghiệm 180 s, tác tử 1 và 5
dừng ở [38,4; 78] s rồi hoạt động lại. Với `fleet:=five`, hai xe vật lý 1 và 5 dừng.

Đường scaled cần góc lái bicycle lớn nhất khoảng **62°**, trong khi MPCC giới hạn **20°**.
Đường feasible cần khoảng **17,4°**. Tăng kích thước vùng làm thay đổi bài toán thực nghiệm,
không phải tái lập chính xác điều kiện của tác giả.

Các node nhận cùng tham số đường từ file scenario. Các tham số xe/MPCC nằm trong `config/mpcc.yaml`;
tham số đường trong scenario được ưu tiên. Cấu hình xe cơ khí dùng chung URDF và CAD nằm tại
`src/ackermann_description/config/vehicle.json`. Nếu đổi thông số cơ khí, phải cập nhật tương ứng
chiều dài cơ sở/bán kính/track trong `ackermann_gazebo/config/controllers.yaml` và `config/mpcc.yaml`.

Muốn chạy vòng hở: sao chép scenario thành file mới và đặt `feedback: false`.
Muốn bỏ sự cố: đặt `failure_start` và `failure_end` lớn hơn `duration`.
Muốn nhiễu pha: đặt `perturb_time >= 0`, `perturb_amount` và `perturb_agent` (mặc định 1).
Nhiễu pha trực tiếp hiện áp dụng cho tác tử ảo; không dịch chuyển tức thời xe vật lý.

## Mô hình số và đối chiếu MATLAB

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

# Cùng hình học/vòng truyền thông của thí nghiệm bài báo
.venv/bin/python -m patrol_evaluation.offline --paper --duration 45 \
  --output patrol_runs/ideal_feedback
.venv/bin/python -m patrol_evaluation.offline --paper --feedback-off --duration 45 \
  --output patrol_runs/ideal_open_loop

# Ba xe bicycle lý tưởng + hai tác tử ảo; không chạy Gazebo
.venv/bin/python -m patrol_evaluation.offline --mode mpcc --paper --duration 30 \
  --output patrol_runs/paper_mpcc

# Bài kiểm tra phát hiện: 15 tác tử lý tưởng, 10.000 mục tiêu, seed 0..9
.venv/bin/python scripts/detection_sweep.py --output patrol_runs/detection_sweep

# Kiểm tra dữ liệu stale, solver lỗi, hold và đồng hồ quay lùi; cần DDS cục bộ
ROS_DOMAIN_ID=91 .venv/bin/python scripts/validate_ros_safety.py -v
```

Chạy cả bộ kịch bản **bằng vật lý Gazebo**, mỗi lượt 180 s, theo thứ tự để tránh
các bộ giải tranh chấp CPU:

```bash
.venv/bin/python scripts/run_gazebo_suite.py --output patrol_runs/scenario_suite_01
.venv/bin/python scripts/validate_gazebo_faults.py --output patrol_runs/gazebo_faults_01
.venv/bin/python scripts/report_gazebo_suite.py patrol_runs/scenario_suite_01 \
  --faults patrol_runs/gazebo_faults_01
```

Bộ này gồm tuần tra bình thường, dừng/phục hồi, vòng hở đối chứng, nhiễu pha tác tử ảo
và năm xe vật lý. Kiểm thử lỗi dùng các world riêng: pause/resume/reset thông qua dịch vụ
Gazebo và tạm dừng đúng node phối hợp của lượt kiểm thử để đo khả năng dừng khi mất dữ liệu.
Các lượt kiểm tra này chạy headless; kết quả và biểu đồ được xuất vào
`validation/GAZEBO_SCENARIOS.md`. Dự kiến cần khoảng 15–20 phút cho bộ đầy đủ trên máy hiện tại.
Mã thoát 1 của bộ chạy báo có tiêu chí không đạt, kể cả ca vòng hở đối chứng;
các số đo, kiểm tra và kết quả từng ca vẫn được lưu đầy đủ.

`--paper` dùng l=0,175 m, c=0,94, alpha=36,8, beta=0,94, ga [0,1; 0,25],
góc lái ±π/9 và M=15. Chu kỳ 0,1 s, nội suy độ dài cung và quy ước cộng M stage
là lựa chọn triển khai. Bản Python dùng spline trơn cho nghịch đảo độ dài cung để giảm
lỗi hội tụ ở giao điểm của các đoạn nội suy tuyến tính; không tái tạo từng chi tiết bộ giải của tác giả.

Mô hình MPCC số cho phép bộ giải tối đa 1 s để đánh giá thuật toán;
node ROS dùng ngưỡng 100 ms. Các thời gian giải đều là số đo trên máy hiện tại.

Khi có MATLAB, chạy `export_ros_reference('/tmp/patrol_matlab_reference.csv')`
từ thư mục `matlab/`, rồi đối chiếu bằng:

```bash
PYTHONNOUSERSITE=1 .venv/bin/python scripts/compare_matlab.py \
  /tmp/patrol_matlab_reference.csv patrol_runs/ideal_feedback/samples.csv
```

Script trả lỗi nếu sai lệch pha lớn nhất không nhỏ hơn 10⁻³ rad.

## Giao diện và cơ chế dừng

- `/robot_i/ground_truth`: `nav_msgs/Odometry`, pose trong `world`, twist theo thân xe,
  gốc xe tại tâm cầu sau chiếu xuống mặt đất.
- `/robot_i/phase`: `swarm_interfaces/PhaseState`, pha liên tục phản ánh vị trí xe đo được;
  tác tử ảo tích phân động học pha.
- `/robot_i/phase_reference`: `PhaseReference`, tốc độ pha từ hai hàng xóm.
- `/robot_i/control_status`: `ControlStatus`, trạng thái solver, thời gian giải, lệnh và sai số.
- `/robot_i/ackermann_steering_controller/reference`: `geometry_msgs/TwistStamped`.
- `/robot_i/ackermann_steering_controller/odometry`: wheel odometry, độc lập với pose chuẩn.
- `/patrol/start`, `/patrol/finished`: mốc bắt đầu và tín hiệu kết thúc, QoS transient local.

MPCC dự đoán `[x,y,yaw,v,s]` với đầu vào góc lái và ga. Lớp chấp hành 100 Hz
chuyển mô hình ga thành lệnh vận tốc; đây chưa phải điều khiển mô-men động cơ.
Cho phép ga về 0 để xe Gazebo giảm tốc/dừng; khác giới hạn ga tối thiểu của bản paper.
Khi tốc độ pha mong muốn không dương, xe dừng thay vì chạy lùi.

Dữ liệu quá 0,2 s, solver lỗi/hết hạn hoặc đồng hồ quay lùi khiến xe nhận lệnh 0.
Timer chấp hành chạy riêng với timer solver. Đây là phần mềm mô phỏng, không có bảo đảm thời gian thực cứng.
IPOPT dùng giới hạn wall time 90 ms, kèm kiểm tra thời gian giải 100 ms ở ngoài.
Sau hold hoặc lỗi giải, MPCC khởi tạo lại từ độ cong đường và một ramp tăng tốc;
không tiếp tục dùng dự đoán cũ của xe đang chạy khi xe đã đứng yên.
Pause không tích phân theo thời gian thực; reset thời gian kết thúc lượt đánh giá với lý do `clock_reset`.
Khởi động lại launch để có lượt thí nghiệm mới sau reset.

## Kết quả và giới hạn

Mỗi lượt Gazebo xuất:

- `metadata.json`: cấu hình, seed, bán kính cảm biến, các giới hạn lý thuyết và phiên bản.
- `samples.csv`: vị trí, pha, sai số đường, khoảng cách mọi cặp và chỉ số sensing.
- `control.csv`: trạng thái từng lần cập nhật điều khiển.
- `sensing.npz`: thời gian phát hiện mục tiêu và lưới đã quan sát cho từng tác tử.
- `summary.json`, `launch.log`, và `bag/` khi bật ghi rosbag.

Xuất biểu đồ từ một lượt đã chạy:

```bash
PYTHONNOUSERSITE=1 .venv/bin/python scripts/plot_run.py patrol_runs/feasible_run_01
```

`robot_contact_samples` đếm mẫu tiếp xúc giữa các robot từ cảm biến thân, bánh và khớp lái;
bỏ qua tiếp xúc nền và cùng robot. Các lượt kiểm tra đầu chỉ có cảm biến thân và dùng trường `contact_messages`.
Các tác tử ảo chỉ được đánh giá khoảng cách hình học. `revisit_max` đo khoảng vắng quan sát
giữa hai lần nhìn thấy trên lưới đã được thăm, không chứng minh giới hạn tái thăm liên tục.

Cảm biến phát hiện là đĩa lý tưởng với bán kính từ Eq. (7), (8), có biên 1%.
Phủ 100% và phát hiện đủ mục tiêu trong mô phỏng này không chứng minh cảm biến thực đạt tương tự.
Chưa có LiDAR/SLAM/Nav2, vật cản, mất liên lạc kéo dài hay ràng buộc tránh va chạm trong MPCC.
Không dùng kết quả này để tuyên bố bảo đảm an toàn ngoài các giả thiết của bài báo.

Xem [báo cáo kiểm chứng](validation/REPORT.md), [kế hoạch](PLAN_ROS2_SWARM_PATROL.md)
và [CAD tham chiếu](cad/README.md).

## Nguồn mô hình

URDF thích nghi từ [Balakarthik15/Ros2_ackermann_robot](https://github.com/Balakarthik15/Ros2_ackermann_robot),
commit `b298e0551aaddef6f78868f9921e5e42cca91edb`, MIT.
Giữ giấy phép và mô tả thay đổi tại `src/ackermann_description/`.

Fortress có giới hạn context ROS dùng chung giữa các plugin:
[gz_ros2_control issue #145](https://github.com/ros-controls/gz_ros2_control/issues/145).
Vì vậy tất cả xe dùng cùng `controllers.yaml` chứa đủ namespace, thay vì mỗi xe một file.

## Quan sát bằng RViz

```bash
bash scripts/run_patrol.sh rviz:=true
```

Panel **Displays** có Grid, Patrol và RobotModel cho từng xe vật lý. Góc nhìn mặc định
bao quát đường tuần tra; chấm đỏ là tác tử ảo, vòng xanh đánh dấu xe vật lý.
Xe dài khoảng 0,46 m trên vùng rộng 86,4 m, nên cần zoom để thấy rõ thân và bánh.
Trong panel **Views**, chọn góc nhìn lưu sẵn `Robot 2 close view` (hoặc xe 3, 4)
để theo dõi cận cảnh xe. Tắt checkbox **Patrol** khi xem cận cảnh để nhãn lớn
không che mô hình. Có thể dùng con lăn để zoom.
Marker hiển thị cả khi chờ khởi động và sau khi kết thúc nhiệm vụ.

Mỗi lần chạy mặc định tạo thư mục `patrol_runs/interactive_<thời gian>_<mã>/`,
bao gồm cấu hình RViz đã khớp với fleet và scenario. Nếu chỉ định `output_dir:=...`,
hãy chọn thư mục chưa có kết quả; hệ thống không ghi đè lượt chạy cũ.
Minh chứng kiểm thử giao diện: [validation/RVIZ.md](validation/RVIZ.md).
