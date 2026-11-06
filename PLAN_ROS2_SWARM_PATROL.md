# Mô phỏng bầy xe Ackermann tuần tra bằng Kuramoto–Lissajous và MPCC

Ngày lập kế hoạch: 2026-11-06.

Tài liệu này ghi lại kế hoạch đã thống nhất trước khi triển khai.

## 1. Lựa chọn nền tảng

**Chọn xe 4 bánh, hai bánh sau dẫn động, hai bánh trước lái Ackermann**, chạy trên ROS 2 Humble + Gazebo Fortress 6. Cấu hình chính là **3 xe có vật lý trong Gazebo và 2 tác tử ảo**, theo lựa chọn của người dùng.

Với bài báo này, 2WD Ackermann phù hợp hơn: tương thích mô hình bicycle của MPCC, ít thành phần điều khiển và thuận tiện phát triển phần cứng. 4WD Ackermann đáng cân nhắc khi cần lực kéo trên địa hình khó; còn xe 4WD quay bằng chênh lệch tốc độ hai bên cần mô hình điều khiển khác.

Dùng [Balakarthik15/Ros2_ackermann_robot](https://github.com/Balakarthik15/Ros2_ackermann_robot) làm nguồn URDF/Xacro. Repo hiện hướng tới Jazzy/Harmonic, nên chỉ tiếp nhận mô hình và tài nguyên cần thiết; xây lại phần tích hợp Humble/Fortress dựa trên [gz_ros2_control chính thức](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html).

**Chưa xác nhận repo này có bộ CAD cơ khí chỉnh sửa được.** Theo lựa chọn ưu tiên mô phỏng, CAD không chặn giai đoạn đầu; không coi mesh hiển thị là bản CAD chế tạo.

Bài báo tham chiếu: Boldrer et al., *Time-Inverted Kuramoto Model Meets Lissajous Curves: Multi-Robot Persistent Monitoring and Target Detection*, IEEE Robotics and Automation Letters, 8(1), 240–247, 2023. [DOI: 10.1109/LRA.2022.3224661](https://doi.org/10.1109/LRA.2022.3224661).

## 2. Kiến trúc và giao diện

Giữ bản MATLAB hiện có trong `matlab/` làm nguồn đối chiếu. Thêm các ROS package trong `src/`, không ghi đè các thay đổi sắp xếp file đang có.

| Thành phần | Trách nhiệm |
|---|---|
| `ackermann_description` | URDF/Xacro, hình học, quán tính và thông số xe |
| `ackermann_gazebo` | World trống, ros2_control, bridge và spawn |
| `swarm_interfaces` | Message trạng thái pha, tham chiếu và chẩn đoán |
| `swarm_coordination` | Kuramoto phân tán và tác tử ảo |
| `lissajous_path` | Đường tham số, độ dài cung và ước lượng pha |
| `ackermann_mpcc` | MPCC và chuyển lệnh ga thành vận tốc |
| `multi_robot_bringup` | Launch một xe, cấu hình 3+2 và cấu hình 5 xe |
| `patrol_evaluation` | Kịch bản, rosbag, chỉ số và biểu đồ |

Luồng điều khiển:

**Trạng thái xe → ước lượng pha → Kuramoto → tốc độ pha mong muốn → MPCC → ga/góc lái → lớp chấp hành → ros2_control → Gazebo.**

Các giao diện chính:

- Mỗi tác tử công bố `PhaseState`: thời gian, ID, pha liên tục, tốc độ pha và cờ hợp lệ.
- Mỗi node Kuramoto chỉ nhận pha của hai hàng xóm trên vòng truyền thông.
- MPCC nhận trạng thái tại tâm cầu sau và tốc độ pha mong muốn; xuất góc lái, ga và trạng thái bộ giải.
- Lớp chấp hành xuất `TwistStamped` tới controller, với `linear.x = v_cmd`, `angular.z = v_cmd·tan(δ)/L`. Đây là giao diện vận tốc của [steering controller Humble](https://control.ros.org/humble/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html).
- Mỗi xe có namespace, model Gazebo, controller manager và frame riêng; dùng chung `/clock` và một hệ tọa độ `world`.

Dùng Python, NumPy/SciPy cho thuật toán và đánh giá; CasADi/IPOPT cho MPCC. Khóa phiên bản phụ thuộc sau khi xác nhận chạy trên máy hiện tại.

## 3. Các mốc triển khai

### Mốc 1 — Một xe Ackermann chạy đúng

- Ghi lại commit và giấy phép của repo nguồn.
- Giữ kích thước xe nền; sửa quán tính, collision và các khớp cần thiết.
- Loại bỏ plugin Ackermann cũ; chỉ dùng `gz_ros2_control`, `joint_state_broadcaster` và `ackermann_steering_controller`.
- Hai bánh sau nhận lệnh vận tốc; hai khớp lái trước nhận lệnh vị trí; bánh trước quay thụ động.
- Dùng cấu hình Humble đã cài trên máy, bao gồm thứ tự joint phải–trái theo ví dụ chính thức; không sao chép nguyên YAML trong gợi ý.
- Kiểm tra chạy thẳng, rẽ hai phía, dừng và bán kính quay.

**Đầu ra:** một lệnh launch mở world và xe điều khiển được, có odometry và TF hợp lệ.

### Mốc 2 — Kuramoto–Lissajous và đội hình 3+2

- Chuyển các hàm toán học MATLAB thành thư viện Python có thể chạy độc lập ROS.
- Dùng vòng `1–2–5–3–4–1`, `N=5`, `p=2`; xe Gazebo là 2, 3, 4, tác tử ảo là 1, 5.
- Khởi tạo xe trên đường, hướng theo tiếp tuyến và có pha cân bằng tương ứng.
- Ước lượng pha xe từ vị trí cầu sau bằng phép chiếu cục bộ quanh tiến độ trước đó. Kết hợp vận tốc dự đoán để tránh nhảy nhánh tại giao điểm Lissajous.
- Tác tử ảo tích phân động học pha; hiển thị bằng marker, không tạo vật thể va chạm.
- Ban đầu dùng pose chuẩn Gazebo làm nguồn định vị, tương tự vai trò OptiTrack trong bài. Ghi riêng wheel odometry để đánh giá sai lệch.

**Đầu ra:** pha Kuramoto phản hồi theo chuyển động thực tế của ba xe, có thể quan sát đủ năm tác tử trong RViz.

### Mốc 3 — MPCC và mô hình ga

- Dùng trạng thái `[x, y, yaw, v, s]`, đầu vào `[δ, τ]`, mô hình bicycle và `ṡ=v`.
- Giữ cấu trúc chi phí của bài: sai số tốc độ pha, sai số dọc/ngang đường, góc lái và ga; horizon `M=15`.
- Mỗi chu kỳ giữ tốc độ pha mong muốn cố định trên horizon; dùng warm start.
- Chạy MPCC ở 10 Hz, lớp chấp hành và controller manager ở 100 Hz theo thời gian mô phỏng.
- Với mô hình số đối chiếu, dùng hệ số ga và giới hạn trong bài. Với xe Gazebo, dùng chiều dài cơ sở thực của URDF và kiểm chứng đáp ứng ga–vận tốc bằng thí nghiệm bước.
- Lớp chấp hành tích phân mô hình ga để tạo `v_cmd`; công bố cả vận tốc yêu cầu và vận tốc đo. Ghi rõ đây là mô hình chấp hành tương đương, chưa phải mô phỏng động cơ bằng mô-men.
- Khi bộ giải lỗi hoặc hết hạn 100 ms, gửi lệnh dừng, ghi sự kiện và giữ nghiệm hợp lệ gần nhất làm khởi tạo cho lần sau.

**Đầu ra:** ba xe chạy với MPCC, có log trạng thái bộ giải, thời gian giải và sai số bám đường.

### Mốc 4 — Thực nghiệm và đánh giá

Tạo hai nhóm cấu hình độc lập:

- **Đối chiếu bài báo:** mô hình số dùng `A=1,8 m`, `B=0,8 m`, `a=3`, `b=2`, `ω=0,3 rad/s`.
- **Xe Gazebo kích thước thật:** mặc định phóng đại đường bốn lần, `A=7,2 m`, `B=3,2 m`, `ω=0,075 rad/s`; tính lại các điều kiện hình học từ footprint thực. Báo cáo giới hạn độ cong và sai số, không coi đây là thí nghiệm giống hệt bài.

Chạy các kịch bản:

1. Tuần tra bình thường.
2. Dừng tác tử ảo 1 và 5, rồi phục hồi.
3. So sánh Kuramoto có phản hồi với `θ̇=ω`.
4. Nhiễu pha và phục hồi đội hình.
5. Phủ vùng và phát hiện mục tiêu bằng cảm biến đĩa lý tưởng.
6. Mở rộng thành năm xe Gazebo sau khi cấu hình 3+2 ổn định.

Giữ khoảng dừng `[9,6; 19,5] s` cho mô hình đối chiếu; cấu hình Gazebo phóng đại dùng `[38,4; 78] s`, tổng thời gian 180 s. Các mốc này là lựa chọn triển khai được ghi trong cấu hình.

## 4. Kiểm thử và tiêu chí nghiệm thu

- **Mô hình xe:** controller hoạt động, TF không xung đột; sai lệch bán kính quay và yaw rate ổn định dưới 10% trong các ca kiểm tra tốc độ thấp.
- **Toán học:** kiểm tra dấu Kuramoto, pha không wrap, khởi tạo theo vòng và ánh xạ pha–độ dài cung; sai lệch pha Python–MATLAB dưới `10⁻³ rad` khi dùng cùng tích phân và cấu hình.
- **Ước lượng pha:** đi qua giao điểm và qua nhiều vòng mà không đổi nhánh hoặc mất số vòng.
- **MPCC:** kiểm tra giới hạn đầu vào, warm start, timeout và lệnh dừng; báo cáo tỷ lệ giải thành công cùng thời gian giải p50/p95/max.
- **Tích hợp:** pause/reset mô phỏng không gây bước tích phân sai; thiếu trạng thái quá hai chu kỳ khiến node liên quan báo lỗi và xe nhận lệnh dừng.
- **Đánh giá:** xuất sai số bám đường, sai số đội hình sau loại bỏ pha chung, khoảng cách mọi cặp, tiếp xúc vật lý, độ phủ, thời gian thăm lại và thời gian phát hiện mục tiêu.
- Lưu rosbag, CSV, cấu hình, seed và phiên bản phần mềm để chạy lại; dùng seed 42 cho đối chiếu, seed 0–9 cho thống kê phát hiện.

Nghiệm thu yêu cầu tái hiện cơ chế phối hợp và đo được ảnh hưởng của sai số bám đường. Không đặt điều kiện phải khớp từng hình hay hệ số thực nghiệm trong bài.

## 5. Giả định và giới hạn

- Giai đoạn đầu dùng mặt phẳng không vật cản, định vị chuẩn và cảm biến phát hiện lý tưởng; chưa tích hợp SLAM, Nav2 hoặc nhận dạng camera.
- Bản tái lập giữ MPCC không có ràng buộc tránh va chạm như thí nghiệm của bài; mọi va chạm được ghi nhận. Hai tác tử ảo chỉ có kiểm tra khoảng cách hình học.
- Chưa thêm cơ chế tránh va chạm mới vì sẽ thay đổi thuật toán đang đánh giá.
- Sau khi mô phỏng đạt nghiệm thu, lập bộ CAD chỉnh sửa được và xuất STEP theo thông số URDF đã chốt; bộ CAD này là thiết kế tham chiếu, chưa phải hồ sơ chế tạo.
- Bàn giao mã nguồn, launch, cấu hình, hướng dẫn chạy và báo cáo kết quả; phân biệt rõ tham số lấy từ bài, từ repo xe và do triển khai lựa chọn.
