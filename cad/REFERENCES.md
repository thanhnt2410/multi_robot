# Nguồn tham khảo mô hình cơ khí

Kiểm tra trực tiếp các repository ngày 2026-11-06. Các revision bên dưới đã tải về
để đọc; không nhập mesh, CAD hoặc code từ các nguồn mới này vào dự án.
CAD chi tiết trong `mechanical_vehicle.py` được dựng riêng theo kích thước hiện tại.

| Repo / revision | File đã đối chiếu | Kết quả và phần tham khảo |
| --- | --- | --- |
| [Linorobot ackermann_vehicle](https://github.com/linorobot/ackermann_vehicle/tree/fa3c13e199da702f05542e9e05023200d53e0b2a), `fa3c13e` | `ackermann_vehicle_description/urdf/em_3905.urdf.xacro`, `LICENSE.md` | Mô hình RC Traxxas E-Maxx xấp xỉ, có cấu hình hub, bánh, pin, servo và thông số giảm xóc; Apache-2.0. Tham khảo cách chia cụm khung/pin/hub và phân biệt khớp lái với khớp lăn. Không đưa thông số xe lớn hơn hoặc hệ treo của repo vào xe hiện tại. |
| [MIT racecar_simulator](https://github.com/mit-racecar/racecar_simulator/tree/326b258ada9e54cfa142a2f1a0b27c540a408367), `326b258` | `racecar.xacro` | Tác giả mô tả đây là mô hình RViz đơn giản, dùng box/cylinder. Hữu ích để đối chiếu gốc xe và wheelbase; không cung cấp bộ CAD cơ khí đầy đủ để nhập nguyên cụm. |
| [MIT racecar](https://github.com/mit-racecar/racecar/tree/b464bf1f85341037087c7dcc0ff75c0671c26d97), `b464bf1` | `racecar/package.xml`, `racecar/LICENSE`, `racecar/config/racecar-v2/vesc.yaml` | Stack ROS 1 và cấu hình phần cứng; package racecar khai báo BSD. Không có assembly CAD trong cây đã kiểm tra. |
| [Mugilan Ackerman_steering_robot](https://github.com/Mugilan-Theroboticist/Ackerman_steering_robot/tree/a08e3d7ecd273151e99b0b3f42a713af03c7492a), `a08e3d7` | `model/vehicle.urdf.xacro`, `LICENSE` | Mô hình ROS 2 dạng tham số với các link/khớp lái riêng. Hình học cơ bản, không phải bộ CAD gia công. LICENSE tự ghi MIT nhưng văn bản rút gọn; không tái sử dụng tài sản từ repo này. |

Ngoài ra, bố trí sàn thiết bị và trụ đỡ tham khảo mô tả lắp xe của
[F1TENTH build manual](https://f1tenth.github.io/build-old.html).
Không sao chép bản vẽ, mesh hoặc thiết kế độc quyền của xe RC thương mại.

URDF nền vốn đã thích nghi từ Balakarthik15/Ros2_ackermann_robot; nguồn và MIT license
đã giữ ở `src/ackermann_description/UPSTREAM.md` và `LICENSE.upstream`.
