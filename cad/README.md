# CAD tham chiếu xe Ackermann 2WD

- `ackermann_reference.step`: cụm CAD bảy chi tiết, đơn vị mm; mở được trong FreeCAD, SolidWorks hoặc Fusion.
- `export_vehicle.py`: nguồn CadQuery chỉnh sửa được, dựng cùng hình học nguyên thủy của URDF.
- Kích thước dùng chung với mô phỏng tại `src/ackermann_description/config/vehicle.json` (đơn vị m, kg).

Gốc tọa độ ở tâm cầu sau chiếu xuống mặt đất; trục X hướng trước, Y hướng trái, Z hướng lên.
Hai bánh sau chủ động, bánh trước quay thụ động trên hai khớp lái. CAD hiển thị trạng thái lái bằng 0.

Đây là **CAD hình học tham chiếu do dự án tạo**, không phải CAD gốc của tác giả repo xe.
Chưa có servo, động cơ, ổ bi, cơ cấu truyền lái, lỗ lắp hoặc dung sai; cần thiết kế các phần này trước khi chế tạo.
Việc STEP có các solid giao nhau ở vị trí bánh và hub phản ánh mô hình mô phỏng, chưa mô tả mối ghép cơ khí.

Xuất lại trong môi trường riêng để không ảnh hưởng NumPy của ROS:

```bash
python3 -m venv /tmp/patrol-cad
/tmp/patrol-cad/bin/pip install -r cad/requirements.txt
/tmp/patrol-cad/bin/python cad/export_vehicle.py
```

Nguồn hình học và kích thước được thích nghi từ repo Ackermann MIT; xem
`src/ackermann_description/UPSTREAM.md` và `LICENSE.upstream`.
