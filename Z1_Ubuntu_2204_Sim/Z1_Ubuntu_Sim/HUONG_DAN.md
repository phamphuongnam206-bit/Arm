# Mô phỏng Z1 V3 trên Ubuntu 22.04

Gói dùng PyBullet và mô hình URDF, với lưới hình học trích xuất từ
Z1_Servo_Mounts_V3.step mà bạn cung cấp. Có chế độ chạy tự động và
thanh trượt điều khiển từng khớp. Mọi dữ liệu vật lý trong chương trình dùng mét,
kilôgam, giây, radian và N·m.

## Chạy lần đầu

Giải nén ZIP. Mở Terminal trong thư mục Z1_Ubuntu_Sim và chạy

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
bash run.sh
```

Lần đầu cần Internet để tải PyBullet 3.2.7. Chương trình tạo môi trường Python
.venv ngay trong thư mục. Máy cần phiên làm việc đồ họa để mở cửa sổ.

Robot tự tiếp cận, đóng ngàm, nhấc vật lên 40 mm rồi giữ 3 giây.
Sau đó cửa sổ chuyển sang chế độ thanh trượt. Thanh Closure điều chỉnh
mức đóng ngàm, 0 mm là mở ở tư thế CAD tham chiếu, 8 mm là đóng tối đa
trong mô hình gần đúng này. Nhấn R để đặt lại vật, Q để thoát.
Kéo thanh Closure về 0 để thả vật.

Chạy ngay chế độ thủ công

```bash
bash run.sh --manual
```

Chạy thử vật lý không mở cửa sổ

```bash
bash run.sh --headless
```

Kết quả của mỗi lần chạy tự động được in trong Terminal và ghi vào
last_result.json bên cạnh chương trình. Nếu không có màn hình hoặc đang
dùng SSH, dùng --headless. Nếu cửa sổ đồ họa lỗi trên máy ảo, có thể thử

```bash
LIBGL_ALWAYS_SOFTWARE=1 bash run.sh
```

## Bài thử đã thiết lập

| Đại lượng | Giá trị |
| --- | --- |
| Khoảng cách ngang từ trục đế đến tâm vật | 200 mm |
| Khối lượng vật | 50 g |
| Kích thước khối thử | 15 × 15 × 15 mm |
| Độ cao mặt đặt vật tính từ sàn | 60 mm |
| Độ cao nâng mục tiêu | 40 mm |
| Thời gian giữ sau khi nâng | 3 giây |
| Ma sát tiếp xúc ngang | 0,8 |
| Bước thời gian | 1/240 giây |

Kích thước vật, độ cao mặt đặt và thời gian giữ là các lựa chọn cho bài thử,
vì đề bài mới cung cấp tầm với và khối lượng.

Tiêu chí lift_held_3s yêu cầu tiếp xúc cả hai má trước khi nâng và ở mọi
mẫu kiểm tra trong thời gian giữ, tâm vật cao hơn vị trí ban đầu ít nhất 15 mm,
đồng thời lệch ngang dưới 15 mm. Mẫu được kiểm tra mỗi 0,1 giây.
Chương trình dùng trọng lực và tiếp xúc ma sát để giữ vật.

## Giới hạn của bản mô phỏng

- Hình dáng các phần được lấy từ STEP, với tâm khớp suy ra từ trục servo.
- Bốn khớp quay điều khiển đế, vai, khuỷu và cổ tay.
- Hai má kẹp dùng khớp trượt đối xứng và miếng tiếp xúc phẳng gần đúng.
  Các bánh răng và thanh liên kết được gộp vào cổ tay, vì cơ cấu kín
  của ngàm chưa được dựng đầy đủ.
- Khối lượng mỗi khâu, mô-men quán tính, giới hạn góc và mô-men động cơ
  là giá trị giả định trong z1.urdf. Cần cập nhật theo bản in, servo,
  điện áp cấp và số đo thực tế để đánh giá tải sát hơn.
- Các mặt va chạm là hình hộp đơn giản. Va chạm giữa các khâu của robot
  đang tắt. Chưa xác minh khoảng hở trên toàn hành trình.
- Hệ số ma sát là giả định, cần đối chiếu vật liệu má kẹp và vật thử.
- Đế được cố định vào thế giới mô phỏng.
- Kết quả đạt trong mô hình hiện tại áp dụng cho các giả định kể trên.
  Khả năng nâng của robot thực cần kiểm tra với mô hình và linh kiện đã hiệu chuẩn.

## Các tệp chính

| Tệp | Nội dung |
| --- | --- |
| simulate.py | Chương trình chạy và kiểm tra gắp vật |
| run.sh | Tạo môi trường, cài phụ thuộc và khởi động |
| z1.urdf | Khâu, khớp, khối lượng, giới hạn và hình học va chạm |
| meshes/ | Bảy lưới STL từ bản STEP V3 |
| cad_dimensions.json | Tâm khớp tham chiếu và kích thước bao lưới |
| verified_result.json | Kết quả kiểm tra đã chạy khi tạo gói |

Bản đã được kiểm tra bằng PyBullet ở chế độ không cửa sổ, gồm tải URDF,
chạy chu trình gắp, kiểm tra tiếp xúc và dựng ảnh bằng bộ dựng hình phần mềm.
Giao diện đồ họa cần được chạy lại trên máy Ubuntu của bạn.

Tài liệu PyBullet chính thức
https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html
