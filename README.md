#  Hệ thống băng chuyền phân loại sản phẩm theo màu sắc sử dụng STM32

## Mô tả đề tài
Đề tài này triển khai một hệ thống băng chuyền tự động có khả năng phân loại sản phẩm dựa trên màu sắc.  
Sản phẩm được đưa qua cảm biến màu sắc (TCS34725), dữ liệu thu thập được xử lý bởi vi điều khiển STM32 để xác định màu.  
Sau đó, hệ thống điều khiển các khay phân loại (dùng servo) đưa sản phẩm đến đúng vị trí.  
LCD hiển thị số lượng sản phẩm theo từng màu.
Hệ thống sẽ dừng và còi sẽ báo hiệu nếu như hệ thống phát hiện được màu lạ, sau khi sử lý xong lỗi thì nhấn button để hệ thống hoạt động trở lại.

---

##  Yêu cầu phần cứng

| Thiết bị                    | Số lượng | Ghi chú thêm                         |
|----------------------------|----------|--------------------------------------|
| Vi điều khiển STM32F103C8T6| 1        | Bluepill                             |
| Cảm biến màu sắc TCS34725   | 1        | Phát hiện màu RGB                    |
| Stepper Nema 17 + Driver A4988      | 1        | Di chuyển băng chuyền (DM556/Nema17) |
| Servo SG90    | 1     | Điều khiển khay phân loại            |
| LCD 16x2 I2C               | 1        | Hiển thị số lượng sản phẩm           |
| Buzzer (còi cảnh báo)      | 1        | Báo động khi có lỗi                  |
| Adapter 12V              |1
| Khung cơ khí + băng chuyền | 1        | Tuỳ chỉnh theo thiết kế              |

### STM32F103C8T6:
![Image](https://github.com/user-attachments/assets/84a620f0-83c9-4010-932a-2a62334e7e30)
---
### TCS34725:
![Image](https://github.com/user-attachments/assets/a937848f-b6cb-4699-9bb9-c6062b437926)
---
### Nema 17 + A4988:
![Image](https://github.com/user-attachments/assets/4ae81e53-4df7-46e4-9673-2f183d2cb15c)
---
### Servo SG90:
![Image](https://github.com/user-attachments/assets/e438f128-206a-483a-b1cc-ad1457d8e05c)
---
### LCD 16x2:
![Image](https://github.com/user-attachments/assets/a19c9cd5-a059-4365-b278-0fcd1f32888a)
---
### Buzzer:
![Image](https://github.com/user-attachments/assets/35b319c1-6399-4b78-b3dd-d31d8c4dfa42)
---
### Adaper 12V:
![Image](https://github.com/user-attachments/assets/09cb3ccf-18b9-499f-91d1-740ba1150a39)
---
### Khung băng chuyền:
![Image](https://github.com/user-attachments/assets/c55bb06e-ff47-48ab-84ee-68fc08c87784)
---
## Demo:

