# SPEC: Firmware mới cho robot bàn ESP32-S3 (Xiaozhi-compatible)

> Mục đích: viết lại firmware từ đầu, **ưu tiên ổn định + tính năng**, cosmetic sau.
> Phần cứng **không đổi**. Server Xiaozhi **không đổi**. Giao thức **phải tương thích** với `xiaozhi-esp32-server` đang chạy.
> Tài liệu này là kết quả đo đạc thực tế trên phần cứng thật, không phải suy đoán.

---

## 1. Mục tiêu & thứ tự ưu tiên

| Ưu tiên | Nội dung |
|---|---|
| **P0** | Chạy 24/7 không treo, không điếc, không mất màn. Tự phục hồi mọi lỗi. |
| **P0** | Nghe → hiểu → trả lời bằng giọng nói, tiếng Việt, qua server Xiaozhi hiện có. |
| **P1** | Servo 2 trục (gật/lắc/quay), LED trạng thái, nút bấm. |
| **P1** | Gọi tool qua JSON trên đường LLM (giờ, thời tiết, tìm kiếm...). |
| **P2** | Màn hình OLED: mắt biểu cảm, icon trạng thái. |
| **P2** | OTA, đổi WiFi qua portal. |

**Quy tắc vàng:** không có tính năng nào được phép làm hỏng P0. Nếu màn hình nghi ngờ gây treo → màn hình phải nhường.

---

## 2. Phần cứng (đã xác nhận, KHÔNG sửa)

### 2.1 Board
- **ESP32-S3 N16R8**: 16 MB flash, 8 MB PSRAM (octal), không PSRAM thì không đủ chỗ.
- Target: `esp32s3`, ESP-IDF **v5.5.x** (`idf.py set-target esp32s3`).
- MAC thiết bị: `28:84:85:65:2b:2c`.

### 2.2 Sơ đồ chân (bảng chuẩn — dùng đúng, sai là hỏng module)

| Chức năng | Module | Chân ESP32 | Ghi chú |
|---|---|---|---|
| Mic SCK | INMP441 | **GPIO5** | I2S số 0 (RX), 24 kHz, mono |
| Mic WS | INMP441 | **GPIO4** | |
| Mic SD (data) | INMP441 | **GPIO6** | INMP441 ở chế độ kênh trái (L/R nối GND) |
| Loa BCLK | MAX98357A | **GPIO15** | I2S số 1 (TX), 16 kHz |
| Loa LRC/WS | MAX98357A | **GPIO16** | |
| Loa DIN | MAX98357A | **GPIO7** | |
| Màn SDA | SH1106 OLED | **GPIO41** | I2C, địa chỉ **0x3C**, 128×64 |
| Màn SCL | SH1106 OLED | **GPIO42** | **Không có trở kéo ngoài** (xem 2.4) |
| Servo ngang | SG90/MG90 | **GPIO11** | LEDC ch0, 50 Hz, 500–2500 µs |
| Servo dọc | SG90/MG90 | **GPIO12** | LEDC ch1, 50 Hz, 500–2500 µs |
| Nút BOOT | onboard | **GPIO0** | nhấn giữ 2 s = đổi chế độ màn hình |
| Nút Volume + | onboard | **GPIO40** | nhấn = +10, giữ = 100 |
| Nút Volume − | onboard | **GPIO39** | nhấn = −10, giữ = 0 |
| LED RGB | WS2812 onboard | **GPIO48** | **Bị vỏ che — không dùng làm chỉ báo chính** |

### 2.3 Nguồn (nguồn gốc của phần lớn lỗi "treo")
- MAX98357A và servo dùng chung rail **3V3** với ESP32.
- Cấp bằng USB laptop (~500 mA) là **không đủ**: lúc loa nói + servo quay + WiFi phát, rail sụt → I2C/I2S lỗi hàng loạt.
- **Bắt buộc trong firmware** (không sửa được bằng phần cứng):
  - Âm lượng mặc định **≤ 50/100**.
  - Công suất phát WiFi **≤ 10 dBm** (`CONFIG_ESP_PHY_MAX_TX_POWER=40`).
- Khuyến nghị cho người dùng: cấp bằng củ sạc **5 V / 2 A**.

### 2.4 Giới hạn bus màn hình (đã đo, rất quan trọng)
- Không có điện trở kéo ngoài; chỉ dựa vào pull-up nội (~45 kΩ).
- Đo thực tế trên bus này:
  - 100 kHz → nhiều lỗi
  - **50 kHz → ít lỗi nhất** ✅
  - 25 kHz → **nhiều lỗi hơn cả 100 kHz** (nghĩa là vấn đề là nhiễu, không phải rise-time)
- Vì vậy: **dùng 50 kHz**, và mọi thứ khác phải chịu được việc bus thỉnh thoảng lỗi.

---

## 3. Giao thức kết nối server (BẮT BUỘC tương thích)

Tham chiếu chuẩn: repo `78/xiaozhi-esp32` (protocol version 1). Dưới đây là các điểm phải khớp với server đang chạy.

### 3.1 Địa chỉ
- OTA (lấy cấu hình + kiểm tra firmware): `http://192.168.1.106:8000/xiaozhi/ota/`
  → URL này nhập qua portal WiFi, lưu NVS namespace `wifi`, key `ota_url`. **Không hardcode vào source.**
- WebSocket: `ws://192.168.1.106:8000/xiaozhi/v1/`
- Server trả `timezone_offset` (hiện = **7**) → cộng vào epoch khi `settimeofday`.

### 3.2 Bản tin `hello` (client → server)
```json
{"type":"hello","version":1,"features":{"mcp":true},"transport":"websocket",
 "audio_params":{"format":"opus","sample_rate":16000,"channels":1,"frame_duration":60}}
```
Server trả `hello` kèm `session_id` + `audio_params`.

### 3.3 Audio
- **Opus, 16 kHz, mono, frame 60 ms** (960 mẫu/frame).
- **Tần số phát của loa phải = 16000 Hz.** Nếu để 24 kHz, server 16 kHz → mọi câu trả lời đi qua resampler → méo tiếng.
- Mic phần cứng chạy 24 kHz → resample về 16 kHz trước khi đưa vào AFE/wake word.

### 3.4 Bản tin điều khiển (tóm tắt)
| Hướng | Type | Ý nghĩa |
|---|---|---|
| C→S | `listen` {state: start/stop/detect, mode} | bắt đầu/kết thúc nghe |
| S→C | `stt` {text} | chữ nhận được |
| S→C | `llm` {emotion} | cảm xúc (dùng để đổi mặt/servo) |
| S→C | `tts` {state: start/sentence_start/stop, text} | bắt đầu câu / nội dung câu |
| C→S | `abort` | ngắt lời |
| C↔S | `iot`, `mcp` | điều khiển thiết bị / tool MCP |

### 3.5 Tool call qua JSON (quan trọng — đây là cách robot "thông minh")
Server gửi danh sách function trong request LLM. Model có thể trả về **một khối JSON thuần**:
```json
{"tool": "get_now", "arguments": {}}
```
→ Firmware **không được đọc JSON này ra loa**. Phải lọc sạch trước khi đưa text vào TTS.

### 3.6 Server đang chạy gì (không cần sửa)
- ASR: **Sherpa-onnx zipformer VI 30M int8** (local, 0,15–0,25 s).
- TTS: **Edge TTS `vi-VN-HoaiMyNeural`**, rate −15%, độ dài mỗi câu ~2 s.
- Function server-side: `get_laptop_temp`, `get_now`, `get_weather_vn`, `web_search_vn`, `play_music`, `play_online`.
- MCP: `voice-tools` (wiki_lookup, web_search, get_weather).
- Session tự đóng sau 10 s im lặng (`close_connection_no_voice_time: 10`).

---

## 4. Kiến trúc firmware đề xuất

```
┌──────────────┐   ┌──────────────┐   ┌───────────────┐
│ audio_input  │──▶│ AFE (VAD +   │──▶│ state machine │
│ prio 8       │   │ wake word)   │   │ prio 3        │
│ 24k→16k      │   │ prio 4       │   │               │
└──────────────┘   └──────────────┘   └───────┬───────┘
                                              │
┌──────────────┐   ┌──────────────┐   ┌───────▼───────┐
│ audio_output │◀──│ Opus decode  │◀──│ WebSocket     │
│ prio 5       │   │ 16 kHz       │   │ prio 5        │
└──────────────┘   └──────────────┘   └───────────────┘
┌──────────────┐   ┌──────────────┐
│ servo (LEDC) │   │ display      │  ← phải KHÔNG BAO GIỜ chặn 3 khối trên
│ prio 3       │   │ prio 2-3     │
└──────────────┘   └──────────────┘
```

Quy tắc:
1. **Mic prio cao nhất** (8) — không bao giờ để đói CPU.
2. **Loa (5) > màn hình (2–3)** — màn chậm thì được, tiếng hụt thì không.
3. Màn hình **chỉ vẽ khi có thay đổi**; mục tiêu ≤ 5 khung/giây.
4. Không task nào được giữ khóa chung với task màn hình.

---

## 5. YÊU CẦU ỔN ĐỊNH — mỗi dòng dưới đây là một lỗi thật đã xảy ra

> Đây là phần quan trọng nhất của tài liệu. Vi phạm = robot treo đúng như bản cũ.

### 5.1 Đường I2C màn hình
1. **KHÔNG BAO GIỜ** gọi API `esp_lcd_panel_io_*` khi chưa kiểm tra SDA/SCL đang ở mức cao.
   ESP-IDF panel-IO dùng **timeout = −1 (chờ vĩnh viễn)**; bus kẹt = task vẽ treo cứng mãi mãi.
   → Trước mỗi khung: đọc mức 2 chân, nếu có chân thấp thì **bỏ khung** + gọi `lv_disp_flush_ready()` + trả về OK.
2. **KHÔNG dùng `ESP_ERROR_CHECK`** trên đường chạy runtime (kể cả trong hàm init gọi lặp).
   `ESP_ERROR_CHECK` = `abort()` = panic = **vòng lặp reboot**. Mọi lỗi phải được xử lý mềm.
3. Rớt khung → **đánh dấu dirty** → vẽ lại toàn mặt trong ≤ 3 s. Không để màn hiển thị nửa khung.
4. Watchdog màn hình phải dựa trên **số khung hoàn thành thật** (`đã xin vẽ` vs `đã vẽ xong`),
   **không** dựa trên "task vẽ còn chạy". Task vẽ vẫn tick 10 lần/giây khi màn đã chết → watchdog mù.
5. Watchdog **không được `vTaskDelay()` dài** (4–30 phút) bên trong vòng giám sát. Phải canh mốc thời gian và tiếp tục quan sát.
6. Cứu bus: `i2c_master_bus_reset()` → thử init lại; gõ 9 nhịp clock bằng GPIO (đủ **byte**, không dừng giữa chừng) rồi STOP; phải bật lại pull-up nội sau khi tháo driver.
7. Nếu không cứu được sau 90 s → **reboot**. Không bao giờ có trạng thái "bỏ cuộc vĩnh viễn".

### 5.2 Mic (âm thanh vào)
8. **Một lần đọc I2S thất bại KHÔNG được làm task mic thoát.** Bản cũ: rớt 1 lần → `break` → robot **điếc vĩnh viễn**.
   → Bỏ qua slice 32 ms, đọc tiếp.
9. Timeout đọc I2S phải **hữu hạn và đủ lớn cho lần đầu** (DMA còn rỗng): ≥ 300 ms (đã dùng 500 ms).
10. Watchdog mic: **25 s không có lần đọc thành công nào** trong khi đang chờ lệnh → **reboot**.
11. Wake word phải chạy trên AFE (VAD + WakeNet), **không** chặn bởi màn hình.

### 5.3 Âm thanh ra
12. Tần số phát = **16000 Hz** (bằng server). Không resample.
13. Hàng đợi phát ≥ **8 khung** (~480 ms) — 2 khung (120 ms) bị hụt khi CPU bận.
14. Sau khi bật output, đẩy **~120 ms im lặng** trước khung thật (amp + DMA nguội → mất âm tiết đầu).
15. Ghi I2S phải có timeout hữu hạn; ghi hụt thì bỏ khung đó, không treo task.

### 5.4 Vòng điều khiển & khóa
16. Vòng sự kiện chính đập **1 nhịp/giây**; **30 s không đập → reboot**.
    (Watchdog của chip chỉ canh task rảnh — mọi task kẹt chờ khóa thì watchdog **im lặng**.)
17. Mọi khóa (mutex) trên đường runtime phải **try-lock với timeout ≤ 2 s**, và **chỉ unlock khi đã lock được**.
    Bản cũ: lock thất bại vẫn unlock → mở khóa của người khác → crash dây chuyền.
18. Không có trạng thái "chờ vô hạn" nào trong toàn bộ firmware.

### 5.5 Servo
19. Một mutex/spinlock duy nhất bảo vệ PWM; hai task cùng lái servo = giật và hỏng bánh răng.
20. Khi rảnh → về giữa (`90°, 90°`) để giảm dòng giữ.
21. Lệnh "gật 1 cái" = **đúng 1 lần** xuống–lên. Bản cũ lặp 3 lần trong một lệnh.
22. Debounce lệnh chuyển động **3 s** (VAD có thể lặp cùng một câu).

### 5.6 Màn hình OLED / burn-in
23. Không tắt màn khi rảnh (robot cắm điện 24/7). **Ngủ đêm 01:00–07:00** theo giờ server (+7).
24. Tương phản `0x81 0x4F` (không phải 0x7F) để chậm lão hoá.
25. Đang nói → mắt **đứng yên** (mỗi lần vẽ lại đúng lúc loa rút dòng đỉnh = nguồn nhiễu).

---

## 6. Anti-pattern — những thứ đã làm hỏng bản cũ, đừng lặp lại

| Anti-pattern | Hậu quả thật |
|---|---|
| Vẽ full-screen 10 fps | Bus I2C bão hoà 100%, không có khe cho lỗi |
| `DisplayLockGuard` bao quanh cả giao dịch I2C | 1 khung kẹt → khóa cả state machine, servo, LED |
| Đổi biểu cảm theo từng câu khi đang nói | Vẽ đúng lúc loa rút dòng → sập bus |
| Animation task prio 5 > LVGL 3 | Đảo ưu tiên, mic đói CPU |
| `portMAX_DELAY` trong đường runtime | Treo vĩnh viễn khi lỗi |
| Watchdog "bỏ cuộc sau 2 lần" | Robot nằm chết, không tự cứu |
| TTS đọc nguyên khối JSON tool call | Robot đọc `{"tool": ...}` ra loa |
| Liệt kê số bằng dấu phẩy liền (`Một, hai, ba...`) | 10 số trong 1 câu = đọc 2 s không nghỉ, không ai hiểu. Phải tách thành câu: `Một. Hai. Ba.` |
| Trả lời từ trí nhớ cho câu hỏi thời sự | Sai kiến thức (VD: nói chủ tịch nước là người **đã rời ghế**) |

---

## 7. Hợp đồng với LLM (giữ nguyên hành vi đang chạy tốt)

- LLM hiện là **Hermes Agent** qua bridge OpenAI-compatible `http://192.168.1.40:8643/v1/chat/completions` (LAN).
- Bridge xử lý nhanh, không qua LLM: chào, giờ/ngày, nhiệt laptop, thời tiết, gật/lắc, tắt máy 2 bước.
- **Câu hỏi thời sự** ("ai đang là...", "giá vàng", "tin mới") → bridge tự tra web rồi mới trả lời.
- Nếu firmware mới muốn bỏ bridge: phải tự cài tool `get_now`, `get_weather_vn`, `web_search_vn` và **luật chống trả lời từ trí nhớ**.

---

## 8. Test nghiệm thu (bắt buộc trước khi coi là "xong")

| # | Test | Tiêu chí đạt |
|---|---|---|
| 1 | Cắm điện 2 giờ | Không reboot, không treo màn, gọi "Alexa" vẫn nghe |
| 2 | Nói 10 lượt liên tiếp | Mỗi lượt phản hồi < 3 s, tiếng rõ, không mất âm tiết đầu |
| 3 | Gọi lúc màn đang ngủ đêm | Màn sáng ngay, trả lời bình thường |
| 4 | "Gật đầu 1 cái" | Đúng 1 lần gật, không lặp |
| 5 | "Đếm từ một đến mười" | Nghe rõ **từng số**, có khoảng nghỉ |
| 6 | Hỏi "chủ tịch nước hiện nay là ai" | Trả lời đúng người **đương nhiệm** |
| 7 | Rút WiFi 30 s rồi cắm lại | Tự kết nối lại, không cần khởi động lại |
| 8 | Đo 5 phút: log lỗi I2C | ≤ 10 lỗi/5 phút, **0 khung bị bỏ** |
| 9 | Đo RAM 5 phút | `min-ever` không giảm dần (không rò rỉ) |
| 10 | Rút loa ra khi đang nói | Không treo, không panic |

---

## 9. Giới hạn đã biết (đừng cố vượt)

1. **Bus màn hình 2 dây, 50 kHz** → tối đa ~5–8 khung/giây. Không thể mượt như màn SPI. Muốn mượt phải đổi màn SPI (phần cứng).
2. **Không có trở kéo ngoài** → sẽ luôn có vài lỗi I2C lẻ. Firmware phải sống chung (bỏ khung + vẽ lại), không được coi là lỗi fatal.
3. **Nguồn 3V3 chung** với loa 3 W và servo → âm lượng và công suất WiFi phải giữ thấp.
4. **LED bị vỏ che** → mọi chỉ báo trạng thái phải lên màn hình (icon), không dựa vào LED.
5. **Sherpa ASR** nghe khá tốt nhưng không có hotword tùy ý; wake word phải là model có sẵn (`wn9_alexa`, `wn9_jarvis_tts`...).

---

## 10. Tài nguyên tham khảo

- Protocol gốc: repo `78/xiaozhi-esp32` (protocol version 1) và `xinnan-tech/xiaozhi-esp32-server`.
- Mã nguồn bản cũ (để đối chiếu, **không copy kiến trúc**): repo `hyvuonggia/xiaozhi-deskemoji-taibai`.
- Partition table bắt buộc (16 MB, OTA kép):
```
nvs,      data, nvs,     0x9000,    0x4000
otadata,  data, ota,     0xd000,    0x2000
phy_init, data, phy,     0xf000,    0x1000
model,    data, spiffs,  0x10000,   0xF0000
ota_0,    app,  ota_0,   0x100000,  6M
ota_1,    app,  ota_1,   0x700000,  6M
```
- `sdkconfig` bắt buộc:
```
CONFIG_ESP_TASK_WDT_TIMEOUT_S=10
CONFIG_ESP_TASK_WDT_PANIC=y
CONFIG_ESP_PHY_MAX_TX_POWER=40      # 10 dBm
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions/v1/16m.csv"
```
