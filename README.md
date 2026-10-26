# ECG Heart-Rate Classifier — FPGA RTL + Testbench (v3.1)

Hệ thống phân loại nhịp tim ECG chạy trên FPGA (Gowin), 50.4 MHz:
**ADC (AD8232, 360 Hz) → Circular Window (512 mẫu) → Feature Extraction (12 features) → MLP 12→8→4 (Q4.12) → UART/IRQ**.

- **Branch đang push**: `newest-update` (chứa toàn bộ dự án + fix v3.1 + tài liệu)
- **Trạng thái**: Top testbench E2E **8/8 PASSED** (IRQ fire, pipeline chạy đủ vòng), 7/7 unit testbench pass.
- **Report chi tiết việc debug**: [`ecg-thinh-uvm/docs/BUGFIX_8_8_top_tb.md`](ecg-thinh-uvm/docs/BUGFIX_8_8_top_tb.md)

---

## 1. Cấu trúc thư mục

```
.
├── README.md                          ← file này
└── ecg-thinh-uvm/
    ├── adc/adc_interface.v            ← Module 1: latch ADC + rate gate 360 Hz
    ├── window/window_buffer.v         ← Module 2: circular buffer 512 mẫu, overlap 256
    ├── feature/feature_extraction.v   ← Module 3: 12 features thống kê
    ├── ai/ai_classifier.v             ← Module 4: MLP 12→8→4, Q4.12 fixed-point
    ├── ai/mlp_weights_rom.v           ← ROM trọng số (placeholder, cần retrain)
    ├── control/control_fsm.v          ← Module 5 (v3.1): FSM điều khiển 10 trạng thái
    ├── apb/apb_slave_if.v             ← Module 6: APB slave (CSR + IRQ)
    ├── uart/uart_tx.v                 ← Module 7: UART TX 115200 8N1
    ├── top/top_ecg_classifier.v       ← Top-level: wire tất cả module + PL-USB
    ├── tb/                            ← Testbench (Questa)
    │   ├── tb_top_ecg_classifier.sv   ← E2E testbench (8 tests, hiện 8/8 pass)
    │   ├── tb_adc_interface.sv        ← Unit TB module 1
    │   ├── tb_window_buffer.sv        ← Unit TB module 2
    │   ├── tb_feature_extraction.sv   ← Unit TB module 3
    │   ├── tb_ai_classifier.sv        ← Unit TB module 4
    │   ├── tb_control_fsm.sv          ← Unit TB module 5
    │   ├── tb_apb_slave_if.sv         ← Unit TB module 6
    │   ├── tb_uart_tx.sv              ← Unit TB module 7
    │   ├── tb_feature_simple.sv / tb_window_simple.sv  ← TB tối giản (smoke test)
    │   └── uvm/                       ← UVM env cho APB (tb_apb_uvm)
    ├── sim/tb_top_ecg.sv              ← Testbench cũ (bản ECG trước khi có v3, archive)
    └── docs/
        ├── interfaces.md              ← Nguồn sự thật về interface/định dạng số
        ├── MODULE_REPORT.md           ← Báo cáo module gốc (baseline BƯỚC 0)
        ├── baseline_step0.md          ← Baseline BƯỚC 0
        ├── step1_port_mapping.md      ← Mapping port FPGA (PL-USB Gowin)
        └── BUGFIX_8_8_top_tb.md      ← Report debug + fix v3.1 (đọc trước tiên)
```

---

## 2. Từng module — chức năng & điểm chính

### 2.1 `adc/adc_interface.v` — ADC Interface
- Latch 8-bit ECG song song (AD8232) khi `adc_data_valid` pulse lên (rising-edge detect).
- **Rate gate v3.0**: counter `rate_timer` tick mỗi `sample_rate_div` clock → `sample_valid` 1 cycle.
  - Mặc định FPGA: `140000` @ 50.4 MHz = **360 Hz**.
  - Trong sim, top TB override `SYS_SAMPLE_RATE_DIV=20` cho chạy nhanh.
- `overflow_flag` (data đến khi chưa kịp nhả sample trước) → merge vào `sys_overflow`.
- **Lỗi đã sửa (v3.0)**: điều kiện `rate_timer >= (div-1)` thay `>= div` để chu kỳ đúng 140000 clock (không lệch 140001).

### 2.2 `window/window_buffer.v` — Circular Window 512
- RAM 512×8, write circular, baseline subtract (AD8232 ~169 @ 0 V).
- **Trigger v3.0**: `window_ready` (pulse 1 cycle) phát khi tổng mẫu đã ghi `total ≥ 512` và `(total−512) % 256 == 0` → mỗi 256 mẫu overlap (cửa 512 mẫu, 50% overlap), không bao giờ miss R-peak.
- **Read v3.0**: combinational `frame_out = ram[(oldest+read_addr) % 512]` (đổi từ registered read cũ lệch 1 mẫu).
- `soft_reset` → xóa write_ptr/sample_cnt/window_ready.

### 2.3 `feature/feature_extraction.v` — 12 Features
Đầu vào: 512 mẫu × 8-bit (đọc qua `read_en` strobe + `sample_addr`).
Đầu ra (mỗi 16-bit, Q4.12 với feature tỉ lệ):

| # | Feature | Ghi chú |
|---|---------|---------|
| 1 | f_rms | `sum_square >> 9` (truncation theo RTL gốc) |
| 2 | f_var | công thức giữ nguyên RTL gốc (golden bit-exact, owner quyết định) |
| 3 | f_peak | max sample |
| 4 | f_zc | zero-crossing count |
| 5 | f_ptp | peak − min |
| 6 | f_half_ratio | positive sample count, Q4.12 |
| 7 | f_crest | peak / RMS, Q4.12, guard divide-by-zero |
| 8 | f_max | = f_peak |
| 9 | f_rr | RR interval hiện tại |
| 10 | f_rr_prev | RR trước |
| 11 | f_rr_ratio | rr/rr_prev, Q4.12, guard rr_prev=0 → 4096 (1.0) |
| 12 | f_rr_diff | rr − rr_prev |

**Lỗi đã sửa (v3.0)**: alignment mẫu/địa chỉ (cùng cạnh, `sample_addr` phát cnt+1); so sánh signed/unsigned trong RR detection; port `soft_reset`.

### 2.4 `ai/ai_classifier.v` + `ai/mlp_weights_rom.v` — MLP 12→8→4
- Fixed-point **Q4.12, 16-bit**; ReLU ở hidden layer; Argmax tie-break giữ index thấp.
- FSM nội bộ: `LOAD→NORM→FC1→FC1_B→RELU→FC2→FC2_B→ARGMAX` (~161 chu kỳ), `done` pulse 1 cycle khi xong.
- `start && enable` → chạy; `soft_reset` → về IDLE; saturate MAC + `overflow` flag.
- `mlp_weights_rom`: 164 từ × 16-bit, **đang là placeholder — cần retrain với data 360 Hz trước khi dùng trên board**.

### 2.5 `control/control_fsm.v` — Master FSM (⚠️ fix v3.1)
10 trạng thái: `IDLE → LOAD → NORM → FC1 → FC1_B → RELU → FC2 → FC2_B → ARGMAX → DONE`.

- `LOAD`: chờ `window_ready` → phát `feature_start` (pulse 1 cycle, `feat_started` chống re-trigger) → chờ `feat_valid` → `NORM`.
- `NORM`: delay 12 cycle (feature normalize) → `FC1`.
- `FC1`: phát `mlp_start`, chờ `mlp_done` (kèm `MLP_TIMEOUT=10000` chống deadlock).
- `DONE`: `done=1` (→ `sys_done` → APB status + IRQ), `uart_send` pulse.

**Fix v3.1 (bug quan trọng nhất)**: cấu trúc cũ đặt `state <= next_state;` **và** `case(state) next_state <= …` đều non-blocking trong cùng block clocked → FSM trễ chính nó 1 cycle → dao động IDLE↔LOAD mỗi clock, bỏ lỡ mọi pulse 1-chu kỳ (`feat_valid`, `mlp_done`) → **không bao giờ có IRQ**. Sửa: `next_state` tính bằng **blocking**, `state` commit NBA **một lần sau endcase**. Chi tiết + bằng chứng trace: [`docs/BUGFIX_8_8_top_tb.md`](ecg-thinh-uvm/docs/BUGFIX_8_8_top_tb.md).

### 2.6 `apb/apb_slave_if.v` — APB Slave (CSR + IRQ)

| Addr | Tên | R/W | Nội dung |
|------|-----|-----|----------|
| 0x00 | CTRL | R/W | bit0 `enable`, bit1 `start` (pulse), bit4 `soft_reset` (pulse), bit2 `irq_en` |
| 0x04 | STATUS | R | bit0 busy, bit1 done, bit3 overflow |
| 0x08 | THRESH | R/W | [15:0] ngưỡng R-peak (default 80) |
| 0x0C | BAUD | R/W | [15:0] baud divisor (default 437 = 115200) |
| 0x10 | RESULT | R | [1:0] class_code (0 Normal, 1 Fast, 2 Slow, 3 Abnormal) |
| 0x14 | VERSION | R | 0x0200 |
| 0x18 | CONFIG | R/W | bit0 BLE, bit1 UART enable |

- `p_irq = sys_done && irq_en` (level), reset khi `presetn` thấp.
- `start`/`soft_reset` là pulse 1 cycle (ghi `pwdata[1]`/`pwdata[4]`), tự hạ sau.

### 2.7 `uart/uart_tx.v` — UART TX
8N1, LSB-first, `CLKS_PER_BIT=437` (115200 @ 50.4 MHz), baud divisor nạp được từ APB. Khung dữ liệu: `{6'b0, class_code}`.

### 2.8 `top/top_ecg_classifier.v` — Top Level
Wire tất cả module; ports ngoài theo `docs/step1_port_mapping.md` (PL-USB Gowin): `sys_clk, sys_rst_n, paddr/psel/penable/pwrite/pwdata/prdata, adc_data[7:0], adc_valid, uart_txd, irq, status_led[3:0]`.
`SYS_SAMPLE_RATE_DIV=140000` (mặc định, có thể override trong TB).

---

## 3. Testbench

| File | Phạm vi | Cách chạy |
|------|---------|-----------|
| `tb/tb_top_ecg_classifier.sv` | E2E: config APB → start → feed 768 mẫu → chờ IRQ → kiểm class/status/reset (8 tests) | `vlog -sv -mfcu` + `vsim` (Questa) |
| `tb/tb_*.sv` (unit) | Từng module | như trên, `vlog` 1 module + 1 TB |
| `tb/uvm/tb_apb_uvm.sv` | APB qua UVM env (phím tắt, env legacy) | `vcs`/`vsim -sv` |

**Kết quả hiện tại (v3.1)**:
- Top TB: **8/8 PASSED** (`Interrupt received!`, `class_code=00` — Normal; weight ROM là placeholder nên class_code chỉ đúng pipeline, không cam kết giá trị ML).
- 7 unit TB: **OK**, không regression.

### Chạy lại bằng lệnh (Questa 25.1, Windows)

```bash
# 1) Top-level E2E
vsim -c -mfcu -f ../apb/apb_slave_if.v ../adc/adc_interface.v ../window/window_buffer.v \
     ../feature/feature_extraction.v ../ai/ai_classifier.v ../ai/mlp_weights_rom.v \
     ../control/control_fsm.v ../uart/uart_tx.v ../top/top_ecg_classifier.v \
     tb_top_ecg_classifier.sv -work f_top
vsim -c -work f_top tb_top_ecg_classifier -do "run -all; quit -f"

# 2) Unit (ví dụ control FSM)
vlog -sv -mfcu -work f_ctrl ../control/control_fsm.v tb_control_fsm.sv
vsim -c -work f_ctrl tb_control_fsm -do "run -all; quit -f"
```

Lưu ý: `class_code = 00` trong TB là hợp lệ (weight placeholder). Trên board, phải nạp `mlp_weights_rom.hex` (164 từ × 16-bit) retrain với data 360 Hz.

---

## 4. Lịch sử thay đổi chính

| Bản | Nội dung |
|-----|----------|
| v2.x (baseline) | Baseline BƯỚC 0 — RTL + TB PPG cũ (xem `docs/baseline_step0.md`) |
| v3.0 (BƯỚC 2) | PPG→ECG: window overlap trigger theo tổng mẫu, feature alignment + signed RR, rate gate đúng chu kỳ, APB soft_reset/irq_en, FSM mlp_done handshake, renames TB |
| **v3.1** | **Fix cấu trúc control_fsm (blocking next_state) → top TB 8/8**; report `docs/BUGFIX_8_8_top_tb.md` |

## 5. Việc cần làm tiếp (hơn scope repo hiện tại)

- [ ] `mlp_weights_rom.hex`: retrain với dữ liệu ECG 360 Hz, nạp 164 từ × 16-bit.
- [ ] `top`: nối `clk_locked` thật khi dùng PLL (hiện tie 1).
- [ ] `docs/interfaces.md`: hoàn thành mục CTRL/STATUS/CONFIG bit-map (đang để ❓).
- [ ] Synthesis Gowin + port mapping PL-USB (tham khảo `docs/step1_port_mapping.md`).
