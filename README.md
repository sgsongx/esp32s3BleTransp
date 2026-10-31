# ESP32-S3 BLE / UART 透传

基于 ESP-IDF Bluedroid GATT Server 例程，实现单个 BLE 中央设备与 UART 的双向二进制透传，不增加包头、换行或字符编码转换。当前工程使用 ESP-IDF 6.0、ESP32-S3。

## 默认行为

| 项目 | 设置 |
| --- | --- |
| 广播名 | `bletransp` |
| 服务 UUID | `0x00FF`，完整 UUID `000000ff-0000-1000-8000-00805f9b34fb` |
| 数据特征 UUID | `0xFF01`，完整 UUID `0000ff01-0000-1000-8000-00805f9b34fb` |
| BLE → UART | 向数据特征执行 Write 或 Write Without Response |
| UART → BLE | 订阅数据特征的 Notify，CCCD `0x2902` 写入 `01 00` |
| 默认连接间隔 | 广播声明偏好 30 ms；连接后申请一次 30 ms，latency=0，监督超时 4 s |
| BLE PHY | 默认 LE 1M，可通过宏选择 LE 2M / LE Coded |
| UART | UART1，115200，8N1，无流控 |
| UART 引脚 | TX=GPIO17，RX=GPIO18 |
| 本地最大 ATT MTU | 517；未协商时为 23 |

上电完成 GATT 服务创建后开始广播，连接后停止广播，断线自动恢复广播。每次连接都需要重新订阅通知，MTU 也会重置。已移除原例程的第二个演示服务、固定测试通知和测试读数据。

30 ms 是外设申请的偏好，中央设备最终决定是否采用。后续由中央设备请求的合法连接参数更新由协议栈处理，应用不会改回 30 ms。实际间隔和 PHY 更新结果会打印到日志。PHY 需要双方共同支持，选择 2M 或 Coded 并不保证中央设备接受。

## 宏配置与接线

修改 [main/ble_transp_config.h](main/ble_transp_config.h) 后重新编译，也可通过编译器 `-D` 覆盖同名宏：

| 宏 | 用途 |
| --- | --- |
| `BLE_TRANSP_UART_NUM` | 串口外设，如 `UART_NUM_1` / `UART_NUM_2` |
| `BLE_TRANSP_UART_BAUD_RATE` | UART 波特率 |
| `BLE_TRANSP_UART_TX_PIN` / `BLE_TRANSP_UART_RX_PIN` | TX / RX GPIO |
| `BLE_TRANSP_UART_DATA_BITS` / `BLE_TRANSP_UART_PARITY` / `BLE_TRANSP_UART_STOP_BITS` | 数据位、校验和停止位 |
| `BLE_TRANSP_UART_FLOW_CTRL` | 默认 `UART_HW_FLOWCTRL_DISABLE`；可选择 `UART_HW_FLOWCTRL_CTS_RTS` |
| `BLE_TRANSP_UART_RTS_PIN` / `BLE_TRANSP_UART_CTS_PIN` | 使用硬件流控时必须配置实际 GPIO |
| `BLE_TRANSP_PHY` | `ESP_BLE_GAP_PHY_1M_PREF_MASK` / `ESP_BLE_GAP_PHY_2M_PREF_MASK` / `ESP_BLE_GAP_PHY_CODED_PREF_MASK` |
| `BLE_TRANSP_CODED_PHY_OPTIONS` | Coded 编码偏好：`ESP_BLE_GAP_PHY_OPTIONS_NO_PREF` / `ESP_BLE_GAP_PHY_OPTIONS_PREF_S2_CODING` / `ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING` |

ESP32-S3 TX → 对端 RX，ESP32-S3 RX ← 对端 TX，并连接 GND。使用 3.3 V TTL 串口；RS-232 / RS-485 需要匹配的收发器。选 GPIO 时检查具体开发板是否已经占用这些引脚。使用 RTS/CTS 时交叉连接 RTS → CTS、CTS ← RTS。

默认 UART0 保留为日志控制台，透传 UART 不输出日志。编译时会阻止透传 UART 与日志 UART 相同；若要用 UART0，请先把控制台改到 USB 或关闭 UART 控制台。

## 编译、烧录

在已经激活的 ESP-IDF 6.0 终端中执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

`COMx` 替换为开发板烧录/日志端口，UART1 透传数据通过另一个串口适配器收发。退出 monitor 使用 `Ctrl+]`。

工程已在当前 `sdkconfig` 和默认配置中启用 BLE 5 和扩展广播支持。虽然使用扩展广播 API，实际发出的是传统可连接 ADV_IND 广播，手机正常扫描即可发现。若沿用旧的自定义 `sdkconfig`，请在 menuconfig 中设置：

```ini
CONFIG_BT_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_BLE_50_FEATURES_SUPPORTED=y
# CONFIG_BT_BLE_42_FEATURES_SUPPORTED is not set
CONFIG_BT_BLE_50_EXTEND_ADV_EN=y
```

## 数据与缓冲约定

- BLE 写入按收到的原始字节顺序进入 UART 发送队列；串口发送在独立任务中进行，GATT 回调不等待串口。Write 成功响应表示数据已入队，不表示外部串口设备已经接收。
- 每次普通写入长度不超过 `min(协商 MTU - 3, 512)`。更长数据由客户端分多次普通写入；不支持 Prepare / Execute Write，收到后返回不支持错误，避免提前输出未提交的数据。
- UART 输入按 `min(协商 MTU - 3, 512)` 分成通知。例如默认 MTU=23 时每条最多 20 字节。BLE 通知边界与 UART 输入边界不对应，接收端应按字节流拼接。
- BLE → UART 队列容量为 8 个数据块。队满时 Write 返回 `ESP_GATT_NO_RESOURCES`，客户端应降低发送速率并重试。Write Without Response 没有远端错误响应，队满只记录日志，可能丢数据。
- UART RX 缓冲为 8192 字节。BLE 拥塞时保留当前通知块重试；持续超速输入会溢出，程序打印警告并清理 RX 缓冲。持续传输建议配置 RTS/CTS，并由发送端配合流控。
- 未连接或未订阅时持续读取并丢弃 UART 输入，不提供离线存储。每次连接或订阅状态切换时 RX 任务会清空串口输入缓冲，切换期间的字节不保留；断线、退订会取消应用中等待重试的旧通知块。断线前已经接受的 BLE 写入仍会发送到 UART。
- Notify 成功表示提交到协议栈，没有应用层接收确认。本工程没有增加加密配对、重传协议或业务帧格式；需要这些能力时应在客户端和 UART 对端约定。

## 验证方法

主机自动回归测试直接编译并调用生产 C 实现，使用真实 ESP-IDF BLE 公共头文件，模拟 UART、FreeRTOS 队列和蓝牙发送接口。需要主机 GCC 或 Clang：

```powershell
python tests/run_tests.py
```

脚本优先读取 `IDF_PATH`，也可从现有 `build/project_description.json` 获取 SDK 路径；测试构建产物放在临时目录，运行后自动清理。已通过 5 组回归：二进制写入及队满、CCCD 校验、MTU 分包和拥塞重试、UART 轮询及状态切换清理、连接及广播参数。ESP32-S3 固件完整编译通过；2M + UART2/230400、自定义引脚以及 Coded S8 + RTS/CTS 的宏组合也通过目标编译器检查。

硬件验证步骤：

1. 编译并烧录。通过日志确认 UART 参数、`Advertising as bletransp`；手机 BLE 工具扫描并连接 `bletransp`，找到服务 `0x00FF` 和特征 `0xFF01`。
2. 向特征写入十六进制 `00 01 7F 80 FE FF`，串口工具使用同样 UART 参数，应收到完全一致的 6 字节。分别测试 Write 和 Write Without Response，不应出现原例程的固定测试包。
3. 开启 Notify，再由串口工具发送相同二进制数据；手机应收到一致的字节流。先保持 MTU=23，再协商更大的 MTU，用 512 字节递增模式检查分包拼接和字节顺序。
4. 同时进行双向收发。使用 Write 时根据响应节奏发送，验证队满返回失败后可重试。长时间高速串口输入时检查流控和溢出日志。
5. 从中央设备请求新的连接间隔，观察 `Connection update` 日志；间隔不应被应用反复改回 30 ms。分别编译 1M、2M、Coded，使用支持对应 PHY 的中央设备检查 `PHY update` 日志。
6. 发送期间断开连接，确认恢复广播；重新连接后先不开 Notify，应无旧的待发通知块。再次订阅后验证双向收发。

主机回归测试只验证软件逻辑；广播、PHY、连接参数协商和真实串口吞吐仍需按上述步骤在开发板上验证。
