# 硬件与接线

目标板：ESP32-Touch-LCD-3.5，ST7796 480 × 320 显示、FT6336 触摸、板载 microSD、16 MB Flash、PSRAM。固件引脚定义以 [`v2.h`](firmware/collect-v2/v2.h) 为准。

| 用途 | ESP32 GPIO | 扩展座标号 |
| --- | ---: | ---: |
| GPS 串口输入（接 GPS 的 TX/T） | 32 | 14 |
| GPS 供电 3.3 V | — | 32 |
| GPS 地 | — | 30 |
| SD CS | 15 | 板载 |
| SPI SCK / MISO / MOSI | 18 / 19 / 23 | 板载 |
| LCD CS / DC / 背光 | 5 / 27 / 25 | 板载 |
| BOOT 键 | 0 | 板载 |

PG-131F GPS 模块只接 `V→3V3`、`G→GND`、`T→GPIO32` 即可接收 9600 波特 NMEA；`R` 和 `P` 在当前固件中不用。先断电接线，确认插针编号与方向后再通电。GPS 室内或窗边可能有报文而尚未定位。

OBD 连接通过 BLE 适配器完成，板子与汽车 OBD 口之间没有直接导线。本代码的 BLE 服务与特征选择面向已验证的 FFF0 / FFF1 / FFF2 类适配器；其他适配器兼容性待验证，不能保证支持所有汽车或所有 ELM327 克隆设备。
