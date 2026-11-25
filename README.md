# LAOMO GAUGE ESP32

![老莫造物标识](assets/logo-horizontal.svg)

老莫造物的 ESP32 随车数据仪表源码。目标硬件是 ESP32-Touch-LCD-3.5（480 × 320、ST7796、FT6336、16 MB Flash、PSRAM），通过 BLE OBD-II 适配器读取标准 PID，并可把行程记录到 SD 卡。

## 当前状态

**源码公开版**（2026-11-25 首次发布）。`firmware/collect-v2/` 以实车验证过的 R6 采集固件为起点；LVGL 首页迁移放在后续独立分支，未完成前不会宣称「全 LVGL 界面」。

本仓库是**源码仓库**，不含预编译 BIN，也**不是**某块板上正在运行的镜像。仓库不包含个人行程、GPS 原始记录、车辆身份、设备绑定信息、Wi-Fi 配置、刷写日志或编译缓存。

## 构建

Arduino-ESP32 core 3.3.12，板型 `esp32:esp32:esp32`，选项：`FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,UploadSpeed=921600,FlashMode=dio,FlashFreq=80,CPUFreq=240`。另需 GFX Library for Arduino 1.6.9 和 TCA9554 0.1.2。字体来源和依赖许可见 [THIRD_PARTY.md](THIRD_PARTY.md)。本源码已在上述环境中干净编译；公开包未刷到开发板，屏幕字形和车载行为仍需单独验证。

Arduino IDE 可直接打开 `firmware/collect-v2/collect-v2.ino`。先按 [HARDWARE.md](HARDWARE.md) 核对目标板与 GPS 引脚。中文文案变化后，在仓库根目录运行 `python tools/cjk_font_gen.py` 重生字模；需安装 Pillow 并设置 `LAOMO_CJK_FONT` 指向 Noto Sans SC 字体文件。

## 后续待办

- 确认板卡配套 `TCA9554.h` 的包来源与再发布许可。
- 检查 Noto Sans SC 重生的中文字模在实屏上的显示效果。
- LVGL 首页迁移：完成后作为独立分支提交，进入主线前需替换第三方商标素材。

## 许可

代码与文档采用 MIT 许可，见 [LICENSE](LICENSE)。`assets/logo-horizontal.svg` 及「老莫造物 / LAOMO WORKS」名称与标识的版权单独保留，不随 MIT 授权，见 [assets/LICENSE-BRAND.md](assets/LICENSE-BRAND.md)。第三方依赖与字体见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 说明

行车时勿操作触摸屏。本固件提供数据展示与记录，不替代车辆诊断设备。
