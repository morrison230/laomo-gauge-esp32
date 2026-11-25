# 第三方依赖与素材

- Arduino-ESP32 core 3.3.12：构建依赖，仓库不包含其源码。
- GFX Library for Arduino 1.6.9：显示依赖，仓库不包含其源码。上游 [Arduino_GFX](https://github.com/moononournation/Arduino_GFX) 的 `license.txt` 为 SPDX `BSD-2-Clause`，版权声明为 `Copyright (c) 2012 Adafruit Industries`。
- TCA9554 0.1.2：I²C 扩展器依赖，仓库不包含其源码；本机安装包的 `LICENSE` 为 SPDX `MIT`，版权声明为 `Copyright (c) 2025-2025 Rob Tillaart`。
- Noto Sans SC：`firmware/collect-v2/cjk_font.h` 的 16×16 点阵来源。使用 SPDX `OFL-1.1`，完整声明见 [NOTO-SANS-SC-OFL.txt](third_party/NOTO-SANS-SC-OFL.txt)。字模由 `tools/cjk_font_gen.py` 重新生成，原 Windows 字体点阵未收入此包。字体本体不收入仓库。

`assets/logo-horizontal.svg` 的使用边界见 [品牌声明](assets/LICENSE-BRAND.md)。铃木商标和汽车厂商图形未收入此包。
