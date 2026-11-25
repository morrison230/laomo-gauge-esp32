#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cjk_font_gen.py —— 从界面源码里抽取实际用到的汉字，生成 16x16 点阵头文件。

为什么需要它：
    屏幕只有 ASCII 字库，中文必须自带点阵。上一次是手工列了一个字表，
    结果界面后来改成「系统诊断」「发动机水温」等文案，字模里没有这些字，
    屏幕上就出现大片空白 / 只剩半个词。这个工具直接把源码里的字符串扫出来，
    用到什么字就生成什么字，并在 --check 模式下报出"缺字"。

用法：
    python tools/cjk_font_gen.py                 # 扫描并重写 cjk_font.h
    python tools/cjk_font_gen.py --check         # 只检查，不写文件（缺字则返回 1）
    python tools/cjk_font_gen.py --preview out.png  # 顺带渲染一张字模预览

字模格式（与 e_ui.ino 的 cjkGlyph 一致）：
    每个字 16 行，每行 2 字节，高位在左；行内 1 bit = 1 像素。
"""
import argparse
import os
import re
import sys

SRC_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "firmware", "collect-v2")
# 扫描 firmware/collect-v2 下全部 .ino；单个文件清单会漏掉诊断文案。


def scan_files():
    if not os.path.isdir(SRC_DIR):
        raise SystemExit("源码目录不存在：%s" % SRC_DIR)
    files = sorted(f for f in os.listdir(SRC_DIR) if f.endswith(".ino"))
    if not files:
        raise SystemExit("源码目录没有 .ino 文件：%s" % SRC_DIR)
    return files


OUT_H = os.path.join(SRC_DIR, "cjk_font.h")

# 基础字符集：即使当前文案没用到，也先备着，避免下次改文案又要回来补
BASE = (
    "主界面蓝牙设备无线网络系统诊断返回扫描连接重新取消保存未已绑定中数据记录行数错误"
    "就绪触摸校准请点击左上角右下完成当前名称地址信号先等待按此服务识别超时键运行间"
    "复位原因停止开始高级模式真实采集模拟测试车辆转速车速发动机水温电压暂不可用正常"
    "异常定位卫星存储密码输入串口命令第页上下所选发现无配置卡读核验断电恢复负荷支持位图"
    "个共次自动结果会存起来用手指在方框内"
    # 2026-11-14 追加：阶段/结果/通知/封存相关（文案主要写在 c_obd.ino 与 a_store.ino）
    "探针已发送收到搜索解析调用回调注册未知封存写入阶段设备找到候选台"
    "超时无数据无法解析发送失败最近失败轮询通信观察未已"
)

def find_font():
    path = os.environ.get("LAOMO_CJK_FONT", "")
    if not path or not os.path.isfile(path):
        raise SystemExit("找不到 Noto Sans SC 字体，请设置 LAOMO_CJK_FONT 为该字体的路径")
    return path


def scan_used_chars():
    """从源码的字符串字面量里抽出所有非 ASCII 字符。"""
    used = set()
    files = scan_files()
    for fn in files:
        path = os.path.join(SRC_DIR, fn)
        if not os.path.exists(path):
            continue
        src = open(path, encoding="utf-8").read()
        # 去掉块注释与行注释，避免注释里的汉字混进字模
        src = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
        src = re.sub(r"//[^\n]*", " ", src)
        for lit in re.findall(r'"((?:[^"\\]|\\.)*)"', src):
            for ch in lit:
                if ord(ch) > 0x7F:
                    used.add(ch)
    return used


def render(chars):
    from PIL import Image, ImageDraw, ImageFont

    font = ImageFont.truetype(find_font(), 16)
    # Noto Sans SC is a variable font whose default weight is Thin (100).
    # Medium glyphs retain connected strokes without closing 16 px counters.
    font.set_variation_by_axes([500])
    glyphs = {}
    empty = []
    for ch in chars:
        img = Image.new("L", (16, 16), 0)
        d = ImageDraw.Draw(img)
        d.text((1, -2), ch, fill=255, font=font)
        px = img.load()
        rows = []
        for y in range(16):
            hi = lo = 0
            for x in range(8):
                if px[x, y] > 110:
                    hi |= 1 << (7 - x)
            for x in range(8, 16):
                if px[x, y] > 110:
                    lo |= 1 << (7 - (x - 8))
            rows.append((hi, lo))
        ink = sum(bin(a).count("1") + bin(b).count("1") for a, b in rows)
        if ink < 6:
            empty.append(ch)
        glyphs[ch] = rows
    return glyphs, empty


def build_header(chars, glyphs):
    lines = []
    lines.append("/* cjk_font.h —— 16x16 1bpp 中文字模（自动生成，请勿手改）")
    lines.append(" *")
    lines.append(" * 由 tools/cjk_font_gen.py 扫描 firmware/collect-v2 下全部 .ino 的字符串字面量生成，")
    lines.append(" * 共 %d 个字，按码点升序排列（cjkIndex 用二分查找，顺序不能乱）。" % len(chars))
    lines.append(" * 改文案后请重新运行： python tools/cjk_font_gen.py")
    lines.append(" * 字形来源：Noto Sans SC，字重 500（SIL OFL 1.1）；见 third_party/NOTO-SANS-SC-OFL.txt。")
    lines.append(" */")
    lines.append("#pragma once")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("#define CJK_W 16")
    lines.append("#define CJK_H 16")
    lines.append("#define CJK_N %d" % len(chars))
    lines.append("")
    lines.append("static const uint32_t cjk_code[CJK_N] = {")
    for i in range(0, len(chars), 8):
        chunk = ", ".join("0x%04X" % ord(c) for c in chars[i:i + 8])
        lines.append("  " + chunk + ",")
    lines.append("};")
    lines.append("")
    lines.append("static const uint8_t cjk_glyph[CJK_N][32] = {")
    for c in chars:
        body = ",".join("0x%02X,0x%02X" % (hi, lo) for hi, lo in glyphs[c])
        lines.append("  {%s},  /* %s */" % (body, c))
    lines.append("};")
    lines.append("")
    return "\n".join(lines)


def check_only(chars):
    if not os.path.exists(OUT_H):
        print("字模文件不存在：%s" % OUT_H)
        return 1
    src = open(OUT_H, encoding="utf-8").read()
    m = re.search(r"cjk_code\[CJK_N\] = \{(.*?)\};", src, re.S)
    if not m:
        print("无法解析 cjk_code 数组")
        return 1
    codes = {int(x, 16) for x in m.group(1).replace("\n", "").split(",") if x.strip()}
    have = {chr(c) for c in codes}
    missing = sorted(chars - have)
    if missing:
        print("[FAIL] 界面用到了但字模缺失 %d 个字：%s" % (len(missing), "".join(missing)))
        return 1
    print("[PASS] 界面用到的 %d 个汉字全部在字模内（字模共 %d 字）" % (len(chars), len(have)))
    return 0


def main():
    global SRC_DIR, OUT_H
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只检查缺字，不写文件")
    ap.add_argument("--preview", metavar="PNG", help="渲染字模预览图")
    ap.add_argument("--src-dir", default=SRC_DIR, help="目标源码目录（默认现场源码）")
    args = ap.parse_args()
    SRC_DIR = args.src_dir
    OUT_H = os.path.join(SRC_DIR, "cjk_font.h")

    used = scan_used_chars()
    all_chars = sorted(set(used) | set(BASE), key=ord)

    if args.check:
        return check_only(used)

    glyphs, empty = render(all_chars)

    with open(OUT_H, "w", encoding="utf-8", newline="\n") as f:
        f.write(build_header(all_chars, glyphs))

    print("已生成 %s" % OUT_H)
    print("  界面实际用字 %d 个；并入基础字符集后共 %d 个" % (len(used), len(all_chars)))
    print("  文件大小 %.1f KB（每字 32 字节）" % (len(all_chars) * 32 / 1024.0))
    if empty:
        print("  [WARN] 以下字渲染墨迹过少，可能字体不支持：%s" % "".join(empty))

    if args.preview:
        from PIL import Image
        cols = 24
        rows = (len(all_chars) + cols - 1) // cols
        img = Image.new("1", (cols * 16, rows * 16), 0)
        px = img.load()
        for gi, c in enumerate(all_chars):
            gx = (gi % cols) * 16
            gy = (gi // cols) * 16
            for y in range(16):
                hi, lo = glyphs[c][y]
                for x in range(8):
                    if (hi >> (7 - x)) & 1:
                        px[gx + x, gy + y] = 1
                for x in range(8):
                    if (lo >> (7 - x)) & 1:
                        px[gx + 8 + x, gy + y] = 1
        img.resize((cols * 16 * 2, rows * 16 * 2), Image.NEAREST).save(args.preview)
        print("  预览已写 %s" % args.preview)
    return 0


if __name__ == "__main__":
    sys.exit(main())
