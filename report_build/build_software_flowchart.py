from __future__ import annotations

from pathlib import Path
from math import atan2, cos, sin, pi
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "project" / "deliverables" / "figures"
PNG_PATH = OUT_DIR / "图3-6_软件总体流程.png"
SVG_PATH = OUT_DIR / "图3-6_软件总体流程.svg"

W, H = 2600, 4350
FONT_REGULAR = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"

COLORS = {
    "bg": "#FFFFFF",
    "ink": "#1F2933",
    "main": "#EAF2F8",
    "decision": "#FFF8E7",
    "loop": "#F6F8FA",
    "success": "#EAF6EC",
    "recovery": "#FDEDEC",
    "muted": "#52616B",
}

img = Image.new("RGB", (W, H), COLORS["bg"])
draw = ImageDraw.Draw(img)
font_title = ImageFont.truetype(FONT_BOLD, 66)
font_group = ImageFont.truetype(FONT_BOLD, 45)
font_node = ImageFont.truetype(FONT_REGULAR, 37)
font_node_bold = ImageFont.truetype(FONT_BOLD, 38)
font_label = ImageFont.truetype(FONT_BOLD, 34)
font_note = ImageFont.truetype(FONT_REGULAR, 30)

svg: list[str] = [
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
    "<defs>",
    '<marker id="arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto" markerUnits="strokeWidth">',
    f'<path d="M0,0 L12,6 L0,12 z" fill="{COLORS["ink"]}"/>',
    "</marker>",
    "</defs>",
    f'<rect x="0" y="0" width="{W}" height="{H}" fill="{COLORS["bg"]}"/>',
]


def text_center(cx: int, cy: int, lines: list[str], font, size: int, bold: bool = False,
                fill: str = COLORS["ink"], line_gap: int = 12) -> None:
    heights = []
    widths = []
    for line in lines:
        box = draw.textbbox((0, 0), line, font=font)
        widths.append(box[2] - box[0])
        heights.append(box[3] - box[1])
    total = sum(heights) + line_gap * (len(lines) - 1)
    y = cy - total / 2
    svg_lines = []
    for i, line in enumerate(lines):
        x = cx - widths[i] / 2
        draw.text((x, y), line, font=font, fill=fill)
        baseline = y + heights[i]
        svg_lines.append(
            f'<tspan x="{cx}" y="{baseline:.1f}">{escape(line)}</tspan>'
        )
        y += heights[i] + line_gap
    weight = 700 if bold else 400
    svg.append(
        f'<text text-anchor="middle" font-family="Microsoft YaHei, SimSun, sans-serif" '
        f'font-size="{size}" font-weight="{weight}" fill="{fill}">' + "".join(svg_lines) + "</text>"
    )


def rounded_box(x: int, y: int, w: int, h: int, lines: list[str], *,
                fill: str = COLORS["main"], bold: bool = False, radius: int = 22,
                stroke_width: int = 5) -> None:
    draw.rounded_rectangle((x, y, x + w, y + h), radius=radius, fill=fill,
                           outline=COLORS["ink"], width=stroke_width)
    svg.append(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" '
        f'fill="{fill}" stroke="{COLORS["ink"]}" stroke-width="{stroke_width}"/>'
    )
    text_center(x + w // 2, y + h // 2, lines,
                font_node_bold if bold else font_node, 38 if bold else 37, bold=bold)


def ellipse_node(x: int, y: int, w: int, h: int, text: str, *, fill: str) -> None:
    draw.ellipse((x, y, x + w, y + h), fill=fill, outline=COLORS["ink"], width=5)
    svg.append(
        f'<ellipse cx="{x + w / 2}" cy="{y + h / 2}" rx="{w / 2}" ry="{h / 2}" '
        f'fill="{fill}" stroke="{COLORS["ink"]}" stroke-width="5"/>'
    )
    text_center(x + w // 2, y + h // 2, [text], font_node_bold, 38, bold=True)


def diamond(cx: int, cy: int, hw: int, hh: int, lines: list[str]) -> None:
    points = [(cx, cy - hh), (cx + hw, cy), (cx, cy + hh), (cx - hw, cy)]
    draw.polygon(points, fill=COLORS["decision"], outline=COLORS["ink"])
    draw.line(points + [points[0]], fill=COLORS["ink"], width=5, joint="curve")
    pts = " ".join(f"{x},{y}" for x, y in points)
    svg.append(f'<polygon points="{pts}" fill="{COLORS["decision"]}" stroke="{COLORS["ink"]}" stroke-width="5"/>')
    text_center(cx, cy, lines, font_node_bold, 37, bold=True, line_gap=8)


def group_box(x: int, y: int, w: int, h: int, title: str) -> None:
    draw.rounded_rectangle((x, y, x + w, y + h), radius=28, fill=COLORS["loop"],
                           outline=COLORS["muted"], width=4)
    # Dashed border overlay.
    dash = 24
    gap = 16
    for xx in range(x + 20, x + w - 20, dash + gap):
        draw.line((xx, y, min(xx + dash, x + w), y), fill=COLORS["muted"], width=5)
        draw.line((xx, y + h, min(xx + dash, x + w), y + h), fill=COLORS["muted"], width=5)
    for yy in range(y + 20, y + h - 20, dash + gap):
        draw.line((x, yy, x, min(yy + dash, y + h)), fill=COLORS["muted"], width=5)
        draw.line((x + w, yy, x + w, min(yy + dash, y + h)), fill=COLORS["muted"], width=5)
    svg.append(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="28" fill="{COLORS["loop"]}" '
        f'stroke="{COLORS["muted"]}" stroke-width="5" stroke-dasharray="24 16"/>'
    )
    text_center(x + w // 2, y + 52, [title], font_group, 45, bold=True)


def arrow(points: list[tuple[int, int]], label: str | None = None,
          label_pos: tuple[int, int] | None = None) -> None:
    draw.line(points, fill=COLORS["ink"], width=6, joint="curve")
    (x1, y1), (x2, y2) = points[-2], points[-1]
    angle = atan2(y2 - y1, x2 - x1)
    length = 23
    wing = 11
    p1 = (x2, y2)
    p2 = (x2 - length * cos(angle) + wing * sin(angle), y2 - length * sin(angle) - wing * cos(angle))
    p3 = (x2 - length * cos(angle) - wing * sin(angle), y2 - length * sin(angle) + wing * cos(angle))
    draw.polygon([p1, p2, p3], fill=COLORS["ink"])
    pts = " ".join(f"{x},{y}" for x, y in points)
    svg.append(
        f'<polyline points="{pts}" fill="none" stroke="{COLORS["ink"]}" stroke-width="6" '
        f'stroke-linejoin="round" stroke-linecap="round" marker-end="url(#arrow)"/>'
    )
    if label and label_pos:
        lx, ly = label_pos
        box = draw.textbbox((0, 0), label, font=font_label)
        pad = 8
        draw.rectangle((lx - pad, ly - pad, lx + box[2] + pad, ly + box[3] + pad), fill=COLORS["bg"])
        draw.text((lx, ly), label, font=font_label, fill=COLORS["ink"])
        svg.append(
            f'<rect x="{lx - pad}" y="{ly - pad}" width="{box[2] + 2 * pad}" height="{box[3] + 2 * pad}" fill="{COLORS["bg"]}"/>'
        )
        svg.append(
            f'<text x="{lx}" y="{ly + box[3]}" font-family="Microsoft YaHei, SimSun, sans-serif" '
            f'font-size="34" font-weight="700" fill="{COLORS["ink"]}">{escape(label)}</text>'
        )


# Title and closed-loop regions.
text_center(W // 2, 70, ["软件总体流程"], font_title, 66, bold=True)
group_box(70, 2360, 1160, 1230, "抓取闭环")
group_box(1370, 2360, 1160, 1230, "放置闭环")

# Main-flow arrows are drawn before nodes, so connectors stay behind boxes.
arrow([(1300, 240), (1300, 290)])
arrow([(1300, 430), (1300, 490)])
arrow([(1300, 630), (1300, 680)])
arrow([(1300, 880), (1300, 950)], "是", (1330, 895))
arrow([(1300, 1060), (1300, 1120)])
arrow([(1300, 1230), (1300, 1300)])
arrow([(1300, 1490), (1300, 1560)], "是", (1330, 1500))
arrow([(1300, 1690), (1300, 1750)])
arrow([(1300, 1890), (1300, 1950)])
arrow([(1300, 2090), (1300, 2140)])
arrow([(1300, 2320), (1300, 2390)], "是", (1330, 2325))

# IDLE polling loop.
arrow([(1000, 1400), (760, 1400), (760, 1175), (1000, 1175)], "否", (775, 1350))

# Failure lane from self-check and task validation to controlled recovery.
arrow([(1620, 780), (2570, 780), (2570, 3660), (2350, 3660)], "否", (1650, 720))
arrow([(1620, 2220), (2570, 2220), (2570, 3660), (2350, 3660)], "否", (1650, 2160))

# Task-type branches.
arrow([(1000, 2470), (650, 2470), (650, 2500)], "抓取", (785, 2405))
arrow([(1600, 2470), (1950, 2470), (1950, 2500)], "放置", (1635, 2405))

# Grasp-loop connectors.
arrow([(650, 2630), (650, 2680)])
arrow([(650, 2880), (650, 2930)], "是", (680, 2880))
arrow([(910, 2780), (1060, 2780), (1060, 2920)], "否", (925, 2718))
arrow([(1060, 3040), (1160, 3040), (1160, 2555), (1020, 2555)])
arrow([(650, 3040), (650, 3075)])
arrow([(650, 3185), (650, 3220)])
arrow([(650, 3330), (650, 3370)])
arrow([(650, 3470), (650, 3510)])

# Placement-loop connectors.
arrow([(1950, 2630), (1950, 2680)])
arrow([(1950, 2880), (1950, 2930)], "是", (1980, 2880))
arrow([(2210, 2780), (2355, 2780), (2355, 2920)], "否", (2225, 2718))
arrow([(2355, 3040), (2480, 3040), (2480, 2555), (2320, 2555)])
arrow([(1950, 3040), (1950, 3070)])
arrow([(1950, 3230), (1950, 3260)], "是", (1980, 3225))
arrow([(2200, 3150), (2355, 3150), (2355, 3280)], "否", (2215, 3088))
arrow([(2355, 3400), (2490, 3400), (2490, 2555), (2320, 2555)])
arrow([(1950, 3350), (1950, 3380)])
arrow([(1950, 3465), (1950, 3500)])

# Success and failure exits.
arrow([(430, 3540), (430, 3690), (780, 3690)], "是", (450, 3580))
arrow([(870, 3540), (870, 3610), (2000, 3610), (2000, 3650)], "否", (885, 3545))
arrow([(1730, 3570), (1350, 3570), (1350, 3650)], "是", (1580, 3505))
arrow([(2170, 3570), (2170, 3650)], "否", (2190, 3580))

# Completion returns to IDLE.
arrow([(780, 3755), (35, 3755), (35, 1175), (1000, 1175)], "记录日志并返回IDLE", (55, 3685))

# Recovery decision, retry path, and abort path.
arrow([(2050, 3770), (2050, 3830)])
arrow([(2300, 3930), (2580, 3930), (2580, 1005), (1650, 1005)], "是：回Home后重试", (2100, 3860))
arrow([(2050, 4030), (2050, 4070)], "否", (2080, 4025))
arrow([(2050, 4170), (2050, 4210)])

# Main nodes.
ellipse_node(1080, 130, 440, 110, "启动", fill=COLORS["success"])
rounded_box(600, 290, 1400, 140, ["加载系统配置", "舵机ID、限位、Home、机械尺寸、相机、模型后端"], bold=True)
rounded_box(680, 490, 1240, 140, ["发现串口并读取关节状态"], bold=False)
diamond(1300, 780, 320, 100, ["自检通过？"])
rounded_box(950, 950, 700, 110, ["返回 Home"], bold=True)
rounded_box(1000, 1120, 600, 110, ["进入 IDLE"], fill=COLORS["success"], bold=True)
diamond(1300, 1400, 300, 90, ["收到任务？"])
rounded_box(650, 1560, 1300, 130, ["采集 600×480、60 fps RGB 图像", "生成视觉状态"], bold=False)
rounded_box(650, 1750, 1300, 140, ["模型后端：云端 / 本地可选", "返回结构化任务"], bold=True)
rounded_box(600, 1950, 1400, 140, ["本地安全校验", "Schema、对象ID、动作白名单、风险检查"], bold=False)
diamond(1300, 2220, 320, 100, ["任务有效？"])
diamond(1300, 2470, 300, 80, ["任务类型"])

# Grasp-loop nodes.
rounded_box(280, 2500, 740, 130, ["检测目标与夹爪", "计算图像误差"], bold=False)
diamond(650, 2780, 260, 100, ["对准误差", "≤ 阈值？"])
rounded_box(930, 2920, 260, 120, ["小步调整", "关节位置"], fill=COLORS["decision"], bold=False)
rounded_box(220, 2930, 570, 110, ["闭合 J6 夹爪"], bold=False)
rounded_box(280, 3075, 740, 110, ["J6 接触检测与夹持判定"], bold=False)
rounded_box(300, 3220, 700, 110, ["抬起目标并验证稳定性"], bold=False)
rounded_box(300, 3370, 700, 100, ["视觉确认抓取结果"], bold=False)
diamond(650, 3540, 220, 70, ["抓取成功？"])

# Placement-loop nodes.
rounded_box(1580, 2500, 740, 130, ["检测放置目标与夹爪", "计算图像误差"], bold=False)
diamond(1950, 2780, 260, 100, ["对准误差", "≤ 阈值？"])
rounded_box(2220, 2920, 270, 120, ["小步调整", "保持对准"], fill=COLORS["decision"], bold=False)
rounded_box(1510, 2930, 600, 110, ["保持对准并小步下降"], bold=False)
diamond(1950, 3150, 250, 80, ["到达释放位置？"])
rounded_box(2220, 3280, 270, 120, ["继续校正", "并下降"], fill=COLORS["decision"], bold=False)
rounded_box(1510, 3260, 570, 90, ["J6 张开释放"], bold=False)
rounded_box(1600, 3380, 700, 85, ["视觉确认放置结果"], bold=False)
diamond(1950, 3540, 220, 70, ["放置成功？"])

# Completion / recovery / abort nodes.
rounded_box(780, 3650, 570, 105, ["任务完成"], fill=COLORS["success"], bold=True)
rounded_box(1700, 3650, 700, 120, ["进入受控恢复", "停止动作并保留故障信息"], fill=COLORS["recovery"], bold=True)
diamond(2050, 3930, 250, 100, ["故障可恢复？"])
rounded_box(1750, 4070, 600, 100, ["ABORT"], fill=COLORS["recovery"], bold=True)
ellipse_node(1810, 4210, 480, 100, "任务结束", fill=COLORS["recovery"])

# Figure note inside the image, kept subtle and print-safe.
note = "说明：抓取与放置均采用视觉闭环；任一环节失败均进入受控恢复或 ABORT。"
draw.text((85, 4270), note, font=font_note, fill=COLORS["muted"])
svg.append(
    f'<text x="85" y="4310" font-family="Microsoft YaHei, SimSun, sans-serif" font-size="30" '
    f'fill="{COLORS["muted"]}">{escape(note)}</text>'
)

svg.append("</svg>")
OUT_DIR.mkdir(parents=True, exist_ok=True)
img.save(PNG_PATH, dpi=(300, 300), optimize=True)
SVG_PATH.write_text("\n".join(svg), encoding="utf-8")
print(PNG_PATH)
print(SVG_PATH)
