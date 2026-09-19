#!/usr/bin/env python3
"""Render the paper-ready Scout Mini core-framework data-flow figure."""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image
from reportlab.graphics import renderPDF, renderPM, renderSVG
from reportlab.graphics.shapes import Drawing, Ellipse, Path as ShapePath, Polygon, Rect, String
from reportlab.lib.colors import HexColor, white
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont


ROOT = Path(__file__).resolve().parent
WIDTH, HEIGHT = 1200, 760

NAVY = HexColor("#304B60")
TEXT = HexColor("#243447")
MUTED = HexColor("#52697A")
PANEL_BORDER = HexColor("#AFC4D5")
PANEL_FILL = HexColor("#F7FAFC")
BLUE_FILL = HexColor("#EAF2F8")
BLUE_BORDER = HexColor("#557FA3")
ORANGE_FILL = HexColor("#FFF3DB")
ORANGE_BORDER = HexColor("#C98B2E")
HYBRID_FILL = HexColor("#F8F0E5")
HYBRID_BORDER = HexColor("#A87635")
GREEN_FILL = HexColor("#EAF7F0")
GREEN_BORDER = HexColor("#4C8A68")
RED_FILL = HexColor("#FCEEEE")
RED_BORDER = HexColor("#B76565")
GRAY_FILL = HexColor("#EDF1F4")
GRAY_BORDER = HexColor("#52697A")


def register_fonts() -> None:
    pdfmetrics.registerFont(
        TTFont("FigureSans", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    )
    pdfmetrics.registerFont(
        TTFont("FigureSans-Bold", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
    )


def add_text(
    drawing: Drawing,
    x: float,
    y: float,
    value: str,
    size: float,
    *,
    bold: bool = False,
    color=TEXT,
    anchor: str = "middle",
) -> None:
    drawing.add(
        String(
            x,
            y,
            value,
            fontName="FigureSans-Bold" if bold else "FigureSans",
            fontSize=size,
            fillColor=color,
            textAnchor=anchor,
        )
    )


def add_panel(drawing: Drawing, x: float, y: float, w: float, h: float, title: str) -> None:
    drawing.add(
        Rect(
            x,
            y,
            w,
            h,
            rx=12,
            ry=12,
            fillColor=PANEL_FILL,
            strokeColor=PANEL_BORDER,
            strokeWidth=1.4,
        )
    )
    add_text(drawing, x + 14, y + h - 22, title, 16, bold=True, anchor="start")


def add_node(
    drawing: Drawing,
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    lines: tuple[str, ...],
    fill,
    stroke,
    *,
    shape: str = "round",
    title_size: float = 14,
    body_size: float = 10.5,
    stroke_width: float = 1.5,
) -> None:
    if shape == "ellipse":
        drawing.add(
            Ellipse(
                x + w / 2,
                y + h / 2,
                w / 2,
                h / 2,
                fillColor=fill,
                strokeColor=stroke,
                strokeWidth=stroke_width,
            )
        )
    elif shape == "diamond":
        drawing.add(
            Polygon(
                [x + w / 2, y + h, x + w, y + h / 2, x + w / 2, y, x, y + h / 2],
                fillColor=fill,
                strokeColor=stroke,
                strokeWidth=stroke_width,
            )
        )
    else:
        drawing.add(
            Rect(
                x,
                y,
                w,
                h,
                rx=10,
                ry=10,
                fillColor=fill,
                strokeColor=stroke,
                strokeWidth=stroke_width,
            )
        )

    entries = [(title, title_size, True)] + [(line, body_size, False) for line in lines]
    total = sum(size + 1.5 for _, size, _ in entries)
    cursor = y + h / 2 + total / 2 - entries[0][1]
    for value, size, bold in entries:
        add_text(drawing, x + w / 2, cursor, value, size, bold=bold)
        cursor -= size + 1.5


def add_arrow(
    drawing: Drawing,
    points: tuple[tuple[float, float], ...],
    *,
    color=MUTED,
    width: float = 1.5,
    dashed: bool = False,
    arrow_size: float = 7,
) -> None:
    path = ShapePath()
    path.moveTo(*points[0])
    for point in points[1:]:
        path.lineTo(*point)
    path.fillColor = None
    path.strokeColor = color
    path.strokeWidth = width
    if dashed:
        path.strokeDashArray = [6, 5]
    drawing.add(path)

    (x1, y1), (x2, y2) = points[-2], points[-1]
    angle = math.atan2(y2 - y1, x2 - x1)
    left = (
        x2 - arrow_size * math.cos(angle - math.pi / 6),
        y2 - arrow_size * math.sin(angle - math.pi / 6),
    )
    right = (
        x2 - arrow_size * math.cos(angle + math.pi / 6),
        y2 - arrow_size * math.sin(angle + math.pi / 6),
    )
    drawing.add(
        Polygon(
            [x2, y2, left[0], left[1], right[0], right[1]],
            fillColor=color,
            strokeColor=color,
            strokeWidth=0.5,
        )
    )


def add_label(
    drawing: Drawing,
    x: float,
    y: float,
    value: str,
    *,
    color=MUTED,
    size: float = 9.5,
) -> None:
    width = pdfmetrics.stringWidth(value, "FigureSans", size) + 8
    drawing.add(
        Rect(
            x - width / 2,
            y - 3,
            width,
            size + 4,
            rx=3,
            ry=3,
            fillColor=white,
            strokeColor=None,
        )
    )
    add_text(drawing, x, y, value, size, color=color)


def build() -> Drawing:
    register_fonts()
    d = Drawing(WIDTH, HEIGHT)

    # Publication panels.
    add_panel(d, 20, 545, 1160, 195, "(a) Task-level semantic navigation")
    add_panel(d, 20, 330, 1160, 190, "(b) NeuPAN navigation and sensed-data loop")
    add_panel(d, 140, 45, 1040, 260, "(c) Event-triggered, dual-gate obstacle traversal assessment")

    # Main task flow: two-row snake to preserve readable type at two-column width.
    add_arrow(d, ((330, 682), (455, 682)))
    add_arrow(d, ((715, 682), (855, 682)))
    add_arrow(d, ((995, 655), (995, 625)))
    add_arrow(d, ((855, 598), (765, 598)))
    add_arrow(d, ((410, 598), (285, 598)))

    # Global goal dispatch to NeuPAN.
    add_arrow(d, ((165, 570), (165, 530), (960, 530), (960, 482)), width=1.8)

    # Execution loop.
    add_arrow(d, ((305, 452), (400, 452)))
    add_arrow(d, ((680, 452), (810, 452)))
    add_arrow(d, ((960, 425), (960, 382), (840, 382)))
    add_arrow(d, ((520, 377), (345, 377)))
    add_arrow(d, ((195, 404), (195, 435)), color=HexColor("#7D8F9D"))

    # Event inputs: sensor snapshot, NeuPAN path, and Module 4 task state.
    add_arrow(d, ((175, 435), (175, 232), (430, 232)), color=GREEN_BORDER)
    add_arrow(d, ((960, 435), (960, 318), (610, 318), (610, 259)))
    add_arrow(
        d,
        ((760, 598), (1165, 598), (1165, 232), (790, 232)),
        color=HexColor("#6B7D8B"),
        dashed=True,
    )

    # Parallel visual reasoning and deterministic dual gate.
    add_arrow(d, ((520, 205), (520, 195), (360, 195), (360, 185)), color=GREEN_BORDER)
    add_arrow(d, ((700, 205), (700, 195), (825, 195), (825, 185)), color=ORANGE_BORDER)
    add_arrow(d, ((360, 135), (575, 112)), color=MUTED)
    add_arrow(d, ((825, 125), (645, 112)), color=MUTED)
    add_arrow(d, ((755, 85), (820, 85)), color=GREEN_BORDER, width=1.8)
    add_arrow(d, ((465, 85), (390, 85)), color=RED_BORDER, width=1.8)

    # Authorization/supervision edges return to the execution loop.
    add_arrow(
        d,
        ((1120, 84), (1168, 84), (1168, 320), (540, 320), (540, 425)),
        color=GREEN_BORDER,
        dashed=True,
    )
    add_arrow(
        d,
        ((970, 113), (1128, 113), (1128, 312), (680, 312), (680, 350)),
        color=GREEN_BORDER,
        dashed=True,
    )
    add_arrow(
        d,
        ((170, 84), (145, 84), (145, 312), (720, 312), (720, 350)),
        color=RED_BORDER,
        dashed=True,
    )

    # Nodes are drawn after edges to keep arrow lines out of labels.
    add_node(
        d, 50, 655, 280, 54, "Natural-language instruction", ("/user_instruction",),
        white, NAVY, shape="ellipse",
    )
    add_node(
        d, 455, 655, 260, 54, "Module 1 · Intent parsing",
        ("LLM → strict semantic JSON", "goal · relation · constraints · strategy"),
        ORANGE_FILL, ORANGE_BORDER,
    )
    add_node(
        d, 855, 655, 280, 54, "Module 2 · Semantic grounding",
        ("entity resolution on static YAML graph", "goal ID · pose · topology · map revision"),
        BLUE_FILL, BLUE_BORDER,
    )
    add_node(
        d, 855, 570, 280, 55, "Module 3 · Fail-closed verification",
        ("entity → topology → geometry → Navfn", "map · costmap · TF/pose → VerificationResult"),
        BLUE_FILL, BLUE_BORDER,
    )
    add_node(
        d, 410, 565, 355, 66, "Module 4 · Execution bridge",
        (
            "VLM coarse waypoints + deterministic sanitization",
            "connected free space · snapping · FIFO/watchdogs",
            "terminal iff active target = ∅ and queue = ∅",
        ),
        HYBRID_FILL, HYBRID_BORDER, body_size=9.5,
    )
    add_node(
        d, 50, 570, 235, 55, "Verified FIFO goals", ("/clicked_point",),
        white, BLUE_BORDER,
    )

    add_node(
        d, 45, 425, 260, 54, "On-board sensing",
        ("RGB · CameraInfo · raw LiDAR", "TF / odometry"),
        GREEN_FILL, GREEN_BORDER,
    )
    add_node(
        d, 400, 425, 280, 54, "Authorization-scoped scan filter",
        ("default: full scan retained", "only the tracked cluster is temporarily removed"),
        BLUE_FILL, BLUE_BORDER, body_size=9.5,
    )
    add_node(
        d, 810, 425, 300, 54, "NeuPAN local navigation",
        ("/scan + FIFO goal → local path / raw velocity",),
        GRAY_FILL, GRAY_BORDER,
    )
    add_node(
        d, 520, 350, 320, 54, "Fail-closed velocity gate",
        ("STOP while verifying · speed limit if authorized",),
        BLUE_FILL, BLUE_BORDER,
    )
    add_node(
        d, 45, 350, 300, 54, "Scout Mini / Isaac Sim",
        ("/cmd_vel · physical/world feedback",),
        white, NAVY, shape="ellipse",
    )

    add_node(
        d, 430, 205, 360, 54, "Traversal event coordinator",
        ("path-corridor obstacle detection · synchronized snapshot", "LiDAR cluster → RGB ROI · task/track correlation"),
        BLUE_FILL, BLUE_BORDER, body_size=9.5,
    )
    add_node(
        d, 230, 135, 260, 50, "Identity gate",
        ("Target detector candidates", "+ verifier ROI validation"),
        GREEN_FILL, GREEN_BORDER,
    )
    add_node(
        d, 650, 125, 350, 60, "Pushability gate",
        ("Structured scene assessment", "+ decision adapter with vehicle state"),
        ORANGE_FILL, ORANGE_BORDER,
    )
    add_node(
        d, 465, 50, 290, 70, "Both gates pass?",
        ("identity verified ∧ P(pushable) ≥ 0.70",),
        white, NAVY, shape="diamond", body_size=9.5,
    )
    add_node(
        d, 820, 55, 300, 58, "Time-bounded authorization",
        ("tracked cluster only · speed ≤ 0.15 m/s", "expiry/pass/watchdog → restore full scan"),
        GREEN_FILL, GREEN_BORDER, body_size=9.5,
    )
    add_node(
        d, 170, 55, 220, 58, "Reject / provider error",
        ("retain full scan · avoid or stop",),
        RED_FILL, RED_BORDER, body_size=9.5,
    )

    # Compact data-contract labels.
    add_label(d, 392, 689, "text")
    add_label(d, 785, 689, "NavigationIntent")
    add_label(d, 1040, 635, "ResolutionResult")
    add_label(d, 810, 605, "verified only")
    add_label(d, 345, 605, "sanitized points")
    add_label(d, 510, 536, "next goal")
    add_label(d, 352, 459, "/scan_raw")
    add_label(d, 746, 459, "/scan")
    add_label(d, 892, 390, "/neupan_cmd_vel_raw")
    add_label(d, 430, 384, "gated command")
    add_label(d, 845, 322, "initial path")
    add_label(d, 440, 189, "same image + ROI", color=GREEN_BORDER)
    add_label(d, 750, 189, "same image + ROI + vehicle state", color=ORANGE_BORDER)

    # Legend.
    legend_y = 16
    drawing_items = (
        (210, ORANGE_FILL, ORANGE_BORDER, "generative reasoning"),
        (430, BLUE_FILL, BLUE_BORDER, "deterministic verification / control"),
        (730, GREEN_FILL, GREEN_BORDER, "sensing / perception"),
    )
    for x, fill, stroke, label in drawing_items:
        d.add(Rect(x, legend_y, 24, 14, rx=3, ry=3, fillColor=fill, strokeColor=stroke))
        add_text(d, x + 32, legend_y + 2, label, 9.5, anchor="start")
    path = ShapePath()
    path.moveTo(925, legend_y + 7)
    path.lineTo(965, legend_y + 7)
    path.strokeColor = MUTED
    path.strokeWidth = 1.5
    path.strokeDashArray = [6, 5]
    path.fillColor = None
    d.add(path)
    add_text(d, 975, legend_y + 2, "authorization / supervisory state", 9.5, anchor="start")

    return d


def main() -> None:
    drawing = build()
    renderSVG.drawToFile(drawing, str(ROOT / "scout_core_dataflow.svg"))
    renderPDF.drawToFile(drawing, str(ROOT / "scout_core_dataflow.pdf"))
    raster = build()
    raster_scale = 300.0 / 72.0
    raster.scale(raster_scale, raster_scale)
    raster.width = WIDTH * raster_scale
    raster.height = HEIGHT * raster_scale
    renderPM.drawToFile(
        raster,
        str(ROOT / "scout_core_dataflow_300dpi.png"),
        fmt="PNG",
        dpi=72,
    )
    png_path = ROOT / "scout_core_dataflow_300dpi.png"
    with Image.open(png_path) as image:
        image.save(png_path, dpi=(300, 300), optimize=True)


if __name__ == "__main__":
    main()
