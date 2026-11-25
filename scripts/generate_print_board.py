#!/usr/bin/env python3
"""Generate the printable A4 board from the runtime geometry and ArUco dictionary.

Offline publishing tool only. Requires reportlab, opencv-contrib-python, PyYAML,
numpy and scipy; these PDF dependencies are not required to run the robot.
"""
import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/doodle_ur7e'))
from doodle_ur7e.vision import dictionary, marker_points


def generate(config, output):
    paper = yaml.safe_load(Path(config).read_text())['paper']
    w, h, s, margin = [paper[k] * 1000 for k in
                        ('width', 'height', 'marker_size', 'margin')]
    if not (0 < margin < min(w, h) / 2 and s > 0 and
            w + s <= 185 and h + s <= 185):
        raise ValueError('Board does not fit this A4 layout or has invalid dimensions')
    # PDF +Y is up, exactly as in the runtime paper frame. Origin = ID 0 center.
    ox, oy = (210 - w) / 2, (297 - h) / 2
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    c = canvas.Canvas(str(output), pagesize=A4, invariant=1)
    c.setTitle('Doodle UR7e - A4 drawing and ArUco board')
    c.setAuthor('Doodle_Ur7e')
    c.scale(mm, mm)

    def label(x, y, text, size=9):
        c.setFont('Helvetica', size / mm)
        c.drawCentredString(x, y, text)

    label(105, 280, 'DOODLE / UR7e', 17)
    label(105, 272, 'A4 210 x 297 mm | Print actual size / 100% | Do not fit to page', 9)
    label(105, 257, 'FAR EDGE - away from robot base', 10)
    c.setLineWidth(.3)
    c.line(105, 241, 105, 252)
    c.line(105, 252, 103, 249)
    c.line(105, 252, 107, 249)
    label(118, 245, '+Y paper', 8)

    for ident, points in marker_points(paper).items():
        # Same 6 x 6 cells (4 data + black border) as vision.board().
        if hasattr(cv2.aruco, 'drawMarker'):
            bits = cv2.aruco.drawMarker(dictionary(), ident, 6)
        else:
            bits = cv2.aruco.generateImageMarker(dictionary(), ident, 6)
        left, top = points[0, :2].astype(float) * 1000 + [ox, oy]
        for row, col in np.argwhere(bits == 0):
            c.rect(left + col*s/6, top - (row+1)*s/6,
                   s/6, s/6, stroke=0, fill=1)
        cx, cy = points.mean(axis=0)[:2] * 1000 + [ox, oy]
        label(cx, cy - s/2 - 5, f'ID {ident}', 9)

    # Light dashed guide stays outside the runtime drawing envelope.
    c.setStrokeColorRGB(.65, .65, .65)
    c.setLineWidth(.15)
    c.setDash(1.5, 1.5)
    c.rect(ox+margin-.6, oy+margin-.6,
           w-2*margin+1.2, h-2*margin+1.2, fill=0, stroke=1)
    c.setDash()
    c.setFillColorRGB(.35, .35, .35)
    label(105, oy+margin-4, f'DRAWING AREA {w-2*margin:g} x {h-2*margin:g} mm', 8)
    c.setFillColorRGB(0, 0, 0)
    label(105, 48, 'NEAR EDGE - robot base on this side', 10)
    label(105, 41, 'Paper origin: ID 0 center | +X: ID 0 to ID 1 | +Z: out of page', 8)
    label(105, 35, f'DICT_4X4_50 | Marker black edge {s:g} mm | Centers {w:g} x {h:g} mm', 8)
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(.2)
    c.line(80, 23, 130, 23)
    c.line(80, 21, 80, 25)
    c.line(130, 21, 130, 25)
    label(105, 17, '50 mm ruler check - measure after printing', 8)
    c.showPage()
    c.save()
    print(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/lab.yaml')
    parser.add_argument('--output', type=Path,
                        default=ROOT/'docs/print/ur7e_drawing_board_A4.pdf')
    args = parser.parse_args()
    generate(args.config, args.output)
