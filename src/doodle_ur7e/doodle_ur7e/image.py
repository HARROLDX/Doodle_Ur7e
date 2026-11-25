"""Contour scan adapted from DoodleDroid / LingDong linedraw (see third_party)."""
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageOps
from .core import route, validate_strokes

def getdots(IM):
    print("getting contour points...")
    PX = IM.load()
    dots = []
    w,h = IM.size
    for y in range(h-1):
        row = []
        for x in range(1,w):
            if PX[x,y] == 255:
                if len(row) > 0:
                    if x-row[-1][0] == row[-1][-1]+1:
                        row[-1] = (row[-1][0],row[-1][-1]+1)
                    else:
                        row.append((x,0))
                else:
                    row.append((x,0))
        dots.append(row)
    return dots

def connectdots(dots):
    print("connecting contour points...")
    contours = []
    for y in range(len(dots)):
        for x,v in dots[y]:
            if v > -1:
                if y == 0:
                    contours.append([(x,y)])
                else:
                    closest = -1
                    cdist = 100
                    for x0,v0 in dots[y-1]:
                        if abs(x0-x) < cdist:
                            cdist = abs(x0-x)
                            closest = x0

                    if cdist > 3:
                        contours.append([(x,y)])
                    else:
                        found = 0
                        for i in range(len(contours)):
                            if contours[i][-1] == (closest,y-1):
                                contours[i].append((x,y,))
                                found = 1
                                break
                        if found == 0:
                            contours.append([(x,y)])
        contours = [c for c in contours if not (c[-1][1] < y-1 and len(c)<4)]
    return contours


def sketch(image, resolution=320):
    if image is None or image.size == 0:
        raise ValueError('image not readable')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    h, w = gray.shape
    size = (max(2, round(w * resolution / max(w,h))), max(2, round(h * resolution / max(w,h))))
    im = ImageOps.autocontrast(Image.fromarray(gray).resize(size), 10)
    edges = cv2.Canny(cv2.GaussianBlur(np.array(im), (3,3), 0), 100, 200)
    # Two perpendicular scan passes recover horizontal and vertical contours.
    lines = connectdots(getdots(Image.fromarray(edges)))
    other = connectdots(getdots(Image.fromarray(edges.T)))
    lines += [[(y,x) for x,y in line] for line in other]
    result = []
    for line in lines:
        if len(line) < 5:
            continue
        pts = cv2.approxPolyDP(np.asarray(line, np.float32), 1.0, False).reshape(-1,2)
        if len(pts) >= 2 and np.linalg.norm(np.diff(pts,axis=0),axis=1).sum() >= 5:
            # Normalize using full image canvas; preserve aspect and whitespace.
            pts = (pts - (np.array(size)-1)/2) / (max(size)-1) + .5
            result.append(pts.tolist())
    return [x.tolist() for x in route(validate_strokes(result))]


def preview(lines, path):
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 1000" width="700" height="700">',
             '<rect width="1000" height="1000" fill="white"/>']
    for line in validate_strokes(lines):
        points = ' '.join(f'{x*1000:.2f},{y*1000:.2f}' for x,y in line)
        parts.append(f'<polyline points="{points}" fill="none" stroke="black" stroke-width="1.5"/>')
    Path(path).write_text('\n'.join(parts+['</svg>']))


def prepare(input_path, output):
    out = Path(output); out.mkdir(parents=True, exist_ok=True)
    lines = sketch(cv2.imread(str(input_path)))
    (out/'strokes.json').write_text(json.dumps(lines))
    preview(lines, out/'preview.svg')
    print(f'{len(lines)} strokes saved to {out}')
