"""Metric four-marker board, with +Z out of paper and +Y up."""
from pathlib import Path
import cv2
import numpy as np
from .core import transform


def marker_points(paper):
    w,h,s = paper['width'], paper['height'], paper['marker_size'] / 2
    # IDs 0 BL, 1 BR, 2 TR, 3 TL. Detector corner order TL TR BR BL.
    return {i: np.array([[x-s,y+s,0],[x+s,y+s,0],[x+s,y-s,0],[x-s,y-s,0]], np.float32)
            for i,(x,y) in enumerate([(0,0),(w,0),(w,h),(0,h)])}


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def estimate(image, K, D, paper):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary(), parameters=params)
    if ids is None or set(ids.ravel()) != {0,1,2,3} or len(ids) != 4:
        raise ValueError('need exactly board markers 0,1,2,3, all visible')
    objects = marker_points(paper)
    obj = np.concatenate([objects[int(i)] for i in ids.ravel()])
    img = np.concatenate(corners).reshape(-1,2)
    ok, rv, tv = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('paper PnP failed')
    R = cv2.Rodrigues(rv)[0]
    if tv[2,0] <= 0 or np.dot(R[:,2], tv[:,0]) >= 0:
        raise ValueError('paper pose faces away from camera')
    projected = cv2.projectPoints(obj, rv, tv, K, D)[0].reshape(-1,2)
    error = float(np.sqrt(np.mean(np.sum((projected-img)**2,axis=1))))
    if error > 1.5:
        raise ValueError(f'paper reprojection error {error:.2f}px >1.5px')
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = tv[:,0]
    return T, error


def board(paper, path):
    # SVG physical dimensions, padding sufficient for labels and quiet zones.
    w,h,s = [paper[k]*1000 for k in ['width','height','marker_size']]
    pad = s/2 + 12
    W,H = w+2*pad,h+2*pad
    text = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">',
            f'<rect width="{W}" height="{H}" fill="white"/>']
    for i,pts in marker_points(paper).items():
        pixels = cv2.aruco.drawMarker(dictionary(), i, 6)
        x,y = pts[0,:2]*1000
        x,y = x+pad, h-y+pad
        for row,col in np.argwhere(pixels == 0):
            text.append(f'<rect x="{x+col*s/6}" y="{y+row*s/6}" width="{s/6+.001}" height="{s/6+.001}"/>')
    text += [f'<text x="{pad+20}" y="{H-4}" font-size="3">100% scale; marker centers {w:g} x {h:g} mm</text>', '</svg>']
    Path(path).write_text('\n'.join(text))
