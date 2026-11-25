from pathlib import Path
import cv2
import numpy as np
import pytest
from doodle_ur7e.core import load_config
from doodle_ur7e.vision import marker_points, dictionary, estimate
from doodle_ur7e.image import sketch
ROOT = Path(__file__).resolve().parents[1]


def synthetic_board():
    paper = load_config(ROOT/'config/lab.yaml')['paper']
    K = np.array([[1000.,0,640],[0,1000.,480],[0,0,1]])
    rv = np.array([np.pi,0.,0.]); tv = np.array([-.075,.075,.55])
    image = np.full((960,1280),255,np.uint8)
    for i,obj in marker_points(paper).items():
        dst = cv2.projectPoints(obj,rv,tv,K,np.zeros(5))[0].reshape(4,2)
        marker = cv2.aruco.drawMarker(dictionary(),i,200)
        H = cv2.getPerspectiveTransform(np.float32([[0,0],[199,0],[199,199],[0,199]]),dst)
        warped = cv2.warpPerspective(marker,H,(1280,960),borderValue=255)
        image = np.minimum(image,warped)
    return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR),K,paper,tv


def test_metric_pose_from_rendered_markers():
    image,K,paper,tv = synthetic_board()
    T,error = estimate(image,K,np.zeros(5),paper)
    assert np.linalg.norm(T[:3,3]-tv) < .003
    assert T[2,2] < -.99
    assert error < 1.5


def test_missing_markers_and_blank_image():
    image,K,paper,_ = synthetic_board()
    with pytest.raises(ValueError): estimate(np.full_like(image,255),K,np.zeros(5),paper)
    with pytest.raises(ValueError): sketch(np.full_like(image,255))


def test_lineart_deterministic_and_aspect():
    image = np.full((200,400,3),255,np.uint8)
    cv2.rectangle(image,(80,40),(320,160),(0,0,0),3)
    a,b = sketch(image),sketch(image)
    assert a == b and len(a)>0
    pts = np.concatenate(a)
    assert pts[:,0].min()>=0 and pts.max()<=1
    extent = np.ptp(pts,axis=0)
    assert 1.7 < extent[0]/extent[1] < 2.3
