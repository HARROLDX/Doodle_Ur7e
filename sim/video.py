#!/usr/bin/env python3
"""Compose rendered frames with an inset reconstructed ONLY from measured tip samples."""
import argparse, subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'runs/isaac'
p=argparse.ArgumentParser();p.add_argument('--speed',type=int,default=60);args=p.parse_args()
trace=np.loadtxt(OUT/'actual_tip.csv',delimiter=',')
frames=sorted((OUT/'frames').glob('*.jpg'))
if not frames or len(trace)<2:raise RuntimeError('no actual recorded run')
# Recorder captures every 0.5 simulation seconds. Frame-to-trace alignment is
# bounded by one recording interval; the inset is explanatory, not a metric input.
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
heading=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
canvas=Image.new('RGB',(300,300),'white');ink=ImageDraw.Draw(canvas)
cmd=['ffmpeg','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(2*args.speed),'-i','-',
     '-vf','fps=30','-c:v','libx264','-crf','20','-pix_fmt','yuv420p',str(OUT/f'drawing_{args.speed}x.mp4')]
stride=max(1,args.speed//15)
cmd[cmd.index('-r')+1]=str(2*args.speed/stride)
process=subprocess.Popen(cmd,stdin=subprocess.PIPE)
last=0
try:
    for i,path in enumerate(frames):
        end=min(len(trace)-1,int(round(i*(len(trace)-1)/max(1,len(frames)-1))))
        for j in range(max(1,last+1),end+1):
            a,b=trace[j-1],trace[j]
            if a[4] and b[4] and np.linalg.norm(a[1:4]-b[1:4])<.003:
                coords=[((r[1]+.065)/.13*300,(.565-r[2])/.13*300) for r in [a,b]]
                ink.line(coords,fill=(15,30,65),width=2)
        last=end
        if i%stride:continue
        frame=Image.new('RGB',(1280,720),(235,238,242));frame.paste(Image.open(path).convert('RGB'),(0,0))
        draw=ImageDraw.Draw(frame)
        draw.rectangle((10,10,950,55),fill=(25,30,40));draw.text((22,25),f'UR7e | Isaac Sim + ROS 2 Humble | {args.speed}x simulation time',fill='white',font=heading)
        frame.paste(canvas,(970,120));draw.text((980,80),'MEASURED PEN-TIP TRACE',fill=(20,30,40),font=font)
        draw.text((980,450),f'Simulated time: {trace[end,0]-trace[0,0]:.1f} s',fill=(20,30,40),font=font)
        draw.text((980,480),'Ink: geometric paper contact',fill=(20,30,40),font=font)
        draw.text((980,510),'Not a physical robot recording',fill=(20,30,40),font=font)
        process.stdin.write(frame.tobytes())
    # Hold the final observed state for three video seconds.
    for _ in range(int(3*2*args.speed/stride)):
        process.stdin.write(frame.tobytes())
finally:
    process.stdin.close()
if process.wait()!=0:raise RuntimeError('video encoding failed')
