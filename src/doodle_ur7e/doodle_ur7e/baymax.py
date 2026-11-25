"""Optional dialogue/image input. This module never imports ROS or sends motion."""
import base64
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import urllib.error
import urllib.request
import uuid

import numpy as np
from PIL import Image, ImageDraw, ImageOps
from .core import validate_strokes
from .image import sketch, preview

MODEL = 'gpt-5.6-sol'
IMAGE_MODEL = 'gpt-image-2.5-flare'
MAX_IMAGE_BYTES = 20 * 1024 * 1024
STYLE = ('Create one simple black line drawing on a plain white square background for a '
         'single-pen robot. Use a few smooth, clearly separated outlines, generous white '
         'margins, no shading, hatching, color, texture, text, border or filled areas. '
         'Do not draw the robot or a scene of drawing. Depict only the agreed subject.')


def request_api(payload):
    """One explicit API call; no retries, shell tools, or credential persistence."""
    key = os.environ.get('OPENAI_API_KEY')
    if not key:
        raise ValueError('GPT mode requires OPENAI_API_KEY in this terminal; --image works offline')
    request = urllib.request.Request('https://api.openai.com/v1/responses',
        data=json.dumps(dict(payload, store=False)).encode(),
        headers={'Authorization': f'Bearer {key}', 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=300) as response:
            raw = response.read(40 * 1024 * 1024 + 1)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f'OpenAI API HTTP {exc.code}; check model access, key and quota. No motion sent.') from None
    except urllib.error.URLError:
        raise RuntimeError('OpenAI API connection failed. No motion sent.') from None
    if len(raw) > 40 * 1024 * 1024:
        raise ValueError('API response too large')
    data = json.loads(raw)
    if data.get('status') != 'completed':
        raise ValueError('API response incomplete or refused; no motion permitted')
    return data


def chat_reply(messages):
    response = request_api({'model': MODEL, 'instructions':
        'You are Baymax, a friendly robot drawing assistant. Discuss the desired simple '
        'single-pen line drawing in the user language. Ask briefly when unclear. '
        'Do not claim to operate the robot or generate an image in this text-only exchange. '
        'The user uses /generate to see a picture and /confirm to accept that exact picture. '
        'Never treat conversation as authorization to move. '+STYLE,
        'input': messages, 'max_output_tokens': 1500, 'reasoning': {'effort': 'low'}})
    text = '\n'.join(part['text'] for item in response.get('output', [])
        if item.get('type') == 'message' for part in item.get('content', [])
        if part.get('type') == 'output_text')
    if not text.strip():
        raise ValueError('No assistant text returned')
    return text


def generate_image(messages):
    response = request_api({'model': MODEL, 'instructions': STYLE,
        'input': messages + [{'role':'user','content':'Generate the agreed drawing now. '+STYLE}],
        'tools': [{'type':'image_generation','model':IMAGE_MODEL,'size':'1024x1024',
                   'quality':'low','output_format':'png','background':'opaque'}],
        'tool_choice': {'type':'image_generation'}, 'parallel_tool_calls':False})
    images = [item for item in response.get('output', [])
              if item.get('type') == 'image_generation_call' and item.get('status') == 'completed']
    if len(images) != 1 or not images[0].get('result'):
        raise ValueError('Expected exactly one completed generated image')
    try:
        data = base64.b64decode(images[0]['result'], validate=True)
    except (ValueError, TypeError):
        raise ValueError('Invalid generated image encoding') from None
    return data, {'response_id':response.get('id'), 'model':MODEL, 'image_model':IMAGE_MODEL,
                  'revised_prompt':images[0].get('revised_prompt')}


def raster(data):
    if not data or len(data) > MAX_IMAGE_BYTES:
        raise ValueError('image must be nonempty and <=20 MiB')
    with Image.open(io.BytesIO(data)) as image:
        if image.format not in ['PNG','JPEG','WEBP','BMP']:
            raise ValueError('use PNG, JPEG, WebP or BMP')
        if min(image.size) < 16 or max(image.size) > 4096 or image.width*image.height > 16_000_000:
            raise ValueError('image dimensions must be 16..4096 pixels and <=16 megapixels')
        suffix = {'PNG':'.png','JPEG':'.jpg','WEBP':'.webp','BMP':'.bmp'}[image.format]
        rgba = ImageOps.exif_transpose(image).convert('RGBA')
        opaque = Image.new('RGBA',rgba.size,'white'); opaque.alpha_composite(rgba)
        return opaque.convert('RGB'), suffix


def identifier():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]


def archive(data, kind, root, metadata=None):
    image, suffix = raster(data)
    job_id = identifier(); folder = Path(root)/kind/job_id
    folder.mkdir(parents=True, exist_ok=False)
    source = folder/('original'+suffix); source.write_bytes(data)
    image.save(folder/'review.png')
    job = {'id':job_id,'kind':kind,'source':str(source.resolve()),
           'sha256':hashlib.sha256(data).hexdigest(),'metadata':metadata or {},'approved':False}
    (folder/'source.json').write_text(json.dumps(job,ensure_ascii=False,indent=2))
    return job


def approve(job):
    """Called only by explicit human /confirm, never model output."""
    if hashlib.sha256(Path(job['source']).read_bytes()).hexdigest() != job['sha256']:
        raise ValueError('image changed after preview; generate and review again')
    job = dict(job, approved=True, approved_at=datetime.now(timezone.utc).isoformat())
    (Path(job['source']).parent/'source.json').write_text(json.dumps(job,ensure_ascii=False,indent=2))
    return job


def check_strokes(lines, config):
    lines = validate_strokes([np.asarray(line).tolist() for line in lines])
    count = sum(map(len,lines))
    scale = min(config['paper']['width'],config['paper']['height'])-2*config['paper']['margin']
    length = float(sum(np.linalg.norm(np.diff(line,axis=0),axis=1).sum() for line in lines)*scale)
    if len(lines)>120 or count>3000 or length>2.5:
        raise ValueError(f'drawing too complex: {len(lines)} strokes, {count} points, {length:.3f}m ink; '
                         'limits are 120 strokes, 3000 points, 2.5m; simplify the image')
    return {'strokes':len(lines),'points':count,'ink_length_m':length}


def process(job, config, root):
    """Fail closed before ROS; geometric feasibility still requires live MoveIt."""
    if job['kind']=='generated' and job.get('approved') is not True:
        raise ValueError('generated image requires explicit human confirmation')
    data = Path(job['source']).read_bytes()
    if hashlib.sha256(data).hexdigest()!=job['sha256']:
        raise ValueError('source image changed; import/review again')
    out = Path(root)/'processed'/job['id']; out.mkdir(parents=True,exist_ok=False)
    report = dict(source=job,image_checks_passed=False,robot_feasibility='requires_live_planning')
    try:
        image, _ = raster(data)
        gray = np.array(image.convert('L'))
        ratio = float(np.mean(gray<190))
        if not .0002 <= ratio <= .4:
            raise ValueError('image is blank or too densely filled; use sparse black lines on white')
        lines = sketch(np.array(image)[:,:,::-1].copy())
        report.update(check_strokes(lines,config),dark_pixel_fraction=ratio)
        (out/'strokes.json').write_text(json.dumps(lines))
        preview(lines,out/'preview.svg')
        drawing=Image.new('RGB',(1024,1024),'white'); pen=ImageDraw.Draw(drawing)
        for line in lines:
            pen.line([(x*1023,y*1023) for x,y in line],fill='black',width=2)
        drawing.save(out/'processed.png')
        report['image_checks_passed']=True
    except (ValueError,OSError) as exc:
        report['rejection']=str(exc)
        raise
    finally:
        (out/'checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(f'Image checks passed: {out}/processed.png; robot feasibility still requires live planning.')
    return out


def external(path, config, root='images'):
    path=Path(path)
    if path.stat().st_size>MAX_IMAGE_BYTES:
        raise ValueError('image exceeds 20 MiB')
    job=archive(path.read_bytes(),'external',root,{'filename':path.name})
    return process(job,config,root)


def dialogue(config, root='images', read=input, write=print):
    """Text UI: discuss -> generate -> explicitly confirm -> process. No motion here."""
    if not os.environ.get('OPENAI_API_KEY'):
        raise ValueError('Set OPENAI_API_KEY in this terminal for GPT chat; do not paste it into the chat')
    messages=[]; candidate=None
    write('Baymax: 请描述想画的简单图案。/generate 出图；查看图片后 /confirm 确认；/quit 退出。')
    while True:
        try: text=read('你: ').strip()
        except EOFError: return None
        if not text: continue
        if text=='/quit': return None
        if text=='/confirm':
            if candidate is None:
                write('请先 /generate 并查看当前图片。'); continue
            return process(approve(candidate),config,root)
        if text=='/generate':
            if not messages:
                write('请先描述想画的内容。'); continue
            # A failed new generation cannot leave an older image confirmable.
            candidate=None
            data,metadata=generate_image(messages)
            candidate=archive(data,'generated',root,dict(metadata,conversation=messages.copy()))
            write(f"图片已生成，请打开查看: {Path(candidate['source']).parent/'review.png'}")
            write('满意请 /confirm；不满意请继续描述修改，再 /generate。尚未允许机械臂动作。')
            continue
        if len(text)>2000 or sum(len(m['content']) for m in messages)+len(text)>20000:
            raise ValueError('conversation too long; start a new short drawing request')
        if candidate is not None:
            write('已继续讨论，上一张图片的待确认状态取消；修改后请重新 /generate。')
            candidate=None
        messages.append({'role':'user','content':text})
        reply=chat_reply(messages);messages.append({'role':'assistant','content':reply});write('Baymax: '+reply)
