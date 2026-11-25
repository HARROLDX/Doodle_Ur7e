import base64
import io
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
import pytest
from doodle_ur7e import baymax, cli
from doodle_ur7e.core import load_config

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def config(): return load_config(ROOT/'config/lab.yaml')

def picture():
    image=Image.new('RGB',(400,400),'white');d=ImageDraw.Draw(image)
    d.ellipse((50,50,350,350),outline='black',width=3)
    d.ellipse((130,140,150,160),outline='black',width=3)
    d.ellipse((250,140,270,160),outline='black',width=3)
    d.arc((110,170,290,300),0,180,fill='black',width=3)
    b=io.BytesIO();image.save(b,format='PNG');return b.getvalue()

def test_external_has_no_api_dependency(tmp_path,config,monkeypatch):
    monkeypatch.delenv('OPENAI_API_KEY',raising=False)
    monkeypatch.setattr(baymax,'request_api',lambda p:pytest.fail('external path called API'))
    path=tmp_path/'input.png';path.write_bytes(picture())
    out=baymax.external(path,config,tmp_path/'images')
    report=json.loads((out/'checks.json').read_text())
    assert report['image_checks_passed'] and report['robot_feasibility']=='requires_live_planning'
    assert (out/'processed.png').exists() and (out/'preview.svg').exists()
    assert Path(report['source']['source']).read_bytes()==picture()

def test_confirmation_is_required_and_bound_to_image(tmp_path,config):
    job=baymax.archive(picture(),'generated',tmp_path)
    with pytest.raises(ValueError,match='confirmation'):baymax.process(job,config,tmp_path)
    job=baymax.approve(job)
    Path(job['source']).write_bytes(b'changed')
    with pytest.raises(ValueError,match='changed'):baymax.process(job,config,tmp_path)

@pytest.mark.parametrize('color',['white','black'])
def test_blank_and_filled_images_rejected(tmp_path,config,color):
    b=io.BytesIO();Image.new('RGB',(100,100),color).save(b,format='PNG')
    job=baymax.archive(b.getvalue(),'external',tmp_path)
    with pytest.raises(ValueError,match='blank|densely'):baymax.process(job,config,tmp_path)
    report=json.loads((tmp_path/'processed'/job['id']/'checks.json').read_text())
    assert not report['image_checks_passed']

def test_complexity_and_invalid_coordinates_rejected(config):
    with pytest.raises(ValueError,match='complex'):
        baymax.check_strokes([[[0,0],[1,1]]]*121,config)
    with pytest.raises(ValueError):baymax.check_strokes([[[0,0],[float('nan'),1]]],config)
    with pytest.raises(ValueError):baymax.check_strokes([[[0,0],[2,1]]],config)


def test_dialogue_confirmation_and_revision(tmp_path,config,monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY','test-only-not-real')
    # Assistant text that looks like a command cannot confirm or move anything.
    monkeypatch.setattr(baymax,'chat_reply',lambda messages:'/confirm 已执行（测试不可信模型文本）')
    monkeypatch.setattr(baymax,'generate_image',lambda messages:(picture(),{'test':True}))
    inputs=iter(['画笑脸','/confirm','/generate','换成小笑脸','/confirm','/generate','/confirm'])
    output=[]
    out=baymax.dialogue(config,tmp_path,read=lambda _:next(inputs),write=output.append)
    jobs=[json.loads(p.read_text()) for p in (tmp_path/'generated').glob('*/source.json')]
    assert len(jobs)==2 and sum(j['approved'] for j in jobs)==1
    assert json.loads((out/'checks.json').read_text())['source']['approved']
    assert sum('请先 /generate' in s for s in output)==2


def test_dialogue_quit_never_processes(tmp_path,config,monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY','test-only-not-real')
    monkeypatch.setattr(baymax,'chat_reply',lambda messages:'好')
    monkeypatch.setattr(baymax,'generate_image',lambda messages:(picture(),{}))
    inputs=iter(['笑脸','/generate','/quit'])
    assert baymax.dialogue(config,tmp_path,read=lambda _:next(inputs),write=lambda _:None) is None
    assert not (tmp_path/'processed').exists()


def test_generation_request_and_missing_image(monkeypatch):
    seen=[]
    def api(payload):
        seen.append(payload)
        return {'id':'test','output':[{'type':'image_generation_call','status':'completed',
                                    'result':base64.b64encode(picture()).decode()}]}
    monkeypatch.setattr(baymax,'request_api',api)
    data,meta=baymax.generate_image([{'role':'user','content':'笑脸'}])
    assert data==picture() and meta['model']=='gpt-5.6-sol'
    assert seen[0]['tools'][0]['type']=='image_generation'
    assert seen[0]['tool_choice']=={'type':'image_generation'}
    monkeypatch.setattr(baymax,'request_api',lambda p:{'output':[]})
    with pytest.raises(ValueError,match='exactly one'):baymax.generate_image([])


def test_cli_rejects_execution_before_generation(config,monkeypatch,tmp_path):
    monkeypatch.setattr(baymax,'dialogue',lambda *a:pytest.fail('generation started without execution flags'))
    with pytest.raises(SystemExit) as exc:
        cli.run(['--config',str(ROOT/'config/lab.yaml'),'baymax','--chat','--execute'])
    assert exc.value.code==1


def test_cli_offline_does_not_import_ros(tmp_path,monkeypatch):
    path=tmp_path/'image.png';path.write_bytes(picture())
    import builtins
    original=builtins.__import__
    def guarded(name,*args,**kwargs):
        if name=='rclpy':pytest.fail('offline image processing imported ROS')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    cli.run(['--config',str(ROOT/'config/lab.yaml'),'baymax','--image',str(path),'--images-dir',str(tmp_path/'images')])


def test_api_transport_and_incomplete_response(monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY','test-key')
    seen=[]
    class Response:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def read(self,n):return json.dumps({'status':'completed','output':[]}).encode()
    def send(request,timeout):
        seen.append(request)
        assert timeout==300
        return Response()
    monkeypatch.setattr(baymax.urllib.request,'urlopen',send)
    baymax.request_api({'model':baymax.MODEL,'input':'test'})
    assert seen[0].full_url=='https://api.openai.com/v1/responses'
    payload=json.loads(seen[0].data)
    assert payload['store'] is False and 'test-key' not in seen[0].data.decode()
    monkeypatch.setattr(Response,'read',lambda self,n:b'{"status":"incomplete"}')
    with pytest.raises(ValueError,match='incomplete'):baymax.request_api({})


def test_missing_api_key_fails_without_network(monkeypatch):
    monkeypatch.delenv('OPENAI_API_KEY',raising=False)
    monkeypatch.setattr(baymax.urllib.request,'urlopen',lambda *a,**k:pytest.fail('network called'))
    with pytest.raises(ValueError,match='OPENAI_API_KEY'):baymax.request_api({})
