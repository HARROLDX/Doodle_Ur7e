"""CLI/preflight checks: no driver, scene write or trajectory requests."""
import os
from pathlib import Path
import subprocess
import sys
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def invoke(script, *args, env=None):
    return subprocess.run([sys.executable, str(ROOT/'scripts'/script), *args],
                          cwd=ROOT, env=env, capture_output=True, text=True, timeout=15)


@pytest.mark.parametrize('script', ['check_markers.py', 'save_drawing_start.py',
                                    'return_drawing_start.py', 'lift_clearance_check.py',
                                    'record_trial.py'])
def test_script_help_needs_no_robot(script):
    result = invoke(script, '--help')
    assert result.returncode == 0, result.stderr
    assert 'usage:' in result.stdout


def test_save_start_refuses_overwrite_before_ros(tmp_path):
    saved = tmp_path/'start.json'
    saved.write_text('{"keep": true}')
    result = invoke('save_drawing_start.py', '--output', str(saved))
    assert result.returncode != 0
    assert 'already exists' in result.stderr
    assert saved.read_text() == '{"keep": true}'


def test_return_missing_input_fails_before_ros(tmp_path):
    result = invoke('return_drawing_start.py', '--config', str(ROOT/'config/lab.yaml'),
                    '--start', str(tmp_path/'missing.json'))
    assert result.returncode != 0 and 'missing.json' in result.stderr


@pytest.mark.parametrize('script', ['save_drawing_start.py', 'return_drawing_start.py'])
def test_hardware_helpers_reject_verified_synthetic_paper(tmp_path, script):
    config = yaml.safe_load((ROOT/'config/lab.yaml').read_text())
    config['verified'] = {k: True for k in config['verified']}
    config_path = tmp_path/'config.yaml'
    config_path.write_text(yaml.safe_dump(config))
    paper = tmp_path/'paper.yaml'
    paper.write_text('verified: true\nsynthetic: true\n')
    args = ['--config', str(config_path), '--paper', str(paper)]
    if script == 'return_drawing_start.py':
        saved = tmp_path/'start.json'
        saved.write_text('{}')
        args += ['--start', str(saved)]
    else:
        args += ['--output', str(tmp_path/'new-start.json')]
    result = invoke(script, *args)
    assert result.returncode != 0 and 'synthetic/simulation paper' in result.stderr


def test_recorder_invalid_duration_creates_nothing(tmp_path):
    output = tmp_path/'record'
    result = invoke('record_trial.py', '--output', str(output), '--duration', '-1')
    assert result.returncode != 0 and not output.exists()


def test_wrapper_missing_inputs_stops_before_speed_service(tmp_path):
    if not Path('/opt/ros/humble/setup.bash').is_file():
        pytest.skip('wrapper requires Humble environment setup')
    env = dict(os.environ, CONFIG_FILE=str(tmp_path/'missing.yaml'),
               ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='231')
    result = subprocess.run(['bash', str(ROOT/'scripts/run_smiley_from_start.sh')],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert 'Missing local input' in result.stderr
    assert 'requester:' not in result.stdout
