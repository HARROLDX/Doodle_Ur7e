from concurrent.futures import Future
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from doodle_ur7e.ros import Robot


@pytest.mark.parametrize('name,attempts', [('/controller_manager/list_controllers', 2),
                                          ('/get_planning_scene', 2),
                                          ('/apply_planning_scene', 1),
                                          ('/compute_cartesian_path', 1)])
def test_only_read_status_is_retried(name, attempts):
    requests, removed, warnings = [], [], []
    def request(_):
        f = Future(); requests.append(f); return f
    client = SimpleNamespace(wait_for_service=lambda **_: True, call_async=request,
                             remove_pending_request=removed.append)
    def result(future, timeout):
        if len(requests) == 1:
            raise TimeoutError('injected lost response')
        return 'response'
    node = SimpleNamespace(service_clients={name: client}, result=result,
                           get_logger=lambda: SimpleNamespace(warning=warnings.append))
    if attempts == 2:
        assert Robot.call(node, object, name, object()) == 'response'
        assert len(warnings) == 1
    else:
        with pytest.raises(TimeoutError, match=name):
            Robot.call(node, object, name, object())
    assert len(requests) == attempts
    assert removed == [requests[0]]
    assert requests[0].cancelled()


def test_read_status_retries_are_bounded():
    requests = []
    def request(_):
        f = Future(); requests.append(f); return f
    def timeout(*_):
        raise TimeoutError('injected persistent timeout')
    client = SimpleNamespace(wait_for_service=lambda **_: True, call_async=request,
                             remove_pending_request=lambda _: None)
    node = SimpleNamespace(service_clients={'/controller_manager/list_controllers': client},
                           result=timeout, get_logger=lambda: SimpleNamespace(warning=lambda _: None))
    with pytest.raises(TimeoutError, match='2 attempt'):
        Robot.call(node, object, '/controller_manager/list_controllers', object())
    assert len(requests) == 2 and all(f.cancelled() for f in requests)
