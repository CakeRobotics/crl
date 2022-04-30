import pytest
from cake import Robot

def test_exceptions_from_backend_loop():
    robot = Robot({})
    assert robot.health()
    @robot.runtime.run_in_event_loop
    async def body():
        raise Exception("test-exception")
    with pytest.raises(Exception, match='test-exception'):
        body()
    robot.shutdown()
