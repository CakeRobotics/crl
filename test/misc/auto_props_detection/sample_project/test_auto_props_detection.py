import cake

def test_auto_props_detection():
    robot = cake.Robot()
    assert robot.props['name'] == "test-project"
    robot.shutdown()
