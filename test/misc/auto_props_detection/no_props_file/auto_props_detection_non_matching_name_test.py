import cake
import pytest

def test_props_detection_non_matching_name():
    # File name must be main.py or test_... for auto props file detection to work.
    with pytest.raises(
        Exception,
        match=r"Can't detect project directory. Make sure the robot code is inside a file named `main.py`."
    ):
        _robot = cake.Robot()
