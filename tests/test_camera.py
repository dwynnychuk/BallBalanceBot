from camera import CameraConfig


def test_camera_config_defaults():
    config = CameraConfig()
    
    assert config.buffer_size == 1
