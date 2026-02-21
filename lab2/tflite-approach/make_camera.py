from picamera2 import Picamera2
import time
def make_camera(width: int, height: int):
    pc = Picamera2()
    cfg = pc.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": 30},
    )
    pc.configure(cfg)
    pc.start()
    time.sleep(0.2)
    return pc
