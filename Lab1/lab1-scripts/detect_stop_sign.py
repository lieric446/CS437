from ultralytics import YOLO
from picamera2 import Picamera2
import cv2

# 33333 microseconds = ~30 FPS
fps_limit = 33333

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)},
    controls={"FrameDurationLimits": (fps_limit, fps_limit)}
)
picam2.configure(config)
picam2.start()

# model from https://www.kaggle.com/models/phillipssempeebwa/yolov8-road-sign-detection
model = YOLO('model_unzipped/best.pt')

while True:
    frame = picam2.capture_array()

    results = model(frame, stream=True, conf=0.75)

    for r in results:
        annotated_frame = r.plot()
    
    cv2.imshow("Pi5 Camera - VNC View", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
