import cv2
import numpy as np

def preprocess(frame_rgb: np.ndarray, input_shape, input_dtype) -> np.ndarray:
    _, H, W, _ = input_shape
    img = cv2.resize(frame_rgb, (W, H), interpolation=cv2.INTER_LINEAR)

    if input_dtype == np.float32:
        x = img.astype(np.float32)
        # common SSD MobileNet float models: [-1, 1]
        x = (x - 127.5) / 127.5
        return np.expand_dims(x, axis=0)
    else:
        return np.expand_dims(img.astype(np.uint8), axis=0)
