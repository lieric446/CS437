import cv2
from picamera2 import Picamera2
import asyncio
import websockets
import numpy as np

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (640, 480)},
    controls={"FrameRate": 60}
)
picam2.configure(config)
picam2.start()

async def pi_stream():
    uri = "ws://COMPUTER_IP:8000/ws"

    async with websockets.connect(uri) as websocket:
        while True:
            frame = picam2.capture_array()
            _, buffer = cv2.imencode('.jpg', frame)

            # send bytes to server
            await websocket.send(buffer.tobytes())

            # get annotated bytes
            data = await websocket.recv()
            nparr = np.frombuffer(data, np.uint8)
            annotated = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            cv2.imshow('WebSocket Stream', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    picam2.stop()
    cv2.destroyAllWindows()

asyncio.run(pi_stream())
