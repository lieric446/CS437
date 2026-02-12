import cv2
from picamera2 import Picamera2
import asyncio
import websockets
import numpy as np
from dotenv import load_dotenv
import os
import json
import struct

load_dotenv()

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (640, 480)},
    controls={"FrameRate": 60}
)
picam2.configure(config)
picam2.start()

async def pi_stream():
    uri = f"ws://{os.getenv('LOCAL_SERVER_IP')}:{os.getenv('LOCAL_SERVER_PORT')}/ws"

    async with websockets.connect(uri) as websocket:
        while True:
            frame = picam2.capture_array()
            _, buffer = cv2.imencode('.jpg', frame)

            # send bytes to server
            await websocket.send(buffer.tobytes())

            # get annotated bytes
            data = await websocket.recv()
            
            # get the header
            header_size = 4
            header = data[:header_size]

            # get the metadata
            metadata_length = struct.unpack('!I', header)[0]
            metadata_bytes = data[header_size:header_size + metadata_length]
            metadata = json.loads(metadata_bytes.decode('utf-8'))

            # get the annotated image bytes
            frame_bytes = data[header_size + metadata_length:]
            nparr = np.frombuffer(frame_bytes, np.uint8)
            annotated_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)


            cv2.imshow('WebSocket Stream', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    picam2.stop()
    cv2.destroyAllWindows()

asyncio.run(pi_stream())
