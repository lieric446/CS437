from dotenv import load_dotenv
import cv2
from picamera2 import Picamera2
import websockets
import numpy as np
from picarx import Picarx
import os
import asyncio
import threading
import queue
import time

# load any environment variables from .env file
load_dotenv()

# constants and end conditions
SERVO_ROTATION_END = 180
SAFE_DISTANCE = 30
FPS = 20
TIME_PER_FRAME = 1 / FPS

# function to handle websocket connection
# connection is used to process one frame
# detect objects in the frame, then decided to move
# left/right depending on the position of objects
# annotated in the frame
async def server_connection(waiting_annotation_queue, annotated_queue):
    uri = f"ws://{os.getenv('LOCAL_SERVER_IP')}:{os.getenv('LOCAL_SERVER_PORT')}/ws"

    while True:
        frame = await waiting_annotation_queue.get()
        try:
            async with websockets.connect(uri) as websocket:
                _, buffer = cv2.imencode('.jpg', frame)

                # send bytes to server
                await websocket.send(buffer.tobytes())

                # get annotated bytes
                data = await websocket.recv()
                nparr = np.frombuffer(data, np.uint8)
                annotated = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if annotated_queue.full():
                    annotated_queue.get_nowait() # discard oldest frame
                annotated_queue.put(annotated)
        except Exception as e:
            print(f"ws connection error: {e}")
            await asyncio.sleep(1)

def thread_server_connection(waiting_annotation_queue, annotated_queue):
    asyncio.run(server_connection(waiting_annotation_queue, annotated_queue))

# picam config
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (640, 480)},
    controls={"FrameRate": 20}
)
picam2.configure(config)
picam2.start()

# function to stream frame to see what camera sees
def picam_stream(stream_queue):
    while True:
        frame = picam2.capture_array()
        if stream_queue.full():
            stream_queue.get_nowait() # discard oldest frame
        stream_queue.put(frame)
        time.sleep(TIME_PER_FRAME)

if __name__ == "__main__":
    try:
        # start picam stream
        picam_stream_queue = queue.Queue(maxsize=5)
        threading.Thread(target=picam_stream, args=(picam_stream_queue,), daemon=True).start()

        # instantiate queue for annotated frames and start server connection
        waiting_annotation = queue.Queue(maxsize=5)
        annotated_queue = queue.Queue(maxsize=5)
        threading.Thread(target=thread_server_connection, args=(waiting_annotation, annotated_queue), daemon=True).start()

        # picar config
        try:
            px = Picarx()
        except Exception as e:
            print(f"Error initializing Picarx: {e}")
            exit(1)
        
        curr_servo_rotation = 0
        while curr_servo_rotation < SERVO_ROTATION_END:
            if not picam_stream_queue.empty():
                frame = picam_stream_queue.get()
                cv2.imshow("Pi5 Camera - VNC View", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            distance = round(px.ultrasonic.read(), 2)
            if distance > SAFE_DISTANCE or distance < 0:
                px.forward(25)
            else:
                px.forward(0)

                frame_to_annotate = picam_stream_queue.get()
                if waiting_annotation.full():
                    waiting_annotation.get_nowait() # discard oldest frame
                waiting_annotation.put(frame_to_annotate)

                
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

