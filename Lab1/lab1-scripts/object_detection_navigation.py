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
import struct
import json

# load any environment variables from .env file
load_dotenv()

# constants and end conditions
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
OBSTACLES_DETECTED_STOP = 5
SAFE_DISTANCE = 40
FPS = 20
TIME_PER_FRAME = 1 / FPS

# function to handle websocket connection
# connection is used to process one frame
# detect objects in the frame, then decided to move
# left/right depending on the position of objects
# annotated in the frame
async def server_connection(waiting_annotation_queue: queue.Queue, 
                            annotated_queue: queue.Queue):
    uri = f"ws://{os.getenv('LOCAL_SERVER_IP')}:{os.getenv('LOCAL_SERVER_PORT')}/ws"

    while True:
        frame = waiting_annotation_queue.get()
        try:
            async with websockets.connect(uri) as websocket:
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

                # add to queue
                if annotated_queue.full():
                    annotated_queue.get_nowait() # discard oldest frame
                annotated_queue.put({ 'frame': annotated_frame, 'metadata': metadata })
        except Exception as e:
            print(f"ws connection error: {e}")
            await asyncio.sleep(1)

def thread_server_connection(waiting_annotation_queue, annotated_queue):
    asyncio.run(server_connection(waiting_annotation_queue, annotated_queue))

# picam config
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)},
    controls={"FrameRate": FPS}
)
picam2.configure(config)
picam2.start()

# function to stream frame to see what camera sees
def picam_stream(stream_queue: queue.Queue):
    while True:
        frame = picam2.capture_array()
        if stream_queue.full():
            stream_queue.get_nowait() # discard oldest frame
        stream_queue.put(frame)
        time.sleep(TIME_PER_FRAME)


async def main(picam_stream_queue: queue.Queue, 
               waiting_annotation_queue: queue.Queue, 
               annotated_queue: queue.Queue, 
               picar: Picarx):
    obstacles_detected = 0
    # not all frames will be annotated, so we need to keep track of the
    last_annotated_frame_data = None
    while obstacles_detected < OBSTACLES_DETECTED_STOP:
        # get a frame, annotate it, and show it
        frame_to_annotate = picam_stream_queue.get()
        if waiting_annotation_queue.full():
            waiting_annotation_queue.get_nowait() # discard oldest frame
        waiting_annotation_queue.put(frame_to_annotate)

        # get the annotated frame and metadata, and show the frame
        last_annotated_frame_data = annotated_queue.get()
        cv2.imshow("Annotated Frame", last_annotated_frame_data['frame'])

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # if distance is safe, continue straight
        distance = round(picar.ultrasonic.read(), 2)
        if distance > SAFE_DISTANCE or distance < 0:
            picar.set_dir_servo_angle(0)
            picar.forward(25)
        else:
            # if not safe, turn servo based on last annotated frame
            picar.forward(0)
            picar.set_dir_servo_angle(0)
            picar.backward(60)
            time.sleep(1.5)
            picar.forward(0)
            obstacles_detected += 1

            if last_annotated_frame_data['metadata']['annotated']:
                coords = last_annotated_frame_data['metadata']['largest_box_coords']
                x_center = (coords[0] + coords[2]) / 2
                
                if x_center > FRAME_WIDTH / 2:
                    picar.set_dir_servo_angle(-20)
                else:
                    picar.set_dir_servo_angle(20)

                # move forward at angle
                picar.forward(25)
                time.sleep(1.5)
            else:
                # # turn right, read distance
                # picar.set_dir_servo_angle(20)
                # picar.forward(25)
                # time.sleep(1)
                # picar.forward(0)
                # # take five distance readings and average them to reduce noise
                # right_distance = 0
                # for _ in range(5):
                #     right_distance += round(picar.ultrasonic.read(), 2)
                #     time.sleep(0.1)
                # right_distance = round(right_distance / 5, 2)

                # # go back to original position
                # picar.backward(25)
                # time.sleep(1)

                # # turn left, read distance
                # picar.set_dir_servo_angle(-20)
                # picar.forward(25)
                # time.sleep(1)
                # picar.forward(0)
                # # take five distance readings and average them to reduce noise
                # left_distance = 0
                # for _ in range(5):
                #     left_distance += round(picar.ultrasonic.read(), 2)
                #     time.sleep(0.1)
                # left_distance = round(left_distance / 5, 2)

                # # go back to original position
                # picar.backward(25)
                # time.sleep(1)
                # picar.forward(0)
                # picar.set_dir_servo_angle(0)

                # # turn to direction with more space
                # if right_distance > left_distance:
                #     picar.set_dir_servo_angle(20)
                # else:
                #     picar.set_dir_servo_angle(-20)
                # picar.forward(25)
                # time.sleep(1)
                SAFE_DIST = 40
                DANGER_DIST = 20
                POWER = 25
                if distance >= SAFE_DIST:
                    # Path is clear
                    px.set_dir_servo_angle(0)
                    px.forward(POWER)
                elif distance >= DANGER_DIST:
                    # Getting close, turn left to avoid
                    px.set_dir_servo_angle(-30)
                    px.forward(POWER)
                    time.sleep(0.1)
                else:
                    # Too close! Stop and back up
                    px.forward(0)
                    px.set_dir_servo_angle(30)
                    px.backward(POWER)
                    time.sleep(0.5)


if __name__ == "__main__":
    try:
        # start picam stream
        picam_stream_queue = queue.Queue(maxsize=5)
        threading.Thread(target=picam_stream, args=(picam_stream_queue,), daemon=True).start()

        # instantiate queue for annotated frames and start server connection
        waiting_annotation_queue = queue.Queue(maxsize=5)
        annotated_queue = queue.Queue(maxsize=5)
        threading.Thread(target=thread_server_connection, args=(waiting_annotation_queue, annotated_queue), daemon=True).start()

        # picar config
        try:
            px = Picarx()
            px.set_dir_servo_angle(0)
        except Exception as e:
            print(f"Error initializing Picarx: {e}")
            exit(1)
        
        # run main loop
        asyncio.run(main(picam_stream_queue, waiting_annotation_queue, annotated_queue, px))
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        px.forward(0)
        picam2.stop()
        cv2.destroyAllWindows()

