import cv2
import numpy as np
from fastapi import FastAPI, WebSocket
from ultralytics import YOLO
import uvicorn
import json
import struct

# use this to offload to gpu if you have one available and cuda installed
app = FastAPI()

# load model to GPU
model = YOLO("yolo11l.pt")
model.to('cuda')

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Client connected")
    
    try:
        while True:
            # get bytes from client
            data = await websocket.receive_bytes()
            
            # decode image
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                results_gen = model.predict(frame, stream=True, verbose=False, conf=0.50)
                result = next(results_gen)
                annotated_frame = frame

                # flags
                annotated = False
                largest_box_coords = None

                # check if any boxes were detected
                if result.boxes:
                    annotated = True
                    annotated_frame = result.plot()
                    # compare area of boxes to get the largest
                    largest_box = max(result.boxes, key=lambda box: (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1]))

                    # get the coordinates of the largest box
                    largest_box_coords = largest_box.xyxy[0].tolist()
                
                # bytes to send back
                _, buffer = cv2.imencode('.jpg', annotated_frame)
                metadata = {
                    "annotated": annotated,
                    "largest_box_coords": largest_box_coords
                }

                # convert all to bytes and send back
                metadata_bytes = json.dumps(metadata).encode('utf-8')
                header = struct.pack("!I", len(metadata_bytes))
                await websocket.send_bytes(header + metadata_bytes + buffer.tobytes())
    except Exception as e:
        print(f"Connection closed: {e}")

if __name__ == "__main__":
    # run server
    uvicorn.run(app, host="0.0.0.0", port=8000)
