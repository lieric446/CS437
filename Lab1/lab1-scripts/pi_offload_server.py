import cv2
import numpy as np
from fastapi import FastAPI, WebSocket
from ultralytics import YOLO
import uvicorn
import asyncio

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
                # # streaming?
                # results_gen = model.predict(frame, stream=True, verbose=False, conf=0.50)
                # result = next(results_gen)
                # annotated_frame = result.plot()

                # single frames?
                results = model.predict(frame, verbose=False, conf=0.50)
                annotated_frame = results[0].plot()
                
                # send back
                _, buffer = cv2.imencode('.jpg', annotated_frame)
                await websocket.send_bytes(buffer.tobytes())
                
    except Exception as e:
        print(f"Connection closed: {e}")
    finally:
        await websocket.close()

if __name__ == "__main__":
    # run server
    uvicorn.run(app, host="0.0.0.0", port=8000)
