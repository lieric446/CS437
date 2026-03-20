from flask import Flask, Response
from picamera2 import Picamera2
import io
import time

app = Flask(__name__)

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (640, 480)}
)

picam2.configure(config)
picam2.start()
time.sleep(2)


def generate_frames():
    while True:
        stream = io.BytesIO()

        # Capture one JPEG frame into memory
        picam2.capture_file(stream, format="jpeg")

        frame = stream.getvalue()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/")
def index():
    return "Pi camera server is running"


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, threaded=True)
