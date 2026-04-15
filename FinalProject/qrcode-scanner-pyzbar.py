from picamera2 import Picamera2
from pyzbar.pyzbar import decode
import cv2
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

time.sleep(2)

last_data = None

print("Scanning... Press q to quit")

while True:
    frame = picam2.capture_array()

    # Convert to grayscale for better decoding
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    results = decode(gray)

    for result in results:
        data = result.data.decode("utf-8")

        x, y, w, h = result.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            frame,
            data,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        if data != last_data:
            print("QR Found:", data)
            last_data = data

    cv2.imshow("QR Scanner", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()

# do sudo apt install libzbar0 python3-pip
