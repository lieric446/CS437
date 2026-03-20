from bluedot.btcomm import BluetoothServer
from signal import pause
from bluetooth_control import *

def received_handler(data):
    print("Received from phone:", repr(data))
    if data.startswith("ping"):
        s.send("pong")
    elif data.startswith("forward"):
        s.send("Moving forward")
        move_forward()
    elif data.startswith("backward"):
        s.send("Moving backward")
        move_backward()
    elif data.startswith("stop"):
        s.send("Stopping")
        move_stop()
    elif data.startswith("left"):
        s.send("Turning left")
        turn_left()
    elif data.startswith("right"):  
        s.send("Turning right")
        turn_right()

    # s.send("ACK:" + data)

s = BluetoothServer(received_handler)

print("Bluetooth server running...")
pause()

