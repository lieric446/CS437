import socket
import json
import threading
import time
import os

from picarx import Picarx

HOST = "0.0.0.0"
PORT = 65432

px = Picarx()

current_power = 30
current_steering_angle = 0


def safe_read_grayscale():
    try:
        # Usually returns a list like [left, middle, right]
        values = px.get_grayscale_data()
        return values
    except Exception as e:
        return {"error": f"grayscale read failed: {str(e)}"}


def safe_read_distance():
    try:
        distance = px.ultrasonic.read()
        return distance
    except Exception as e:
        return {"error": f"distance read failed: {str(e)}"}


def get_status():
    return {
        "power": current_power,
        "steering_angle": current_steering_angle,
        "grayscale": safe_read_grayscale(),
        "distance": safe_read_distance(),
        "timestamp": time.time(),
        "temperature": os.popen("vcgencmd measure_temp").readline().strip()
    }


def move_forward():
    px.set_power(current_power)
    px.forward(current_power)


def move_backward():
    px.set_power(current_power)
    px.backward(current_power)


def move_stop():
    px.stop()


def turn_left(angle=20):
    global current_steering_angle
    current_steering_angle = current_steering_angle - abs(angle)
    px.set_dir_servo_angle(current_steering_angle)


def turn_right(angle=20):
    global current_steering_angle
    current_steering_angle = current_steering_angle + abs(angle)
    px.set_dir_servo_angle(current_steering_angle)


def center_wheels():
    global current_steering_angle
    current_steering_angle = 0
    px.set_dir_servo_angle(0)


def handle_command(message):
    global current_power

    command = message.get("command")
    value = message.get("value")

    if command == "get_status":
        return {"ok": True, "type": "status", "data": get_status()}

    elif command == "forward":
        move_forward()
        return {"ok": True, "message": "moving forward","data": get_status()}

    elif command == "backward":
        move_backward()
        return {"ok": True, "message": "moving backward","data": get_status()}

    elif command == "stop":
        move_stop()
        return {"ok": True, "message": "stopped","data": get_status()}

    elif command == "left":
        angle = int(value) if value is not None else 20
        turn_left(angle)
        return {"ok": True, "message": f"turned left {angle}","data": get_status()}

    elif command == "right":
        angle = int(value) if value is not None else 20
        turn_right(angle)
        return {"ok": True, "message": f"turned right {angle}","data": get_status()}

    elif command == "center":
        center_wheels()
        return {"ok": True, "message": "wheels centered","data": get_status()}

    elif command == "set_power":
        if value is None:
            return {"ok": False, "error": "missing value for set_power"}

        current_power = int(value)
        return {"ok": True, "message": f"power set to {current_power}","data": get_status()}

    else:
        return {"ok": False, "error": f"unknown command: {command}","data": get_status()}


def recv_json_line(conn):

    buffer = b""
    while True:
        chunk = conn.recv(1024)
        print(f"Received chunk: {chunk}")
        if not chunk:
            return None
        buffer += chunk
        if b"\n" in buffer:
            line, _rest = buffer.split(b"\n", 1)
            decoded = line.decode("utf-8").strip()
            print(f"Received line: {decoded}")
            try:
                return json.loads(decoded)
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
                raise Exception(f"Invalid JSON: {decoded}")
            # return json.loads(line.decode("utf-8"))


def send_json(conn, payload):
    message = json.dumps(payload) + "\n"
    conn.sendall(message.encode("utf-8"))


def client_thread(conn, addr):
    print(f"Client connected: {addr}")
    try:
        while True:
            print(f"Waiting for message from {addr}...")
            message = recv_json_line(conn)
            print(f"Received from {addr}: {message}")
            if message is None:
                break

            response = handle_command(message)
            send_json(conn, response)

    except Exception as e:
        print(e)
        try:
            send_json(conn, {"ok": False, "error": str(e)})
        except Exception:
            pass
        print(f"Client error {addr}: {e}")
    finally:
        try:
            px.stop()
        except Exception:
            pass
        conn.close()
        print(f"Client disconnected: {addr}")


def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)

    print(f"Control server listening on {HOST}:{PORT}")

    try:
        while True:
            conn, addr = server.accept()
            print(f"Accepted connection from {addr}")
            thread = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
            thread.start()
    finally:
        px.stop()
        server.close()


if __name__ == "__main__":
    start_server()
