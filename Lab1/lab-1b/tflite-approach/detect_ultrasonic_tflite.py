import argparse
import os
from load_labels import load_labels
from preprocess import preprocess
from make_camera import make_camera
from tflite_runtime.interpreter import Interpreter
from picarx import Picarx
from parse_ssd import parse_ssd
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="Path to .tflite model")
    ap.add_argument("--labels", required=True, help="Path to labels.txt")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--topk", type=int, default=5)
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=480)
    ap.add_argument("--print-every", type=float, default=1.0)
    ap.add_argument("--use-ultrasonic", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        raise FileNotFoundError(f"Model not found: {args.model}")
    if not os.path.exists(args.labels):
        raise FileNotFoundError(f"Labels not found: {args.labels}")
    
    labels = load_labels(args.labels)
    interpreter = None
    interpreter = Interpreter(model_path=args.model, num_threads=2)
    interpreter.allocate_tensors()
    in0 = interpreter.get_input_details()[0]
    input_shape = tuple(in0["shape"])
    input_dtype = in0["dtype"]

    print(f"Model: {args.model}")
    print(f"Labels: {args.labels} ({len(labels)} labels)")
    print(f"Input: shape={input_shape} dtype={input_dtype}")

    cam = make_camera(args.cam_width, args.cam_height)
    pc = Picarx()
    last = 0.0

    try:
        while True:
            frame_rgb = cam.capture_array()
            frame_rgb = preprocess(frame_rgb, input_shape, input_dtype)
            interpreter.set_tensor(in0["index"], frame_rgb)
            interpreter.invoke()
            # print("Ultrasonic (cm):", pc.ultrasonic.read())
            dets = parse_ssd(interpreter, labels, args.threshold, args.topk)

            now = time.time()

            if now - last >= args.print_every:
                last = now
                print("Detections:", dets)
                d = pc.ultrasonic.read()
                print("Distance (cm):", d)

                if dets:
                    for name, score in dets:
                        print(f"  {name}: {score:.2f}")
                
                if d < 20:
                    print("Object too close! Stopping.")
                    pc.stop()
                else:
                    print("Path is clear. Moving forward.")
                    pc.forward(2)  # move forward at speed 20 (out of 100)

    except KeyboardInterrupt:
        print("\nStopping...")
        pc.stop()
    finally:
        try:
            cam.stop()
            pc.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()
