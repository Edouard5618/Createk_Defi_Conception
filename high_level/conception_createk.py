import cv2
import numpy as np
import serial
import time
import sys
import traceback
from ultralytics import YOLO

# ----------------------------
# Configuration
# ----------------------------
MODEL_PATH = "yolov8n.pt"  # pretrained YOLOv8 (COCO)
SERIAL_PORT = "COM6"        # e.g. COM6 on Windows, /dev/ttyACM0 on Linux
BAUDRATE = 115200

H_FOV_DEG = 70.0   # horizontal field of view of camera
V_FOV_DEG = 45.0   # vertical field of view of camera

CONFIDENCE_THRESHOLD = 0.4  # minimum detection confidence
MAX_WATCHDOG_FAILURES = 3   # number of consecutive failures before restart
ARDUINO_RECONNECT_INTERVAL = 5.0  # seconds between reconnection attempts

# ----------------------------
# Watchdog restart mechanism
# ----------------------------


def restart_program():
    """Restart the current program."""
    print("\n[WATCHDOG] Restarting program...")
    time.sleep(2)
    python = sys.executable
    import os
    os.execl(python, python, *sys.argv)

# ----------------------------
# Main function with error handling
# ----------------------------


def main():
    print("="*60)
    print("  Person Angle Tracker - Starting Up")
    print("="*60)

    # ----------------------------
    # Serial connection (with fallback)
    # ----------------------------
    ser = None
    try:
        print(f"[1/3] Attempting to connect to Arduino on {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # allow Arduino reset
        print(f"      ✓ Arduino connected successfully on {SERIAL_PORT}")
    except Exception as e:
        print(f"      ⚠ Warning: Could not connect to Arduino: {e}")
        print(f"      → Continuing without Arduino (angles will not be sent)")
        ser = None

    # ----------------------------
    # Load YOLO model
    # ----------------------------
    print(f"[2/3] Loading YOLO model from {MODEL_PATH}...")
    model = YOLO(MODEL_PATH)
    print(f"      ✓ Model loaded successfully")

    # ----------------------------
    # Initialize webcam
    # ----------------------------
    print(f"[3/3] Initializing webcam (camera index 1)...")
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("      ✗ ERROR: Cannot open webcam")
        if ser:
            ser.close()
        raise RuntimeError("Cannot open webcam")
    print(f"      ✓ Webcam initialized successfully")

    print("="*60)
    print("  Setup Complete - Starting Main Loop")
    print("  Press 'q' to quit")
    print("="*60)

    # ----------------------------
    # Main loop with watchdog
    # ----------------------------
    watchdog_failures = 0
    frame_count = 0
    last_arduino_check = time.time()

    try:
        while True:
            frame_count += 1

            # ----------------------------
            # Periodic Arduino connection check
            # ----------------------------
            current_time = time.time()
            if current_time - last_arduino_check >= ARDUINO_RECONNECT_INTERVAL:
                last_arduino_check = current_time

                # If not connected, try to connect
                if ser is None:
                    try:
                        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
                        time.sleep(0.5)  # brief settle time
                        print(f"[ARDUINO] Successfully connected to {SERIAL_PORT}")
                    except Exception as e:
                        pass  # Stay silent about failed reconnection attempts

                # If connected, verify it's still alive
                elif ser is not None:
                    try:
                        # Check if serial port is still open and responsive
                        if not ser.is_open:
                            raise Exception("Serial port closed")
                    except Exception as e:
                        print(f"[ARDUINO] Connection lost: {e}")
                        try:
                            ser.close()
                        except:
                            pass
                        ser = None

            try:
                ret, frame = cap.read()
                if not ret:
                    print(f"[WARNING] Failed to grab frame {frame_count}")
                    watchdog_failures += 1
                    if watchdog_failures >= MAX_WATCHDOG_FAILURES:
                        raise RuntimeError(f"Failed to grab {MAX_WATCHDOG_FAILURES} consecutive frames")
                    time.sleep(0.1)
                    continue

                # Reset watchdog on successful frame capture
                watchdog_failures = 0

                h, w, _ = frame.shape
                cx_img, cy_img = w // 2, h // 2

                # ----------------------------
                # Run detection
                # ----------------------------
                results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)

                # Find the first detected person
                person_box = None
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        if model.names[cls_id] == "person":
                            person_box = box
                            break
                    if person_box is not None:
                        break

                if person_box is not None:
                    # ----------------------------
                    # Bounding box center
                    # ----------------------------
                    x1, y1, x2, y2 = person_box.xyxy[0].cpu().numpy()
                    cx_p = int((x1 + x2) / 2)
                    cy_p = int((y1 + y2) / 2)

                    # ----------------------------
                    # Angle computation
                    # ----------------------------
                    dx = cx_p - cx_img
                    dy = cy_img - cy_p  # inverted because image Y axis goes down

                    yaw = (dx / w) * H_FOV_DEG
                    pitch = (dy / h) * V_FOV_DEG

                    # ----------------------------
                    # Send to Arduino (if connected)
                    # ----------------------------
                    if ser:
                        try:
                            msg = f"{yaw:.2f},{pitch:.2f}\n"
                            ser.write(msg.encode())
                        except Exception as e:
                            print(f"[ARDUINO] Write failed: {e}")
                            # Close the broken connection so it can be re-established
                            try:
                                ser.close()
                            except:
                                pass
                            ser = None
                            print(f"[ARDUINO] Connection reset, will attempt reconnection")

                    # ----------------------------
                    # Visualization
                    # ----------------------------
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(frame, (cx_img, cy_img), 5, (255, 0, 0), -1)  # image center
                    cv2.circle(frame, (cx_p, cy_p), 5, (0, 0, 255), -1)      # person center
                    cv2.line(frame, (cx_img, cy_img), (cx_p, cy_p), (0, 255, 0), 2)
                    text = f"Yaw: {yaw:.1f} deg | Pitch: {pitch:.1f} deg"
                    cv2.putText(frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                else:
                    cv2.putText(frame, "No person detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

                # Add Arduino connection status to display
                arduino_status = "Arduino: Connected" if ser else "Arduino: Disconnected"
                cv2.putText(frame, arduino_status, (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                cv2.imshow("Person Angle Tracker", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n[INFO] User requested quit (pressed 'q')")
                    break

            except Exception as inner_e:
                print(f"[ERROR] Error in frame processing (frame {frame_count}): {inner_e}")
                watchdog_failures += 1
                if watchdog_failures >= MAX_WATCHDOG_FAILURES:
                    raise RuntimeError(f"Too many consecutive errors ({MAX_WATCHDOG_FAILURES})")
                time.sleep(0.1)
                continue

    finally:
        print("\n[CLEANUP] Releasing resources...")
        cap.release()
        cv2.destroyAllWindows()
        if ser:
            ser.close()
        print("[CLEANUP] Complete")


# ----------------------------
# Entry point with watchdog restart
# ----------------------------
if __name__ == "__main__":
    retry_count = 0
    max_retries = 5

    while retry_count < max_retries:
        try:
            main()
            # Normal exit - don't retry
            print("[EXIT] Program ended normally")
            break

        except KeyboardInterrupt:
            print("\n[EXIT] Program interrupted by user (Ctrl+C)")
            break

        except Exception as e:
            retry_count += 1
            print(f"\n[WATCHDOG] Critical error occurred (attempt {retry_count}/{max_retries}):")
            print(f"  Error: {e}")
            traceback.print_exc()

            if retry_count < max_retries:
                print(f"[WATCHDOG] Restarting in 3 seconds...")
                time.sleep(3)
            else:
                print(f"[WATCHDOG] Max retries ({max_retries}) reached. Exiting.")
                break
