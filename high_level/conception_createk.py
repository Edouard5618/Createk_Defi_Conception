import cv2
import numpy as np
import serial
import time
import sys
import traceback
import pygame
from ultralytics import YOLO

# ----------------------------
# Configuration
# ----------------------------
# pretrained YOLOv8 (COCO)
MODEL_PATH = r"C:\Users\Edouard\Documents\Documents\Personnel\Projets_Perso_Studio\Createk_Defi_Conception\runs\detect\train8\weights\best.pt"
SERIAL_PORT = "COM8"
BAUDRATE = 115200

H_FOV_DEG = 70.0   # horizontal field of view of camera
V_FOV_DEG = 45.0   # vertical field of view of camera

CONFIDENCE_THRESHOLD = 0.7  # minimum detection confidence
MAX_WATCHDOG_FAILURES = 3   # number of consecutive failures before restart
ARDUINO_RECONNECT_INTERVAL = 5.0  # seconds between reconnection attempts
ARDUINO_MSG_DELAY = 0.02

SERVO_TRIGGER_VALUE = 95
SERVO_TRIGGER_DURATION = 0.5
RELAY_1_DURATION = 10.0  # seconds

X_SPEED_FACTOR = 70
Y_SPEED_FACTOR = 70

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


def send_arduino_message(ser, last_arduino_msg, x_speed, y_speed, servo_pos=0, r1=0, r2=0, send_anyway=False):
    if (ser and (time.time() - last_arduino_msg >= ARDUINO_MSG_DELAY)) or send_anyway:
        last_arduino_msg = time.time()
        try:
            msg = f"X{x_speed} Y{y_speed} S{servo_pos} R1:{r1} R2:{r2}\n"
            ser.write(msg.encode("ascii"))
            print(f"Sent: {msg.strip()}")
        except Exception as e:
            print(f"[ARDUINO] Write failed: {e}")
            # Close the broken connection so it can be re-established
            try:
                ser.close()
            except:
                pass
            ser = None
            print(f"[ARDUINO] Connection reset, will attempt reconnection")

        ser.flushOutput()
        ser.flushInput()

    return ser, last_arduino_msg


def _read_arduino_serial(ser):
    if ser is None:
        return
    try:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"[ARDUINO] {line}")
    except Exception as e:
        print(f"[ARDUINO] Read failed: {e}")
        try:
            ser.close()
        except:
            pass
        return None

    return ser


def _init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[JOYSTICK] No joystick detected - continuing without joystick")
        return None

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[JOYSTICK] Connected: {js.get_name()}")
    return js


def _read_joystick_controls(js, a_button_state):
    """Read joystick input and return button A press state.

    Args:
        js: Joystick object or None
        a_button_state: Current state of button A (False=not pressed, True=pressed)

    Returns:
        a_button_state: Updated button A state
    """
    if js is None:
        return a_button_state

    pygame.event.pump()

    # A button triggers relay 1 (only on transition from not pressed to pressed)
    a_currently_pressed = js.get_button(0)
    if a_currently_pressed and not a_button_state:
        a_button_state = True

    return a_button_state


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
    # Joystick setup
    js = _init_joystick()

    # State variables for relay 1 and servo sequence
    a_button_pressed = False
    r1_active = False
    r1_start_time = None
    servo_activation_pending = False
    servo_pos = 0
    servo_start_time = None

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
    last_arduino_msg = 0.0

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

            # Read any incoming serial messages from Arduino
            ser_read = _read_arduino_serial(ser)
            if ser_read is None:
                ser = None
            else:
                ser = ser_read

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
                        if model.names[cls_id] == "Chevreuil":
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
                    # Relay 2: activate on detection
                    # ----------------------------
                    r2 = 1

                    # ----------------------------
                    # Relay 1 and Servo State Machine
                    # ----------------------------
                    # Read button A input
                    a_button_pressed = _read_joystick_controls(js, a_button_pressed)

                    # Start relay 1 sequence on button A press
                    if a_button_pressed and not r1_active:
                        r1_active = True
                        r1_start_time = time.time()
                        servo_activation_pending = False
                        print("[RELAY 1] Activated for 15 seconds")

                    # Handle relay 1 timer
                    r1 = 0
                    if r1_active and r1_start_time is not None:
                        elapsed = time.time() - r1_start_time

                        if elapsed < RELAY_1_DURATION - 1.0:
                            # Relay 1 is still active (close 1 second before 15 seconds)
                            r1 = 1
                        elif elapsed < RELAY_1_DURATION:
                            # In the last second - relay 1 is closed, waiting to check for object
                            r1 = 0
                        else:
                            # 15 seconds elapsed - activate servo if object still detected
                            if not servo_activation_pending:
                                servo_activation_pending = True
                                servo_pos = SERVO_TRIGGER_VALUE
                                servo_start_time = time.time()
                                print(f"[SERVO] Activated for {SERVO_TRIGGER_DURATION} seconds")

                    # Handle servo deactivation
                    if servo_pos == SERVO_TRIGGER_VALUE and servo_start_time is not None:
                        if time.time() - servo_start_time >= SERVO_TRIGGER_DURATION:
                            servo_pos = 0
                            servo_start_time = None
                            r1_active = False
                            r1_start_time = None
                            servo_activation_pending = False
                            a_button_pressed = False
                            print("[RELAY 1 & SERVO] Deactivated")

                    # ----------------------------
                    # Send to Arduino (if connected)
                    # ----------------------------
                    x_speed = int(yaw * X_SPEED_FACTOR)
                    y_speed = int(pitch * Y_SPEED_FACTOR)

                    ser, last_arduino_msg = send_arduino_message(
                        ser,
                        last_arduino_msg,
                        x_speed,
                        -y_speed,
                        servo_pos=servo_pos,
                        r1=r1,
                        r2=r2,
                    )

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

                    # ----------------------------
                    # Relay 2: deactivate on no detection
                    # ----------------------------
                    r2 = 0

                    # ----------------------------
                    # Relay 1: handle when object is not detected
                    # ----------------------------
                    r1 = 0
                    if r1_active and r1_start_time is not None:
                        elapsed = time.time() - r1_start_time
                        if elapsed >= RELAY_1_DURATION:
                            # 15 seconds elapsed - object is lost so don't activate servo, reset logic
                            print("[RELAY 1 & SERVO] Deactivated (object lost after timeout)")
                            r1_active = False
                            r1_start_time = None
                            servo_activation_pending = False
                            a_button_pressed = False
                            r1 = 0
                        elif elapsed < RELAY_1_DURATION - 1.0:
                            # Still in active period (before the 1-second window)
                            r1 = 1
                        else:
                            # In the 1-second window before timeout, relay is already closed
                            r1 = 0

                    # Send arduino a neutral command if no person detected
                    ser, last_arduino_msg = send_arduino_message(
                        ser,
                        last_arduino_msg,
                        0,
                        0,
                        servo_pos=servo_pos,
                        r1=r1,
                        r2=r2,
                    )

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
        ser, last_arduino_msg = send_arduino_message(
            ser, last_arduino_msg, 0, 0, servo_pos=0, r1=0, r2=0, send_anyway=True)
        cap.release()
        cv2.destroyAllWindows()
        if ser:
            ser.close()
        pygame.quit()
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
