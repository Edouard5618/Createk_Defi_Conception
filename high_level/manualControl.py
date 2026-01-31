import pygame
import serial
import time

# ------------------ CONFIG ------------------
SERIAL_PORT = "COM9"      # Change to your port (e.g. "/dev/ttyUSB0")
BAUDRATE = 115200

# X_MIN, X_MAX = -1000, 1000
# Y_MIN, Y_MAX = -1000, 1000

SPEED_SCALE = 1000       # Position increment per loop at full stick deflection
LOOP_DT = 0.02            # 50 Hz update
# --------------------------------------------

# Serial setup
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    print(f"✓ Connexion série établie sur {SERIAL_PORT}")
except Exception as e:
    print(f"⚠ ATTENTION: Impossible de se connecter à {SERIAL_PORT}")
    print(f"  Erreur: {e}")
    print(f"  → Continuation sans port série (manette seule)")
    ser = None

# Pygame joystick setup
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

js = pygame.joystick.Joystick(0)

# Positions
x_speed = 0
y_speed = 0
servo_pos = 0
start_time = None


print("Controller active. Sending serial data...")

try:
    while True:
        pygame.event.pump()

        # Read joystick axes
        left_y = -js.get_axis(1)   # invert so up = positive
        right_x = js.get_axis(2)
        # print(f"Left Y: {left_y:.2f}, Right X: {right_x:.2f}")

        # Increment positions
        x_speed = int(right_x * SPEED_SCALE)
        y_speed = int(left_y * SPEED_SCALE)

        # Clamp
        # x_speed = max(X_MIN, min(X_MAX, x_speed))
        # y_speed = max(Y_MIN, min(Y_MAX, y_speed))

        # Buttons (example)
        r1 = js.get_button(5)
        r2 = js.get_button(4)

        # Fixed speed value (adjust if needed)
        a_pressed = js.get_button(0)
        if a_pressed and servo_pos == 0:  # Only activate if servo is not already moving
            servo_pos = 255
            start_time = time.time()  # Record the start time

        # Reset servo position after 3 seconds
        if servo_pos == 255 and start_time is not None and time.time() - start_time >= 3:
            servo_pos = 0

        # Serial message
        msg = f"X{x_speed} Y{y_speed} S{servo_pos} R1:{r1} R2:{r2}\n"

        if ser:
            ser.write(msg.encode("ascii"))
            print(f"Sent: {msg.strip()}")
        else:
            print("Serial not connected. Message would be:")
            print(msg.strip())

        if ser:
            ser.flushOutput()
            ser.flushInput()

        time.sleep(LOOP_DT)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    if ser:
        ser.close()
    pygame.quit()
