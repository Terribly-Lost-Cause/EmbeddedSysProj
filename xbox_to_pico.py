# Use an Xbox controller to drive the Pico robot over USB serial
# Sends the same single-character commands your main.c expects

import time, sys, math
import pygame
import serial

# --- EDIT THIS ---
COM_PORT = "COM5"          # Windows e.g. "COM5"; Linux "/dev/ttyACM0"; macOS "/dev/tty.usbmodemXXXX"
BAUD     = 115200           # must match Pico side
# ------------------

DEADZONE = 0.25         # stick deadzone (ignore tiny movements)
POLL_DT  = 0.01         # loop sleep
RESEND_EVERY = 0.10     # how often to resend the held direction (keeps robot responsive)

# Map stick magnitude to speed level (4..10 -> 40..100%) using clear bins
# Adjust edges if your stick doesn't reach high values (e.g., lower 0.90 to 0.85)
LEVEL_EDGES = [0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.01]
#              ^stop  ^L4   ^L5   ^L6   ^L7   ^L8   ^L9   ^L10(upper cap)

def mag_to_level(m):
    if m < LEVEL_EDGES[0]:
        return None  # treat as STOP
    for i in range(len(LEVEL_EDGES) - 1):
        if LEVEL_EDGES[i] <= m < LEVEL_EDGES[i + 1]:
            return 4 + i
    return 10

# Convert numeric level to the character the Pico expects
def level_to_char(lvl):
    return b'0' if lvl == 10 else bytes(str(lvl), 'ascii')  # '0' means 100%

# Convert 2D stick (ax,ay) to a single direction byte (w/a/s/d) or None
def stick_to_dir(ax, ay):
    # Choose the axis with larger magnitude to get a clean cardinal direction
    if abs(ay) >= abs(ax):
        if ay < -DEADZONE: return b'w'   # forward
        if ay >  DEADZONE: return b's'   # backward
    else:
        if ax < -DEADZONE: return b'a'   # left
        if ax >  DEADZONE: return b'd'   # right
    return None

def main():
    # Serial open
    ser = serial.Serial(COM_PORT, BAUD, timeout=0)
    print(f"[Serial] -> {COM_PORT} @ {BAUD}")

    # Controller init
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found. Plug an Xbox controller and try again.")
        return
    js = pygame.joystick.Joystick(0); js.init()
    print(f"[Controller] {js.get_name()}")

    # Default to 60% to start
    ser.write(b'6')
    last_level = 6                 # <-- track current level
    print("LEVEL: 6 (60%)")

    held_dir = None
    last_sent = 0.0
    rx_buf = b""
    manual_until = 0.0             # <-- ignore stick auto-level until this time if set by buttons

    while True:
        pygame.event.pump()
        ax, ay = js.get_axis(0), js.get_axis(1)  # left stick
        m = math.sqrt(ax*ax + ay*ay)

        # Buttons (may vary slightly by model)
        btn_a = js.get_button(0)  # A -> STOP
        btn_b = js.get_button(1)  # B -> +1 level
        btn_x = js.get_button(2)  # X -> -1 level
        btn_y = js.get_button(3)  # Y -> 100%
        lb    = js.get_button(4)  # LB -> -1 level
        rb    = js.get_button(5)  # RB -> +1 level

        # Quick actions that do not depend on stick
        if btn_a:
            ser.write(b'x')     # stop

        if btn_y:
            ser.write(b'0')           # 100%
            last_level = 10           # <-- keep tracker in sync
            manual_until = time.time() + 0.5
            print("LEVEL: 10 (100%)")

        # Direction from stick (w/a/s/d) + auto-repeat while held
        cmd_dir = stick_to_dir(ax, ay)
        now = time.time()

        # Map magnitude to speed level (4..10) using bins
        lvl = mag_to_level(m)
        if lvl is None:
            # stick returned to center -> stop once
            if held_dir is not None:
                ser.write(b'x')
                held_dir = None
            # do not reset last_level; keep current level memory
        else:
            # Only let stick change level if not in manual override window
            if now >= manual_until and (lvl != last_level):
                ser.write(level_to_char(lvl))
                last_level = lvl
                print(f"LEVEL: {lvl} ({lvl*10}%)")

            # send direction periodically (keeps it responsive)
            if cmd_dir:
                if cmd_dir != held_dir or (now - last_sent) >= RESEND_EVERY:
                    ser.write(cmd_dir)
                    held_dir = cmd_dir
                    last_sent = now

        # B/X or RB/LB adjust levels by 1 step (from the current level)
        if btn_b or rb:
            new_level = min(10, last_level + 1)
            if new_level != last_level:
                ser.write(level_to_char(new_level))
                last_level = new_level
                manual_until = time.time() + 0.5
                print(f"LEVEL: {new_level} ({new_level*10}%)")
            time.sleep(0.08)  # simple de-bounce

        if btn_x or lb:
            new_level = max(4, last_level - 1)
            if new_level != last_level:
                ser.write(level_to_char(new_level))
                last_level = new_level
                manual_until = time.time() + 0.5
                print(f"LEVEL: {new_level} ({new_level*10}%)")
            time.sleep(0.08)

        # Read & print any lines coming from the Pico (speed/status)
        if ser.in_waiting:
            rx_buf += ser.read(ser.in_waiting)
            while b"\n" in rx_buf:
                line, rx_buf = rx_buf.split(b"\n", 1)
                try:
                    print("[PICO]", line.decode(errors="replace"))
                except:
                    print("[PICO] <bin>", line)

        time.sleep(POLL_DT)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
