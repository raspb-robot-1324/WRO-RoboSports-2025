from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Direction, Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from usys import stdin, stdout
from uselect import poll
import time

# -------------------
# Hardware setup
# -------------------
hub = PrimeHub()

lm = Motor(Port.A)
rm = Motor(Port.B, Direction.COUNTERCLOCKWISE)
spin = Motor(Port.C)

drive_base = DriveBase(lm, rm, wheel_diameter=52, axle_track=140)

kbd = poll()
kbd.register(stdin)

def read_line_nonblocking():
    if not kbd.poll(0):
        return None
    line = ""
    while True:
        ch = stdin.read(1)
        if ch is None:
            break
        if ch == "\n":
            break
        line += ch
        if len(line) > 64:  # guard against runaway lines
            break
    return line.strip() if line else None

def parse_coords_strict(line):
    """Require 'x,y' with both ints. Return (x, y) or None."""
    if not line or "," not in line:
        return None
    a, b = line.split(",", 1)
    try:
        x = int(a)
        y = int(b)
        return (x, y)
    except ValueError:
        return None

# -------------------
# Behavior parameters
# -------------------
FOV_W = 600
FOV_H = 480

max_speed = 500
search_speed = 100
chase_adjust_speed = 100

center_band_px = 15
center_hold_s = 0.5

# -------------------
# State
# -------------------
ball_coords = []
search_countdown = 0.0
search_countdown_is_set = False

is_chasing = False
ball_is_close = False

scout_steps = 0

chase_log = []
_last_log_ts = None

# -------------------
# Low-level helpers
# -------------------
def run_motors(L, R):
    lm.run(L)
    rm.run(R)

def read_coords_once():
    global ball_coords, search_countdown_is_set
    stdout.buffer.write(b"rdy")
    line = read_line_nonblocking()
    if line is None:
        return
    coords = parse_coords_strict(line)
    if coords is None:
        return
    x, y = coords
    if 0 <= x < FOV_W and 0 <= y < FOV_H:
        ball_coords = [x, y]
        if not (FOV_W//2 - center_band_px <= x <= FOV_W//2 + center_band_px):
            search_countdown_is_set = False

def _log_chase_slice(L, R):
    global _last_log_ts
    now = time.perf_counter()
    if _last_log_ts is None:
        _last_log_ts = now
        return
    dt = now - _last_log_ts
    _last_log_ts = now
    if dt > 0:
        chase_log.append((int(dt * 1000), float(L), float(R)))

# -------------------
# Behaviors
# -------------------
def scout():
    global scout_steps
    drive_base.turn(90)
    scout_steps += 1
    if (scout_steps % 4) == 0:
        drive_base.straight(-150)

def update_search():
    global is_chasing, search_countdown_is_set, search_countdown, _last_log_ts
    if ball_coords:
        x = ball_coords[0]
        L = -search_speed * (x - FOV_W // 2) * 2 / FOV_W
        R = -search_speed * (FOV_W // 2 - x) * 2 / FOV_W
        run_motors(L, R)

        centered = (FOV_W // 2 - center_band_px) <= x <= (FOV_W // 2 + center_band_px)
        if centered:
            if not search_countdown_is_set:
                search_countdown = time.perf_counter()
                search_countdown_is_set = True
            if time.perf_counter() - search_countdown > center_hold_s:
                is_chasing = True
                chase_log.clear()
                _last_log_ts = None
        else:
            search_countdown_is_set = False
    else:
        scout()

def chase():
    global is_chasing, ball_is_close
    if ball_coords:
        x, y = ball_coords
        L = -max_speed - chase_adjust_speed * (x / FOV_W)
        R = -max_speed - chase_adjust_speed * (1 - x / FOV_W)
        run_motors(L, R)
        _log_chase_slice(L, R)

        if y > (5 * FOV_H) // 6:
            ball_is_close = True
    elif ball_is_close:
        throw_ball()
    else:
        is_chasing = False

def turn_by_degrees(deg):
    drive_base.turn(deg)

def return_to_start():
    global scout_steps
    if chase_log:
        for dt_ms, L, R in reversed(chase_log):
            run_motors(-L, -R)
            wait(max(1, int(dt_ms)))
        lm.hold()
        rm.hold()

    if scout_steps:
        drive_base.turn(-90 * (scout_steps % 4))
        scout_steps = 0

#TODO: VERIFY IF THIS WORKS
def throw_ball():
    global ball_is_close, is_chasing

    spin.run(-120)
    lm.run_angle(-max_speed // 2, 180, wait=False)
    rm.run_angle(-max_speed // 2, 180)
    lm.run_angle(max_speed, 180, wait=False)
    rm.run_angle(max_speed, 180)
    turn_by_degrees(30)
    run_motors(-max_speed, -max_speed)
    spin.run(-500)
    wait(600)
    spin.stop()
    run_motors(max_speed, max_speed)
    wait(600)

    ball_is_close = False
    is_chasing = False
    return_to_start()

def first_balls():
    drive_base.straight(1000)
    spin.run_time(1200, 5000)
    drive_base.straight(-200)
    drive_base.turn(90)
    drive_base.straight(220)
    drive_base.turn(90)
    drive_base.straight(750)
    drive_base.turn(90)
    drive_base.straight(250)
    drive_base.turn(90)
    drive_base.straight(1000)
    spin.run_time(1200, 5000)
    drive_base.straight(-1000)

# -------------------
# Main loop
# -------------------

first_balls()
while True:
    read_coords_once()

    if is_chasing:
        chase()
    else:
        update_search()

    wait(10)
