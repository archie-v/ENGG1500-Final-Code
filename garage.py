from initialise import initialise
from time import sleep, ticks_ms, ticks_diff

# initialise hardware
ir_l, ir_c, ir_r, motor_left, motor_right, ultrasonic, servo, enc, LED, oled = initialise()

motor_left.set_forwards()
motor_right.set_forwards()

# speed settings
BASE_SPEED = 22
MAX_SPEED = 47
MIN_SPEED = 0
K = 65

# calibrated baselines
L_OFF = 2064
C_OFF = 2240
R_OFF = 2352

# thresholds
LINE_THRESHOLD = 140
SEARCH_TIMEOUT_MS = 1500
WALL_THRESHOLD_MM = 200
STOP_DISTANCE_MM = 80
SIDE_THRESHOLD_MM = 150

# state
state = "GARAGE_START"
last_seen = 0
last_line_time = ticks_ms()
garage_end_confirmed = False

def set_motors(left_pwm, right_pwm):
    left_pwm = max(-MAX_SPEED, min(MAX_SPEED, int(left_pwm)))
    right_pwm = max(-MAX_SPEED, min(MAX_SPEED, int(right_pwm)))

    if left_pwm < 0:
        motor_left.set_backwards()
        motor_left.duty(-left_pwm-40)
    else:
        motor_left.set_forwards()
        motor_left.duty(left_pwm+10)

    if right_pwm < 0:
        motor_right.set_backwards()
        motor_right.duty(-right_pwm-10)
    else:
        motor_right.set_forwards()
        motor_right.duty(right_pwm+10)

def set_left_search():
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_left.duty(40)
    motor_right.duty(40)

def set_right_search():
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(40)
    motor_right.duty(40)

def stop():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(0)
    motor_right.duty(0)

def end_sequence():
    angle(180, servo)
    sleep(0.5)
    dist_left = ultrasonic.get_distance()

    angle(0, servo)
    sleep(0.5)
    dist_right = ultrasonic.get_distance()

    angle(90, servo)
    sleep(0.2)

    if dist_left < SIDE_THRESHOLD_MM and dist_right < SIDE_THRESHOLD_MM:
        while True:
            dist_ahead = ultrasonic.get_distance()
            if dist_ahead <= STOP_DISTANCE_MM:
                stop()
                return True
            set_motors(12, 12)
            sleep(0.005)
    else:
        return False


# GARAGE START
motor_left.set_forwards()
motor_right.set_forwards()

while True:
    L_raw = ir_l.read_u16()
    C_raw = ir_c.read_u16()
    R_raw = ir_r.read_u16()

    L_sig = max(0, L_raw - L_OFF)
    C_sig = max(0, C_raw - C_OFF)
    R_sig = max(0, R_raw - R_OFF)

    total_sig = L_sig + C_sig + R_sig

    if total_sig>= LINE_THRESHOLD:
        break

    set_motors(15, 15)
    sleep(0.005)

state = "LINE_FOLLOW"

while True:
    now = ticks_ms()

    L_raw = ir_l.read_u16()
    C_raw = ir_c.read_u16()
    R_raw = ir_r.read_u16()

    L_sig = max(0, L_raw - L_OFF)
    C_sig = max(0, C_raw - C_OFF)
    R_sig = max(0, R_raw - R_OFF)

    total_sig = L_sig + C_sig + R_sig

    if total_sig >= LINE_THRESHOLD:
        state = "LINE_FOLLOW"
        last_line_time = now
    else:
        time_lost = ticks_diff(now, last_line_time)

        if time_lost < SEARCH_TIMEOUT_MS:
            state = "LINE_SEARCH"
        elif not garage_end_confirmed:
            dist = ultrasonic.get_distance()
            if dist < WALL_THRESHOLD_MM:
                state = "GARAGE_END"
            else:
                state = "HALLWAY"
        else:
            state = "HALLWAY"

    if state == "LINE_FOLLOW":
        error = (R_sig - L_sig) / max(total_sig, 1)
        correction = K * error

        left_pwm = BASE_SPEED + correction
        right_pwm = BASE_SPEED - correction

        if C_sig > 200:
            left_pwm += 1
            right_pwm += 1

        set_motors(left_pwm, right_pwm)

        if L_sig > R_sig + 175:
            last_seen = -1
        elif R_sig > L_sig + 175:
            last_seen = 1

    elif state == "LINE_SEARCH":
        if last_seen == -1:
            set_left_search()
        elif last_seen == 1:
            set_right_search()
        else:
            stop()

    elif state == "GARAGE_END":
        stop()
        confirmed = end_sequence()
        if confirmed:
            break
        else:
            state = "HALLWAY"

    elif state == "HALLWAY":
        set_motors(22, 22)

    print("state:", state, "| sig:", L_sig, C_sig, R_sig)

    sleep(0.005)