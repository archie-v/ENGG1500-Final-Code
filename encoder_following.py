from initialise import initialise
from time import sleep, ticks_ms, ticks_diff

# initialise hardware
ir_l, ir_c, ir_r, motor_left, motor_right, ultrasonic, servo, enc, LED, oled = initialise()

motor_left.set_forwards()
motor_right.set_forwards()

# speed settings
BASE_SPEED = 22
MAX_SPEED = 34
MIN_SPEED = 0
K = 65

# calibrated baselines
L_OFF = 2064
C_OFF = 2240
R_OFF = 2352

# thresholds
LINE_THRESHOLD = 140
SEARCH_TIMEOUT_MS = 500

# state
state = "LINE_FOLLOW"
last_seen = 0
last_line_time = ticks_ms()

def set_motors(left_pwm, right_pwm):
    left_pwm = max(MIN_SPEED, min(MAX_SPEED, int(left_pwm)))
    right_pwm = max(MIN_SPEED, min(MAX_SPEED, int(right_pwm)))

    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(left_pwm)
    motor_right.duty(right_pwm)

def set_left_search():
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_left.duty(30)
    motor_right.duty(30)

def set_right_search():
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(30)
    motor_right.duty(30)

def stop():
    motor_left.duty(0)
    motor_right.duty(0)

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

        if L_sig > R_sig + 150:
            last_seen = -1
        elif R_sig > L_sig + 150:
            last_seen = 1
        else:
            last_seen = 0

    elif state == "LINE_SEARCH":
        if last_seen == -1:
            set_left_search()
        elif last_seen == 1:
            set_right_search()
        else:
            stop()

    elif state == "HALLWAY":
        set_motors(25, 25)

    print("state:", state, "| sig:", L_sig, C_sig, R_sig)

    sleep(0.005)