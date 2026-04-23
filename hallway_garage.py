from initialise import initialise
from time import sleep, ticks_ms, ticks_diff

# initialise hardware
ir_l, ir_c, ir_r, motor_left, motor_right, ultrasonic, servo, enc, LED, oled = initialise()

motor_left.set_forwards()
motor_right.set_forwards()

# tuning
BASE_SPEED = 25
MAX_SPEED = 30
MIN_SPEED = 0
K = 42

LEFT_TRIM = 0
RIGHT_TRIM = 0

# servo angles
LEFT_SCAN_ANGLE = 150
RIGHT_SCAN_ANGLE = 30

# servo timing
SERVO_SETTLE_TIME = 0.50
SERVO_PAUSE_BETWEEN_SCANS = 0.50

# hallway thresholds
WALL_MIN = 10
WALL_MAX = 500
HALLWAY_DIFF_TOLERANCE = 30

# burst timing
HALLWAY_BURST_TIME = 0.30
HALLWAY_PAUSE = 0.05

# forward burst
HALLWAY_FORWARD_LEFT_PWM = 35
HALLWAY_FORWARD_RIGHT_PWM = 30

# move left burst
HALLWAY_MOVE_LEFT_LEFT_PWM = 38
HALLWAY_MOVE_LEFT_RIGHT_PWM = 30

# move right burst
HALLWAY_MOVE_RIGHT_LEFT_PWM = 35
HALLWAY_MOVE_RIGHT_RIGHT_PWM = 35

# IR calibration
L_OFF = 2064
C_OFF = 2240
R_OFF = 2352

LINE_THRESHOLD = 140
CENTER_BOOST_THRESHOLD = 200
LAST_SEEN_MARGIN = 150
SEARCH_TIMEOUT_MS = 500

# state
state = "LINE_FOLLOWING"
last_seen = 0
last_line_time = ticks_ms()

# enc

prev_left = 0
prev_right = 0

# helpers
def reset_encoder():
    global prev_left, prev_right
    enc.clear_count()
    prev_left = 0
    prev_right = 0

def drive_straight(base_speed):
    global prev_left, prev_right

    left_now = enc.get_left()
    right_now = enc.get_right()

    delta_left = left_now - prev_left
    delta_right = right_now - prev_right

    prev_left = left_now
    prev_right = right_now

    error = delta_left - delta_right
    K_enc = 1.2

    left_pwm = base_speed - K_enc * error
    right_pwm = base_speed + K_enc * error

    set_motors(left_pwm, right_pwm)

def servoangle(angle):
    min_duty = 1638
    max_duty = 8192
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty)
    sleep(SERVO_SETTLE_TIME)
    servo.duty_u16(0)

def scan_distance(angle):
    servoangle(angle)
    sleep(SERVO_PAUSE_BETWEEN_SCANS)

    readings = []
    for _ in range(3):
        readings.append(ultrasonic.distance_mm())
        sleep(0.02)

    return sum(readings) / len(readings)

def set_motors(left_pwm, right_pwm):
    left_pwm = max(MIN_SPEED, min(MAX_SPEED, int(left_pwm)))
    right_pwm = max(MIN_SPEED, min(MAX_SPEED, int(right_pwm)))
    motor_left.duty(left_pwm)
    motor_right.duty(right_pwm)

def forwards():
    motor_left.set_forwards()
    motor_right.set_forwards()

def stop():
    motor_left.duty(0)
    motor_right.duty(0)

# sensors
def read_signals():
    L_raw = ir_l.read_u16()
    C_raw = ir_c.read_u16()
    R_raw = ir_r.read_u16()

    L_sig = max(0, L_raw - L_OFF)
    C_sig = max(0, C_raw - C_OFF)
    R_sig = max(0, R_raw - R_OFF)

    return L_sig, C_sig, R_sig

def line_detected(L_sig, C_sig, R_sig):
    return (
        L_sig > LINE_THRESHOLD or
        C_sig > LINE_THRESHOLD or
        R_sig > LINE_THRESHOLD
    )

# line following
def do_line_follow(L_sig, C_sig, R_sig):
    global last_seen

    total_sig = L_sig + C_sig + R_sig
    error = (R_sig - L_sig) / max(total_sig, 1)
    correction = K * error

    left_pwm = BASE_SPEED + LEFT_TRIM + correction
    right_pwm = BASE_SPEED + RIGHT_TRIM - correction

    if C_sig > CENTER_BOOST_THRESHOLD:
        left_pwm += 1
        right_pwm += 1

    forwards()
    set_motors(left_pwm, right_pwm)

    if L_sig > R_sig + LAST_SEEN_MARGIN:
        last_seen = -1
    elif R_sig > L_sig + LAST_SEEN_MARGIN:
        last_seen = 1

def do_line_search():
    if last_seen == -1:
        motor_left.set_backwards()
        motor_right.set_forwards()
        motor_left.duty(30)
        motor_right.duty(25)

    elif last_seen == 1:
        motor_left.set_forwards()
        motor_right.set_backwards()
        motor_left.duty(30)
        motor_right.duty(25)

    else:
        stop()

# hallway detection
def detect_hallway():
    stop()
    sleep(HALLWAY_PAUSE)

    left_distance = scan_distance(LEFT_SCAN_ANGLE)
    right_distance = scan_distance(RIGHT_SCAN_ANGLE)

    print("HALLWAY CHECK | L:", left_distance, "| R:", right_distance)

    left_valid = WALL_MIN <= left_distance <= WALL_MAX
    right_valid = WALL_MIN <= right_distance <= WALL_MAX

    return left_valid and right_valid

# hallway logic
def do_hallway():
    print("ENTERED HALLWAY")
    reset_encoder()
    while True:
        L_sig, C_sig, R_sig = read_signals()

        if line_detected(L_sig, C_sig, R_sig):
            print("LINE FOUND AGAIN")
            stop()
            return "LINE_FOLLOWING"

        stop()
        sleep(HALLWAY_PAUSE)

        left_distance = scan_distance(LEFT_SCAN_ANGLE)
        right_distance = scan_distance(RIGHT_SCAN_ANGLE)

        print("SWEEP | L:", left_distance, "| R:", right_distance)

        left_valid = WALL_MIN <= left_distance <= WALL_MAX
        right_valid = WALL_MIN <= right_distance <= WALL_MAX

        if not left_valid or not right_valid:
            print("BAD READ")
            stop()
            sleep(0.05)
            continue

        error = left_distance - right_distance

        forwards()

        # more room on left, too close to right wall -> move left
        if error > HALLWAY_DIFF_TOLERANCE:
            print("MOVE LEFT")
            set_motors(HALLWAY_MOVE_LEFT_LEFT_PWM, HALLWAY_MOVE_LEFT_RIGHT_PWM)

        # more room on right, too close to left wall -> move right
        elif error < -HALLWAY_DIFF_TOLERANCE:
            print("MOVE RIGHT")
            set_motors(HALLWAY_MOVE_RIGHT_LEFT_PWM, HALLWAY_MOVE_RIGHT_RIGHT_PWM)

        # balanced enough -> forward
        else:
            print("FORWARD")
            drive_straight()

        sleep(HALLWAY_BURST_TIME)
        stop()

def dohallway_2():
    print("ENTERED HALLWAY")

    while True:
        left_distance = scan_distance(LEFT_SCAN_ANGLE)
        right_distance = scan_distance(RIGHT_SCAN_ANGLE)

        print("SWEEP | L:", left_distance, "| R:", right_distance)

        L_sig, C_sig, R_sig = read_signals()

        if line_detected(L_sig, C_sig, R_sig):
            print("LINE FOUND AGAIN")
            stop()
            return "LINE_FOLLOWING"

        stop()
        sleep(HALLWAY_PAUSE)

        if left_distance > 800 or right_distance > 800:
            state == "LINE_SEARCHING"

        else:
            error = left_distance - right_distance

            forwards()

            # more room on left, too close to right wall -> move left
            if error > HALLWAY_DIFF_TOLERANCE:
                print("MOVE LEFT")
                set_motors(HALLWAY_MOVE_LEFT_LEFT_PWM, HALLWAY_MOVE_LEFT_RIGHT_PWM)

            # more room on right, too close to left wall -> move right
            elif error < -HALLWAY_DIFF_TOLERANCE:
                print("MOVE RIGHT")
                set_motors(HALLWAY_MOVE_RIGHT_LEFT_PWM, HALLWAY_MOVE_RIGHT_RIGHT_PWM)

            # balanced enough -> forward
            else:
                print("FORWARD")
                drive_straight(35)

            sleep(HALLWAY_BURST_TIME)
            stop()

def do_hallway_3():
    reset_encoder()
    while True:
        L_sig, C_sig, R_sig = read_signals()

        if line_detected(L_sig, C_sig, R_sig):
            print("LINE FOUND AGAIN")
            stop()
            return "LINE_FOLLOWING"
        else:
            drive_straight(30)

print("HALLWAY SWEEP TEST START")

L_sig, C_sig, R_sig = read_signals()
while not line_detected(L_sig, C_sig, R_sig):
    set_motors(35, 35)
    sleep(0.5)
    L_sig, C_sig, R_sig = read_signals()


while True:
    now = ticks_ms()
    L_sig, C_sig, R_sig = read_signals()

    if state == "LINE_FOLLOWING":
        if line_detected(L_sig, C_sig, R_sig):
            do_line_follow(L_sig, C_sig, R_sig)
            last_line_time = now
        else:
            if ticks_diff(now, last_line_time) < SEARCH_TIMEOUT_MS:
                state = "LINE_SEARCH"
            else:
                state = "NO_LINE"

    elif state == "LINE_SEARCH":
        if line_detected(L_sig, C_sig, R_sig):
            state = "LINE_FOLLOWING"
            last_line_time = now
        else:
            if detect_hallway():
                print("HALLWAY DETECTED")
                state = "HALLWAY"
            else:
                do_line_search()

    elif state == "NO_LINE":
        if detect_hallway():
            print("HALLWAY DETECTED")
            state = "HALLWAY"
        else:
            stop()

    elif state == "HALLWAY":
        distance = scan_distance(90)
        if distance < 400:
            state = "GARAGE_END"
        else:
            dohallway_2()
            last_line_time = ticks_ms()
            servoangle(90)
            state = "LINE_FOLLOWING"

    elif state == "GARAGE_END":
        reset_encoder()
        distance = scan_distance(90)
        while distance >= 150:
            drive_straight(30)
            sleep(0.5)
            stop()
            distance = scan_distance(90)
        state = "FINISHED"

    elif state == "FINISHED":
        stop()

    print("STATE:", state)
    sleep(0.01)