from initialise import initialise, angle
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

# line thresholds
LINE_THRESHOLD = 140
CENTRE_BOOST_THRESHOLD = 200
SIDE_DOMINANCE = 175

# fork thresholds
FORK_SIDE_THRESHOLD = 180
FORK_CENTRE_MAX = 120
FORK_TURN_LEFT_PWM = 34
FORK_TURN_RIGHT_PWM = 40

# search settings
SEARCH_TIMEOUT_MS = 500
SEARCH_TURN_MS = 180

# hallway settings
HALLWAY_SPEED = 18
HALLWAY_CORRECT_SPEED = 34
HALLWAY_FORWARD_BURST_MS = 120
HALLWAY_CORRECT_BURST_MS = 90
HALLWAY_STOP_MS = 50
SERVO_SETTLE_TIME = 0.08

# ultrasonic thresholds (mm)
WALL_THRESHOLD_MM = 200
STOP_DISTANCE_MM = 80
SIDE_THRESHOLD_MM = 150
DIST_DIFF_THRESHOLD_MM = 35
DIST_MIN_MM = 20
DIST_MAX_MM = 600

# garage logic
GARAGE_END_COOLDOWN_MS = 2000
MIN_LINE_TIME_MS = 2000

# roundabout thresholds
ROUNDABOUT_ALL_THRESHOLD = 180
ROUNDABOUT_RIGHT_EXIT_THRESHOLD = 180
ROUNDABOUT_ENTRY_TURN_MS = 420
ROUNDABOUT_EXIT_TURN_MS = 320
ROUNDABOUT_COOLDOWN_MS = 1200

# servo angles
SERVO_RIGHT = 0
SERVO_CENTRE = 90
SERVO_LEFT = 180

# state
state = "START"
last_seen = 0
last_line_time = ticks_ms()
line_confirmed_time = None
garage_end_last_failed = 0
search_turn_until = 0

# roundabout flags
roundabout_mode = False
roundabout_stage = 0
roundabout_entry_turning = False
roundabout_entry_start = 0
roundabout_exit_turning = False
roundabout_exit_start = 0
roundabout_cooldown_until = 0

# encoder
prev_left = 0
prev_right = 0


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


def clamp_pwm(x):
    return max(-MAX_SPEED, min(MAX_SPEED, int(x)))


def set_motors(left_pwm, right_pwm):
    left_pwm = clamp_pwm(left_pwm)
    right_pwm = clamp_pwm(right_pwm)

    if left_pwm < 0:
        motor_left.set_backwards()
        motor_left.duty(min(MAX_SPEED, -left_pwm + 35))
    else:
        motor_left.set_forwards()
        motor_left.duty(min(MAX_SPEED, left_pwm + 8))

    if right_pwm < 0:
        motor_right.set_backwards()
        motor_right.duty(min(MAX_SPEED, -right_pwm + 10))
    else:
        motor_right.set_forwards()
        motor_right.duty(min(MAX_SPEED, right_pwm + 8))


def stop():
    motor_left.duty(0)
    motor_right.duty(0)


def set_left_search():
    stop()
    sleep(0.03)
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_left.duty(34)
    motor_right.duty(34)


def set_right_search():
    stop()
    sleep(0.03)
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(34)
    motor_right.duty(34)


def hallway_turn_left():
    stop()
    sleep(0.02)
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_left.duty(HALLWAY_CORRECT_SPEED)
    motor_right.duty(HALLWAY_CORRECT_SPEED)


def hallway_turn_right():
    stop()
    sleep(0.02)
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(HALLWAY_CORRECT_SPEED)
    motor_right.duty(HALLWAY_CORRECT_SPEED)


def set_roundabout_right_turn():
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(38)
    motor_right.duty(34)


def set_fork_left_turn():
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_left.duty(FORK_TURN_LEFT_PWM)
    motor_right.duty(FORK_TURN_RIGHT_PWM)


def read_distance():
    d = ultrasonic.distance_mm()
    if d is None or d <= 0:
        return None
    return d


def valid_distance(d):
    return d is not None and DIST_MIN_MM <= d <= DIST_MAX_MM


def read_line_signals():
    L_raw = ir_l.read_u16()
    C_raw = ir_c.read_u16()
    R_raw = ir_r.read_u16()

    L_sig = max(0, L_raw - L_OFF)
    C_sig = max(0, C_raw - C_OFF)
    R_sig = max(0, R_raw - R_OFF)

    return L_sig, C_sig, R_sig


def line_detected():
    L_sig, C_sig, R_sig = read_line_signals()
    return (L_sig + C_sig + R_sig) >= LINE_THRESHOLD, L_sig, C_sig, R_sig


def scan_environment():
    angle(SERVO_RIGHT, servo)
    sleep(SERVO_SETTLE_TIME)
    right_d = read_distance()

    angle(SERVO_CENTRE, servo)
    sleep(SERVO_SETTLE_TIME)
    centre_d = read_distance()

    angle(SERVO_LEFT, servo)
    sleep(SERVO_SETTLE_TIME)
    left_d = read_distance()

    angle(SERVO_CENTRE, servo)
    sleep(SERVO_SETTLE_TIME)

    right_seen = valid_distance(right_d) and right_d < SIDE_THRESHOLD_MM
    centre_seen = valid_distance(centre_d) and centre_d < WALL_THRESHOLD_MM
    left_seen_scan = valid_distance(left_d) and left_d < SIDE_THRESHOLD_MM

    if left_seen_scan and centre_seen and right_seen:
        env = "FINAL_GARAGE"
    elif left_seen_scan and right_seen:
        env = "HALLWAY_OR_STARTER"
    else:
        env = "OPEN"

    return left_d, centre_d, right_d, env


def end_sequence():
    angle(SERVO_LEFT, servo)
    sleep(0.5)
    dist_left = read_distance()

    angle(SERVO_RIGHT, servo)
    sleep(0.5)
    dist_right = read_distance()

    angle(SERVO_CENTRE, servo)
    sleep(0.2)

    if (
        dist_left is not None and
        dist_right is not None and
        dist_left < SIDE_THRESHOLD_MM and
        dist_right < SIDE_THRESHOLD_MM
    ):
        reset_encoder()

        while True:
            dist_ahead = read_distance()

            if dist_ahead is not None and dist_ahead <= STOP_DISTANCE_MM:
                stop()
                return True

            drive_straight(12)
            sleep(0.005)
    else:
        return False


def hallway_burst_step():
    stop()
    sleep(HALLWAY_STOP_MS / 1000)

    found, L_sig, C_sig, R_sig = line_detected()
    if found:
        return "LINE"

    left_d, centre_d, right_d, env = scan_environment()

    print("scan:", "L=", left_d, "C=", centre_d, "R=", right_d, "| env:", env)

    if env == "FINAL_GARAGE":
        return "FINAL_GARAGE"

    if env == "HALLWAY_OR_STARTER":
        if left_d is not None and right_d is not None:
            diff = left_d - right_d

            if diff > DIST_DIFF_THRESHOLD_MM:
                hallway_turn_left()
                sleep(HALLWAY_CORRECT_BURST_MS / 1000)

            elif diff < -DIST_DIFF_THRESHOLD_MM:
                hallway_turn_right()
                sleep(HALLWAY_CORRECT_BURST_MS / 1000)

    drive_straight(HALLWAY_SPEED)
    sleep(HALLWAY_FORWARD_BURST_MS / 1000)

    stop()
    sleep(HALLWAY_STOP_MS / 1000)

    found, L_sig, C_sig, R_sig = line_detected()
    if found:
        return "LINE"

    return "NONE"


# runtime calibration
stop()
sleep(0.5)

NUM_SAMPLES = 50
l_sum = 0
c_sum = 0
r_sum = 0

for _ in range(NUM_SAMPLES):
    l_sum += ir_l.read_u16()
    c_sum += ir_c.read_u16()
    r_sum += ir_r.read_u16()
    sleep(0.005)

L_OFF = l_sum // NUM_SAMPLES
C_OFF = c_sum // NUM_SAMPLES
R_OFF = r_sum // NUM_SAMPLES

print("Calibrated baselines:", L_OFF, C_OFF, R_OFF)

# garage start
while True:
    found, L_sig, C_sig, R_sig = line_detected()

    if found:
        break

    set_motors(15, 15)
    sleep(0.005)

state = "LINE_FOLLOW"
last_line_time = ticks_ms()

while True:
    now = ticks_ms()

    found, L_sig, C_sig, R_sig = line_detected()
    total_sig = L_sig + C_sig + R_sig

    # forced roundabout entry turn
    if roundabout_entry_turning:
        if ticks_diff(now, roundabout_entry_start) < ROUNDABOUT_ENTRY_TURN_MS:
            state = "ROUNDABOUT_ENTRY"
            set_roundabout_right_turn()
            print("state:", state, "| sig:", L_sig, C_sig, R_sig)
            sleep(0.005)
            continue
        else:
            roundabout_entry_turning = False
            roundabout_stage = 1

    # forced roundabout exit commit turn
    if roundabout_exit_turning:
        if ticks_diff(now, roundabout_exit_start) < ROUNDABOUT_EXIT_TURN_MS:
            state = "ROUNDABOUT_EXIT"
            set_roundabout_right_turn()
            print("state:", state, "| sig:", L_sig, C_sig, R_sig)
            sleep(0.005)
            continue
        else:
            roundabout_exit_turning = False
            roundabout_mode = False
            roundabout_stage = 0
            roundabout_cooldown_until = now + ROUNDABOUT_COOLDOWN_MS

    # search lock
    if ticks_diff(search_turn_until, now) > 0:
        state = "LINE_SEARCH"

        if last_seen == -1:
            set_left_search()
        elif last_seen == 1:
            set_right_search()
        else:
            stop()

        print("state:", state, "| sig:", L_sig, C_sig, R_sig, "| last_seen:", last_seen)
        sleep(0.005)
        continue

    # roundabout trigger
    roundabout_allowed = ticks_diff(now, roundabout_cooldown_until) >= 0

    if (
        roundabout_allowed and
        not roundabout_mode and
        L_sig > ROUNDABOUT_ALL_THRESHOLD and
        C_sig > ROUNDABOUT_ALL_THRESHOLD and
        R_sig > ROUNDABOUT_ALL_THRESHOLD
    ):
        roundabout_mode = True
        roundabout_stage = 0
        roundabout_entry_turning = True
        roundabout_entry_start = now
        state = "ROUNDABOUT_ENTRY"
        print("state:", state, "| trigger full line")
        sleep(0.005)
        continue

    # roundabout state tracking
    if roundabout_mode:
        if roundabout_stage == 1:
            if R_sig > ROUNDABOUT_RIGHT_EXIT_THRESHOLD and C_sig < ROUNDABOUT_ALL_THRESHOLD:
                roundabout_stage = 2

        elif roundabout_stage == 2:
            if R_sig > ROUNDABOUT_RIGHT_EXIT_THRESHOLD and C_sig < ROUNDABOUT_ALL_THRESHOLD:
                roundabout_exit_turning = True
                roundabout_exit_start = now
                state = "ROUNDABOUT_EXIT"
                print("state:", state, "| second exit found")
                sleep(0.005)
                continue

    # state decision
    if found:
        state = "LINE_FOLLOW"
        last_line_time = now

        if line_confirmed_time is None:
            line_confirmed_time = now

    else:
        time_lost = ticks_diff(now, last_line_time)
        time_since_fail = ticks_diff(now, garage_end_last_failed)
        dist_ahead = read_distance()

        line_established = (
            line_confirmed_time is not None and
            ticks_diff(now, line_confirmed_time) > MIN_LINE_TIME_MS
        )

        if (
            dist_ahead is not None and
            dist_ahead < WALL_THRESHOLD_MM and
            line_established and
            time_since_fail > GARAGE_END_COOLDOWN_MS
        ):
            state = "GARAGE_CHECK"

        elif time_lost < SEARCH_TIMEOUT_MS:
            state = "LINE_SEARCH"
            search_turn_until = now + SEARCH_TURN_MS

        else:
            state = "HALLWAY"
            reset_encoder()

    if state == "LINE_FOLLOW":
        if (
            L_sig > FORK_SIDE_THRESHOLD and
            R_sig > FORK_SIDE_THRESHOLD and
            C_sig < FORK_CENTRE_MAX
        ):
            state = "FORK_LEFT"
            set_fork_left_turn()
            last_seen = -1

        elif L_sig > C_sig + SIDE_DOMINANCE and L_sig > R_sig + SIDE_DOMINANCE:
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(32)
            motor_right.duty(38)
            last_seen = -1

        elif R_sig > C_sig + SIDE_DOMINANCE and R_sig > L_sig + SIDE_DOMINANCE:
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(38)
            motor_right.duty(32)
            last_seen = 1

        else:
            error = (R_sig - L_sig) / max(total_sig, 1)
            correction = K * error

            left_pwm = BASE_SPEED + correction
            right_pwm = BASE_SPEED - correction

            if C_sig > CENTRE_BOOST_THRESHOLD:
                left_pwm += 1
                right_pwm += 1
                drive_straight(BASE_SPEED)
            else:
                set_motors(left_pwm, right_pwm)

            if L_sig > R_sig + 120:
                last_seen = -1
            elif R_sig > L_sig + 120:
                last_seen = 1

    elif state == "LINE_SEARCH":
        if last_seen == -1:
            set_left_search()
        elif last_seen == 1:
            set_right_search()
        else:
            stop()

    elif state == "HALLWAY":
        result = hallway_burst_step()

        if result == "LINE":
            last_line_time = ticks_ms()
            state = "LINE_FOLLOW"

        elif result == "FINAL_GARAGE":
            state = "GARAGE_CHECK"

    elif state == "GARAGE_CHECK":
        stop()
        confirmed = end_sequence()

        if confirmed:
            print("state: END_GARAGE")
            break
        else:
            garage_end_last_failed = ticks_ms()
            state = "HALLWAY"
            reset_encoder()

    print(
        "state:", state,
        "| sig:", L_sig, C_sig, R_sig,
        "| last_seen:", last_seen,
        "| roundabout_mode:", roundabout_mode,
        "| roundabout_stage:", roundabout_stage
    )

    sleep(0.005)