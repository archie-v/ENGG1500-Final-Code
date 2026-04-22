from motor import Motor
from ultrasonic import sonic
from machine import Pin, PWM, ADC
from time import sleep, ticks_ms, ticks_diff
from encoder import Encoder

# set IR pin ins as analogue
ir_r = ADC(Pin(26))
ir_c = ADC(Pin(27))
ir_l = ADC(Pin(28))

# set motors
motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)

# set ultrasonic sensor trig and echo
ultrasonic = sonic(2, 3)

# set servo motor pwm pin
pwm = PWM(Pin(15))
pwm.freq(50)

# set encoder
enc = Encoder(18, 19)

# set green LED indicator
LED = Pin("LED", Pin.OUT)

# wait for a moment for it all to turn on
sleep(0.1)
print("All Initialised!")

BASE_SPEED = 22
MAX_SPEED = 35
MIN_SPEED = 0
K = 65

# calibrated baselines
L_OFF = 2064
C_OFF = 2240
R_OFF = 2352

# thresholds
LINE_THRESHOLD = 140
SEARCH_TIMEOUT_MS = 1500

last_seen = 0
last_line_time = ticks_ms()

def calibrate(left, right, enc, pwm=[30, 50, 80, 100], time=1):

    values = {}

    left.set_forwards()
    right.set_forwards()

    for i in pwm:
        enc.clear_count()
        left.duty(i)
        right.duty(i)

        sleep(time)

        l = enc.get_left()
        r = enc.get_right()
        print(l)
        print(r)

        if l > r:
            l = round((r / l) * i)
            r = i

        elif r > l:
            r = round((l / r) * i)
            l = i

        else:
            l = i
            r = i

        values[f"l_{i}"] = l
        values[f"r_{i}"] = r

        left.duty(0)
        right.duty(0)
        sleep(0.5)

    return values

def set_motors(left_pwm, right_pwm):
    left_pwm = max(-MAX_SPEED, min(MAX_SPEED, int(left_pwm)))
    right_pwm = max(-MAX_SPEED, min(MAX_SPEED, int(right_pwm)))

    if left_pwm < 0:
        motor_left.set_backwards()
        motor_left.duty(-left_pwm-10)
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
    motor_left.set_forwards()
    motor_right.set_forwards()

def servoangle(angle):
    min_duty = 1638  # ~0.5 ms
    max_duty = 8192  # ~2.5 ms
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    pwm.duty_u16(duty)
    sleep(2)
    pwm.duty_u16(0)

motor_left.set_forwards()
motor_right.set_forwards()

distance = ultrasonic.distance_mm()

total_sig = 0

state = "GARAGE_START"

while state == "GARAGE_START":
    L_raw = ir_l.read_u16()
    C_raw = ir_c.read_u16()
    R_raw = ir_r.read_u16()

    L_sig = max(0, L_raw - L_OFF)
    C_sig = max(0, C_raw - C_OFF)
    R_sig = max(0, R_raw - R_OFF)

    total_sig = L_sig + C_sig + R_sig

    if total_sig >= LINE_THRESHOLD:
        state = "LINE_FOLLOW"   # assignment, not comparison
    else:
        set_motors(30, 30)      # slightly higher than 15 to overcome static friction
        sleep(0.005)

stop()
sleep(0.1)

state = "LINE_FOLLOW"
servoangle(180)


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
        distance = ultrasonic.distance_mm()
        if distance <= 100:
            state = "HALLWAY/GARAGE"
        else:
            if last_seen == -1:
                set_left_search()
            elif last_seen == 1:
                set_right_search()

    if state == "HALLWAY/GARAGE":
        stop()
        sleep(0.05)
        servoangle(90)
        distance = ultrasonic.distance_mm()
        if distance <= 400:
            state = "GARAGE_END"
        else:
            set_motors(25, 25)
            sleep(1.5)
            servoangle(180)
            state = "LINE_FOLLOW"

    if state == "GARAGE_END":
        while distance > 50:
            distance = ultrasonic.distance_mm()
            set_motors(25, 25)
        state = "FINISHED"

    while state == "FINISHED":
        stop()

    sleep(0.005)