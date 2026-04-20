"""
An initialisation definition that imports all modules, and returns every sensor used on the
robot. This code just has to be called at the start of the program, and it will work.

Example in other code:
from initialise import initialise
ir_l, ir_c, ir_r, motor_left, motor_right, ultrasonic, servo, enc, LED, oled = initialise()

"""

def initialise():
    # bring in a bunch of modules for the robot. As long as the pins are right, this should work
    from motor import Motor
    from ultrasonic import sonic
    from machine import Pin, PWM, ADC, I2C
    from time import sleep
    from encoder import Encoder
    from ssd1306 import SSD1306_I2C

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
    servo = PWM(Pin(15))
    servo.freq(50)

    # set encoder
    enc = Encoder(18, 19)

    # set green LED indicator
    LED = Pin("LED", Pin.OUT)

    # Initialise OLED display
    oled_pin = I2C(0, sda=Pin(4), scl=Pin(5))
    print(oled_pin.scan())
    oled = SSD1306_I2C(128, 64, oled_pin)

    # wait for a moment for it all to turn on
    sleep(0.1)
    print("All Initialised!")

    # send back a list - copy-paste this into the top of the other doc
    return ir_l, ir_c, ir_r, motor_left, motor_right, ultrasonic, servo, enc, LED, oled


"""
angle sets the angle of the servo motor, as a percentage of the duty cycle. You need to specify
both the angle and the servo motor.

Example: angle(90, servo)
"""

def angle(angle, servo):
    position = int(8000 * ((angle + 7) / 180) + 1000)
    servo.duty_u16(position)


"""
calibrate uses the motors and encoders to proportionally adjust the pwm values (for a set list)
and return a dictionary.
This assumes that pwm is in a linear relationship to motor speed, which of course it isn't, but
is fairly good.

Example: pwm = calibrate(motor_left, motor_right, enc)
motor_left.duty(pwm[l_50])
motor_right.duty(pwm[r_50])
"""

def calibrate(left, right, enc, pwm=[30, 50, 80, 100], time=1):
    from motor import Motor
    from encoder import Encoder
    from time import sleep

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


"""
oled_text will set text to the position (x,y) on the screen, and show it. It also prints to the
console.

Example: oled_text("Hello, World!", oled).
"""

def oled_text(text, oled, x=0, y=0):
    from ssd1306 import SSD1306_I2C
    oled.fill(0)
    oled.text(text, x, y)
    oled.show()
    print(text)