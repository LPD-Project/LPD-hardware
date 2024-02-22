from adafruit_servokit import ServoKit
import time

servo_pin = 0
kit = ServoKit(channels=16)
kit.set_pwm_frequency(100)
i = 0
while True:
    kit.servo[2].angle = i
    i+= 10
    print(i)
    time.sleep(1)
