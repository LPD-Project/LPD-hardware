import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(32, GPIO.OUT)
pwm_laser = GPIO.PWM(32, 100)
gnd_laser = GPIO.PWM(33, 100)
pwm_laser.start(0)
gnd_laser.start(0)

pwm_laser.ChangeDutyCycle(50)

time.sleep(5)

pwm_laser.ChangeDutyCycle(100)

time.sleep(5)
