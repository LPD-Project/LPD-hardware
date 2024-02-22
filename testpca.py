from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

minimum_pulse = 500
maximum_pulse = 2500

servo_x = servo.Servo(pca.channels[0], min_pulse = minimum_pulse, max_pulse = maximum_pulse)
servo_y = servo.Servo(pca.channels[1], min_pulse = minimum_pulse, max_pulse = maximum_pulse)
laser_ch = pca.channels[2]

duty_cycle = 0
try:
    while True:
        laser_ch.duty_cycle = duty_cycle * 65535 // 100
        print(duty_cycle)
#        servo_x.angle = duty_cycle * 180 // 100
#        servo_y.angle = duty_cycle * 180 // 100
        time.sleep(1)
        duty_cycle += 10
        
        if duty_cycle > 100:
            print("reset")
            duty_cycle = 0        
        
except KeyboardInterrupt:
    laser_ch.duty_cycle = 0
    servo_x.angle = 45
    servo_y.angle = 45
    #pca.deinit()
    