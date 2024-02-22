import board
import busio
from adafruit_pca9685 import PCA9685
import time

i2c = busio.I2C(board.SCL, board.SDA)

pca = PCA9685(i2c)
pca.frequency = 50

MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500
NEUTRAL_POSITION = 1500
DEAD_BAND_WIDTH = 5

def set_pulse_width(channel, pulse_width_us):
    pulse_width_us = max(MIN_PULSE_WIDTH, min(MAX_PULSE_WIDTH, pulse_width_us))
    pca.channels[channel].duty_cycle = int(pulse_width_us * 65536/20000)
    
def set_servo_position(channel, angle):
    angle = max(0, min(180, angle))
    pulse_width_us = NEUTRAL_POSITION + (angle / 360) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    set_pulse_width(channel, pulse_width_us)
    
def set_led_brightness(brightness):
    set_pulse_width(2, NEUTRAL_POSITION - 1000 * brightness)

def set_led_off():
    set_pulse_width(2, 0)
    
try:
    while True:
        set_servo_position(0, 0)
        set_servo_position(1, 0)
        set_led_brightness(1)
        time.sleep(1)
        
        set_servo_position(0, 45)
        set_servo_position(1, 45)
        set_led_brightness(0.5)
        time.sleep(1)
        
        set_servo_position(0, 90)
        set_servo_position(1, 90)
        set_led_brightness(0.25)
        time.sleep(1)
        
        set_servo_position(0, 180)
        set_servo_position(1, 180)
        set_led_brightness(0)
        time.sleep(1)
        
except KeyboardInterrupt:
    set_servo_position(0, 45)
    set_servo_position(1, 45)
    set_led_off()
        
        
        
        