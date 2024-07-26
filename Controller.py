# sudo raspi-config
# Setting Interfacing Option > SPI > Enable
# sudo reboot

# sudo apt-get update
# sudo apt-get install python3-spidev
# sudo apt-get install python3-rpi.gpio

# MOSI (Pin 19, GPIO 10) to SDI (Pin 4) on MCP4822
# SCK (Pin 23, GPIO 11) to SCK (Pin 3) on MCP4822
# CS (Pin 24, GPIO 8) to CS (Pin 2) on MCP4822
# GND (Pin 25, Ground) to GND (Pin 1) on MCP4822
# 3.3V (Pin 17) to VDD (Pin 5) on MCP4822
import RPi.GPIO as GPIO
import spidev
import time
import math
import numpy as np


class Controller:
  def __init__(self, width, height, laserIsOn):
    self.width = width
    self.height = height
    self.cur_pos = (0, 0)
    self.laserIsOn = laserIsOn

    self.x_channel = 0
    self.y_channel = 1

    self.laser_pin = 18
    self.pwm_freq = 1000

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.laser_pin, GPIO.OUT)    
    self.laser_pwm = GPIO.PWM(self.laser_pin, self.pwm_freq)
    self.laser_pwm.start(0)

    self.spi = spidev.SpiDev()
    self.spi.open(0, 0)
    self.spi.max_speed_hz = 1000000
  
  def laser_on_100(self):
    self.laser_pwm.ChangDutyCycle(100)
  
  def laser_on_50(self):
    self.laser_pwm.ChangeDutyCycle(50)

  def laser_on_25(self):
    self.laser_pwm.ChangeDutyCycle(25)

  def laser_on_10(self):
    self.laser_pwm.ChangeDutyCycle(10)

  def laser_off(self):
    self.laser_pwm.ChangeDutyCycle(0)

  def send_spi_voltage(self, channel, voltage): # Voltage must be 12-bit between 0 and 4095
    if voltage < 0 or voltage > 4095:
      raise ValueError("12-bit Voltage value must be between 0 and 4095")
    command = 0x3000
    if channel == self.y_channel:
      command |= 0x8000
    command |= (voltage & 0x0FFF)
    high_byte = (command >> 8) & 0xFF
    low_byte = command & 0xFF

    self.spi.xfer([high_byte, low_byte])
  
  def updateLaser(self):
    if self.laserIsOn == False:
      self.laser_off()
  
  def pos_to_12bit_voltage(self, pos):
    x_voltage_value = int(pos[0] * 4095 / self.width)
    y_voltage_value = int(pos[1] * 4095 / self.height)
    return (x_voltage_value, y_voltage_value)
  
  def moveGalvo(self, pos):
    x, y = self.pos_to_12bit_voltage(pos)
    self.send_spi_voltage(self.x_channel, x)
    self.send_spi_voltage(self.y_channel, y)
    self.cur_pos = pos

  def move_circular(self, bbox):
    steps = 16
    radius = min(abs(bbox[0] - bbox[2]), abs(bbox[1] - abs[3])) // 2
    for step in range(steps):
      angle = 2 * math.pi * step / steps
      x = radius * math.cos(angle)
      y = radius * math.sin(angle)
      self.moveGalvo(x, y)

  def is_path_clear(self, start, end, obstacles):
    for (x1, y1, x2, y2) in obstacles:
      if self.intersect(start, end, (x1, y1, x2, y2)):
        return False
    return True
  
  def intersect(self, p1, p2, bbox):
    x1, y1, x2, y2 = bbox
    p1x, p1y = p1
    p2x, p2y = p2

    def ccw(A, B, C):
      return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def line_intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
    
    rect_corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    for i in range(len(rect_corners)):
        next_i = (i + 1) % len(rect_corners)
        if line_intersect(p1, p2, rect_corners[i], rect_corners[next_i]):
            return True
    return False
  
  def move_around_obstacle(self, start, destination, obstacles):
    # Simple heuristic: move to a point near the obstacle and then continue to destination
    # This is a naive approach and may not be optimal or efficient
    dx = destination[0] - start[0]
    dy = destination[1] - start[1]
    step_size = 10

    # Move in the direction of the destination until the path is clear
    for step in range(0, int(np.sqrt(dx**2 + dy**2)), step_size):
        intermediate_point = (start[0] + (dx / step_size) * step, start[1] + (dy / step_size) * step)
        if not self.is_path_clear(start, intermediate_point, obstacles):
            # If the path is blocked, adjust path to move around obstacles
            # Move to a point offset from the original line
            offset = (step_size // 2, step_size // 2)
            alternative_point = (intermediate_point[0] + offset[0], intermediate_point[1] + offset[1])
            if self.is_path_clear(start, alternative_point, obstacles):
                return alternative_point
            alternative_point = (intermediate_point[0] - offset[0], intermediate_point[1] - offset[1])
            if self.is_path_clear(start, alternative_point, obstacles):
                return alternative_point
    return destination

  def greedy_pathfinding(self, start, destinations, obstacles):
      path = [start]
      self.cur_pos = start

      for dest in destinations:
          dest_pos = ((dest[2] + dest[0]) // 2, (dest[3] + dest[1]) // 2)
          if self.is_path_clear(self.cur_pos, dest_pos, obstacles):
              path.append(dest_pos)
              self.cur_pos = dest_pos

              steps = 16
              rad = min( (dest[2] - dest[0]), (dest[3] - dest[1])) // 2
              for step in range(steps):
                angle = 2 * math.pi * step / steps
                x = rad * math.cos(angle)
                y = rad * math.sin(angle)
                path.append((x, y))
                self.cur_pos = (x, y)

          else:
              print(f"Path to {dest_pos} blocked, finding an alternative path...")
              new_dest = self.move_around_obstacle(self.cur_pos, dest_pos, obstacles)
              path.append(new_dest)
              self.cur_pos = new_dest
      
      return path

# start = (0, 0)
# destination will be sort by x
# destinations = [(50, 50), (600, 500), (200, 300)]
#  
# path = []
# obstacles = [] 
# obstacles = [item for item in something if datatype == person]
# if self.laserIsOn == True:
#   if destination != []:
#     if obstacles != []:
#       self.laser_on_10() 
#       path = greedy_pathfinding(start, destinations, obstacles)
#       for p in path:
#         moveGalvo(path)
#       moveGalvo(0, 0)
#     else:
#       self.laser_on_100() 
#       for dest in destinations:
#         dest_pos = ((dest[2] + dest[0]) // 2, (dest[3] + dest[1]) // 2)
#         moveGalvo(dest_pos)
#         move_circular(dest_pos)

# window_width = 1920
# window_height = 1080


def test_spi_voltage():
  n = 0
  while n < 10:
    for voltage in range(0, 4096, 128):
      send_spi_voltage(0, voltage)
      send_spi_voltage(1, voltage)
      time.sleep(0.5)
      n += 1
  spi.close()

def test_pwm_laser():
  n = 0
  while n < 10:
    for duty in range(0, 101, 1):
      laser_pwm.ChangeDutyCycle(duty)
      time.sleep(0.1)
    for duty in range (100, -1, -1):
      laser_pwm.ChangeDutyCycle(duty)
      time.sleep(0.1) 
  laser_pwm.stop()
  GPIO.cleanup()
test_spi_voltage()