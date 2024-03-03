from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time

# i2c = busio.I2C(SCL, SDA)
# pca = PCA9685(i2c)
# pca.frequency = 50

# minimum_pulse = 500
# maximum_pulse = 2500

# servo_x = servo.Servo(pca.channels[0], min_pulse = minimum_pulse, max_pulse = maximum_pulse)
# servo_y = servo.Servo(pca.channels[1], min_pulse = minimum_pulse, max_pulse = maximum_pulse)
# laser_ch = pca.channels[2]

# duty_cycle = 0
# try:
#     while True:
#         laser_ch.duty_cycle = duty_cycle * 65535 // 100
#         servo_x.angle = duty_cycle * 180 // 100
#         servo_y.angle = duty_cycle * 180 // 100
#         time.sleep(1)
#         duty_cycle += 10
        
#         if duty_cycle > 100:
#             print("reset")
#             duty_cycle = 0        
#         print(duty_cycle)
# except KeyboardInterrupt:
#     laser_ch.duty_cycle = 0
#     servo_x.angle = 45
#     servo_y.angle = 45
#     #pca.deinit()
    
class HardwareController:
    def __init__(self, width, height, fov_width, fov_height, init_angle_x, init_angle_y):
        self.width = width                                              # Set Video Resolusion Width
        self.height = height                                            # Set Video Resolusion Height
        self.fov_width = fov_width                                      # Set Camera's Field of View Width
        self.fov_height = fov_height                                    # Set Camera's Field of View Height
        self.init_angle_x = init_angle_x                                # Set Initial Motor Angle X
        self.init_angle_y = init_angle_y                                # Set Initial Motor Angle Y
        self.min_angle_x = self.init_angle - (self.fov_width / 2)       # Minimum Rotate Angle X
        self.min_angle_y = self.init_angle - (self.fov_height / 2)      # Minimum Rotate Angle Y
        self.max_angle_x = self.min_angle_x + self.fov_width            # Maximum Rotate Angle X
        self.max_angle_y = self.min_angle_y + self.fov_height           # Maximum Rotate Angle Y

        self.current_position = (960, 480)
        self.laserIsOn = False

        self.servo_x_channel = 0                                        # Set Servo X-axis Channel
        self.servo_y_channel = 1                                        # Set Servo Y-axis Channel
        self.laser_channel = 2                                          # Set Laser Channel

        self.i2c = busio.I2C(SCL, SDA)                                  # Initialize I2C
        self.pca = PCA9685(self.i2c)                                         # Initialize PCA9685
        self.pca.frequency = 50                                         # Set PCA9685 Frequency

        self.minimum_pulse = 500                                        # Set Minimun Pulse to Specific Servo
        self.maximum_pulse = 2500                                       # Set Maximum Pulse to Specific Servo

        self.servo_x = servo.Servo(self.pca.channels[self.servo_x_channel], min_pulse = self.minimum_pulse, max_pulse = self.maximum_pulse)     # Initialize Servo X
        self.servo_y = servo.Servo(self.pca.channels[self.servo_y_channel], min_pulse = self.minimum_pulse, max_pulse = self.maximum_pulse)     # Initialize Servo Y
        self.laser = self.pca.channels[self.laser_channel]                   # Initialize Laser

        self.moveServo(current_position)
        self.laser_off()

    def laser_off(self):
        self.laser.duty_cycle = 0
        print("Turn Off Laser")
        return False

    def laser_on_100(self, laserIsOn):
        if laserIsOn == True:
            self.laser.duty_cycle = 65535                                   # To Adjust PWM 0 to 100 Input Would be 0 to 65535 > EXAMPLE: duty_cycle * 65535 // 100
            print("Set Laser Power to 100")
    
    def laser_on_50(self, laserIsOn):
        if laserIsOn == True:
            self.laser.duty_cycle = 32767
            print("Set Laser Power to 50")

    def laser_on_25(self, laserIsOn):
        if laserIsOn == True:
            self.laser.duty_cycle = 16383
            print("Set Laser Power to 25")

    def updateLaser(self):
        if self.laserIsOn == False:
            self.laser_off()
        else:
            self.laser_on_100()

    def pos2angle(self, position):                                      # Convert Position into Angle
        angle_x = position[0] * self.fov_width / self.width
        angel_y = position[1] * self.fov_height / self.height
        return angle_x, angel_y
    
    def moveServo(self, position):                                      # Control Servo to Move
        angle_x, angle_y = self.pos2angle(position)
        self.servo_x.angle = angle_x + self.min_angle_x
        self.servo_y.angle = angle_y + self.min_angle_y

    def distance(self, p1, p2):
        return (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2

    def move_circle(self, pigeon_pos):
        top = pigeon_pos["Top"]
        bottom = pigeon_pos["Bottom"]
        left = pigeon_pos["Left"]
        right = pigeon_pos["Right"]

        radius = min(abs(top - bottom), abs(left - right))
        mid = (left + (right - left) / 2, top + (bottom -  top) / 2)

        position = []
        
        for i in range(mid[1] - radius, mid[1] + radius + 1, 2):
            for j in range(mid[0] - radius, mid[0] + radius + 1, 2):
                if (j - mid[0]) ** 2 + (i - j) ** 2 <= radius ** 2:
                    position.append((j, i))
                    self.moveServo((j, i))

    def position_adjust(self, curr_pos, dest_pos, data):                    # Calculate Position that would be Move through Human
        person_data = [item for item in data if item["Class"] == "person"]
        curr_x, curr_y = curr_pos
        dest_x, dest_y = dest_pos

        avoid_pos = []
        x_avoid_top = []
        x_avoid_bottom = []
        y_avoid_left = []
        y_avoid_right = []

        passHuman = False

        # Calculate m and c in equation y = mx + c
        if curr_x == dest_x:
            x_avoid_top.append(curr_x)
            x_avoid_bottom.append(curr_x)
            for item in person_data:
                y_avoid_left.append(item["Top"])
                y_avoid_right.append(item["Bottom"])
        elif curr_y == dest_y:
            y_avoid_left.append(curr_y)
            y_avoid_right.append(curr_y)
            for item in person_data:
                x_avoid_top.append(item["Left"])
                x_avoid_bottom.append(item["Right"])
        else:
            m = (dest_y - curr_y) / (dest_x - curr_x)
            c = dest_y - (m * dest_x)
            for item in person_data:
                x_avoid_top.append((item["Top"] - c) / m)
                x_avoid_bottom.append((item["Bottom"] - c) / m)
                y_avoid_left.append((m * item["Left"]) + c)
                y_avoid_right.append((m * item["Right"]) + c)
        # Check Avoidance Position
        for item in person_data:
            for x in x_avoid_top:
                if item["Top"] >= x >= item["Bottom"]:                           # Check line person_y_1
                    avoid_pos.append((x, item["Top"]))
            for x in x_avoid_bottom:
                if item["Top"] >= x >= item["Bottom"]:
                    avoid_pos.append((x, item["Bottom"]))
            for y in y_avoid_left:
                if item["Left"] >= y >= item["Right"]:                           # Check line person_x_1
                    avoid_pos.append((item["Left"], y))
            for y in y_avoid_right:
                if item["Left"] >= y >= item["Right"]:
                    avoid_pos.append((item["Right"], y))
        
        sorted_avoid = sorted(avoid_pos, key=lambda point: self.distance(curr_pos, point))
        next_pos = sorted_avoid[0]
        mid_x = curr_x + ((next_pos[0] - curr_x) / 2)
        mid_y = curr_y + ((next_pos[1] - curr_y) / 2)

        for item in person_data:
            if item["Left"] >= mid_x >= item["Right"] and item["Top"] >= mid_y >= item["Bottom"]:
                passHuman = True
                break
        if passHuman == True:
            self.laser_off()
        else:
            self.laser_on_25()
        self.moveServo((next_pos[0], next_pos[1]))

current_position = (1920 / 2, 1080 / 2)

data_frame = []
