from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import math

class HardwareController:
    def __init__(self, width, height, fov_width, fov_height, init_angle_x, init_angle_y):
        self.width = width                                              # Set Video Resolusion Width
        self.height = height                                            # Set Video Resolusion Height
        self.fov_width = fov_width                                      # Set Camera's Field of View Width
        self.fov_height = fov_height                                    # Set Camera's Field of View Height
        self.init_angle_x = init_angle_x                                # Set Initial Motor Angle X :Calibrating 47.7
        self.init_angle_y = init_angle_y                                # Set Initial Motor Angle Y :Calibrating 30.5
        self.min_angle_x = self.init_angle_x - (self.fov_width / 2)       # Minimum Rotate Angle X
        self.min_angle_y = self.init_angle_y - (self.fov_height / 2)      # Minimum Rotate Angle Y
        self.max_angle_x = self.min_angle_x + self.fov_width            # Maximum Rotate Angle X
        self.max_angle_y = self.min_angle_y + self.fov_height           # Maximum Rotate Angle Y

        self.init_position = (960, 540)
        self.current_position = self.init_position
        self.laserIsOn = True

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

        #print("init hwctl")
        self.moveServo(self.init_position)
        #self.laserIsOn = True
        #self.laser_on_25()
        #time.sleep(10)
        #self.laserIsOn = False
        self.laser_off()

        self.count = 0

    def laser_off(self):
        self.laser.duty_cycle = 0
        #print("Turn Off Laser")
        return False

    def laser_on_100(self):
        if self.laserIsOn == True:
            self.laser.duty_cycle = 65535                                   # To Adjust PWM 0 to 100 Input Would be 0 to 65535 > EXAMPLE: duty_cycle * 65535 // 100
            #print("Set Laser Power to 100")
        else:
            self.laser_off()
    
    def laser_on_50(self):
        if self.laserIsOn == True:
            self.laser.duty_cycle = 32767
            #print("Set Laser Power to 50")
        else:
            self.laser_off()

    def laser_on_25(self):
        if self.laserIsOn == True:
            self.laser.duty_cycle = 6553  #16383
            print("Set Laser Power to 25")
        else:
            self.laser_off()

    def updateLaser(self):
        if self.laserIsOn == False:
            self.laser_off()
        else:
            self.laser_on_100()

    def pos2angle(self, position):                                      # Convert Position into Angle
        angle_x = (position[0] * self.fov_width) / self.width
        angle_y = (position[1] * self.fov_height) / self.height
        return angle_x, angle_y
    
    def moveServo(self, position):                                      # Control Servo to Move
        angle_x, angle_y = self.pos2angle(position)
        self.servo_x.angle = angle_x + self.min_angle_x
        self.servo_y.angle = self.max_angle_y - angle_y
        self.current_position = position
	

    def distance(self, p1, p2):
        return (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2

    def move_circle(self, pigeon_info):
        top = pigeon_info.top
        bottom = pigeon_info.bottom
        left = pigeon_info.left
        right = pigeon_info.right

        radius = min(abs(top - bottom), abs(left - right)) // 2
        mid = (left + right // 2, top + bottom // 2)

        position = []
        #position.append((mid[0], mid[1] + radius))
        position.append((mid[0] + radius, mid[1] + radius))
        #position.append((mid[0] + radius, mid[1]))
        position.append((mid[0] + radius, mid[1] - radius))
        #position.append((mid[0], mid[1] - radius))
        position.append((mid[0] - radius, mid[1] - radius))
        #position.append((mid[0] - radius, mid[1]))
        position.append((mid[0] - radius, mid[1] + radius))
        #position.append((mid[0], mid[1] + radius))
        for pos in position:
            #print("CIRCCLEEEEE")
            self.moveServo(pos)
            time.sleep(0.1)


    def position_adjust(self, dest_pos, data, pigeon_info):                    # Calculate Position that would be Move through Human   #Change curr_pos to self.current_position, Delete 1 argument curr_pos
        self.count += 1
        if self.count < 5:
            return
        else:
            self.count = 0
        #if abs(self.current_position[0] - dest_pos[0]) < 10 and abs(self.current_position[1] - dest_pos[1]) < 10:
            #print("CIRCLE")
            #self.move_circle(pigeon_info)
       
            #return 
        if data != []:
            person_data = [item for item in data if item.class_name == "person"]
            if person_data == []:
                self.laser_on_100()
                self.moveServo(dest_pos)
                return
            else:
                self.laser_on_25()
            curr_x, curr_y = self.current_position   #Change curr_pos to self.current_position
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
                     y_avoid_left.append(item.top)
                     y_avoid_right.append(item.bottom)
            elif curr_y == dest_y:
               	    y_avoid_left.append(curr_y)
                    y_avoid_right.append(curr_y)
                    for item in person_data:
                        x_avoid_top.append(item.left)
                        x_avoid_bottom.append(item.right)
            else:
                m = (dest_y - curr_y) / (dest_x - curr_x)
                c = dest_y - (m * dest_x)
                for item in person_data:
                      x_avoid_top.append((item.top - c) / m)
                      x_avoid_bottom.append((item.bottom - c) / m)
                      y_avoid_left.append((m * item.left) + c)
                      y_avoid_right.append((m * item.right) + c)
		# Check Avoidance Position
                for item in person_data:
                      for x in x_avoid_top:
                           if item.top >= x >= item.bottom:                           # Check line person_y_1
                               avoid_pos.append((x, item.top))
                      for x in x_avoid_bottom:
                           if item.top >= x >= item.bottom:
                               avoid_pos.append((x, item.bottom))
                      for y in y_avoid_left:
                           if item.left >= y >= item.right:                           # Check line person_x_1
                               avoid_pos.append((item.left, y))
                      for y in y_avoid_right:
                           if item.left >= y >= item.right:
                               avoid_pos.append((item.right, y))

                avoid_pos.append(dest_pos)
                
                if avoid_pos != []:
                    #print("avoid more than 0")
                    sorted_avoid = sorted(avoid_pos, key=lambda point: self.distance(self.current_position, point))   #Change curr_pos to self.current_position
                    next_pos = sorted_avoid[0]
                else:
                    next_pos = []
                    return
                for pos in sorted_avoid:
                    if pos[0] == dest_x and pos[1] == dest_y:
                        if item.left >= dest_x >= item.right and item.top >= dest_y >= item.bottom:
                            return
                        else:
                            self.moveServo(pos)
                            return
                    mid_x = (self.current_position[0] + pos[0]) / 2
                    mid_y = (self.current_position[1] + pos[1]) / 2
                    for item in person_data:
                        if item.left >= mid_x >= item.right and item.top >= mid_y >= item.bottom:
                            passHuman = True
                            break

                    if passHuman == True:
                        self.laser_off()
                    else:
                        self.laser_on_25()
                    self.moveServo(pos)
                    #self.position_adjust(dest_pos, data, pigeon_info)  #Change next_pos to self.current_position
                else:
                    self.moveServo(dest_pos)
                    return
#current_position = (1920 / 2, 1080 / 2)
#try:
#	hardware_controller = HardwareController(1920, 1080, 106, 48, 46, 31)
#	hardware_controller.laserIsOn = True
#	
#	while True:
#                x = int(input("pos x: "))
#                y = int(input("pos y: "))
#                hardware_controller.moveServo((x, y))
#                hardware_controller.laser_on_25()
#                time.sleep(0.001)
		#hardware_controller.moveServo((960, 540))
#		
#except KeyboardInterrupt:
#	hardware_controller.laser_off()
#	hardware_controller.moveServo((0, 0))
