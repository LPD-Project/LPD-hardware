# INSTALL ENVIRONMENT FOR JETSON TO USE GPIO
# sudo apt-get install python3-pip  >> install pip

# ALTERNATIVE WAYS to use Servo Library : sudo pip3 install install adafruit-circuitpython-servokit >> install servo library // This library work with PCA9685

# sudo usermod -aG i2c [username of jetson] >> Set the permission to [username of jetson]

# sudo groupadd -f -r gpio >> add gpio group

# sudo usermod -a -G gpio [username of jetson] >> add [username] to gpio group

# sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/ >> copy gpio.rules to rules.d

# sudo udevadm control --reload-rules && sudo udevadm trigger >> [user development administrator]

# REBOOT JETSON
# sudo reboot now

# sudo i2cdetect -y -r 1 >> Check PCA9685 is connected

# CODE SERVO

from adafruit_servokit import Servokit
# import time
# import cv2 

class ServoController:
    def __init__(self, width, height, fov_width, fov_height, init_angle):
        self.width = width
        self.height = height
        self.fov_width = fov_width
        self.fov_height = fov_height
        self.init_angle = init_angle
        self.start_angle_x = self.init_angle - (self.fov_width / 2)
        self.start_angle_y = self.init_angle - (self.fov_height / 2)

        self.servo_kit = Servokit(channels=16)
        self.servo_x = self.servo_kit.servo[0]
        self.servo_y = self.servo_kit.servo[1]

        self.servo_x.angle = self.init_angle
        self.servo_y.angle = self.init_angle

    def pos2angle(self, position):
        angle_x = position[0] * self.fov_width / self.width
        angle_y = position[1] * self.fov_height / self.height
        return angle_x, angle_y
    
    def isHuman(self, position):
        if position["Class"] == "person":
            return True
        else:
            return False

    def moveServo(self, position):
        angle_x, angle_y = self.pos2angle(position)
        self.servo_x.angle = angle_x + self.start_angle_x
        self.servo_y.angle = angle_y + self.start_angle_y


# camera = cv2.VideoCapture(1)
# fov_width = 62.2
# fov_height = 48.8
# start_angle_x = 45 - (fov_width / 2)
# start_angle_y = 45 - (fov_height / 2)
# view_width = 1920
# view_height = 1080
# view_width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
# view_height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

# servo_kit = Servokit(channels=16) # set channels of servo : PCA9685 have 16 channels

# servo_x = servo_kit.servo[0]
# servo_y = servo_kit.servo[1]

# def init_servo():
#     servo_x.angle = 45
#     servo_y.angle = 45

# # Range of Servo X Should be 66 degree ()

# def pos2angle(position):
#     angleX = position[0] * fov_width / view_width
#     angleY = position[1] * fov_height / view_height
#     return angleX, angleY

# def moveServo(position):
#     angle_x, angle_y = pos2angle(position)
#     servo_x.angle = angle_x + start_angle_x
#     servo_y.angle = angle_y + start_angle_y

# init_servo()
# position = 360, 587 # x-axis Range 0 - 1920 y-axis Range 0 - 1080

# # servo_kit.servo[0].angle = 90 # Rotate the servo[0] for 90 degree >> EXAMPLE

# moveServo(position)