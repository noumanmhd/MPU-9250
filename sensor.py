from mpu9250 import MPU9250
import math
import time

alpha = 0.98
beta = 1 - alpha
dt = 0.01

class Sensor(object):
    def __init__(self):
        self.sensor = MPU9250()
        self.accel = self.sensor.readAccel()
        self.gyro = self.sensor.readGyro()
        self.mag = self.sensor.readMagnet()
        
        self.gx_offset = -4.95
        self.gy_offset = -2.35
        self.gz_offset = -12.22

        self.ax = self.accel["x"]
        self.ay = self.accel["y"]
        self.az = self.accel["z"]

        self.last_x = self.get_x_rotation(self.ax, self.ay, self.az)
        self.last_y = self.get_y_rotation(self.ax, self.ay, self.az)

        self.gx_delta = 0
        self.gy_delta = 0

    def readTemp(self):
        return self.sensor.readTemperature()

    def dist(self, a, b):
        '''Calculate Distance'''
        return math.sqrt((a * a) + (b * b))

    def get_y_rotation(self, x, y, z):
        ''' Y angle calculation'''
        radians = math.atan2(x, self.dist(y, z))
        return math.degrees(radians)

    def get_x_rotation(self, x, y, z):
        '''X angle Calculations'''
        radians = math.atan2(y, self.dist(x, z))
        return math.degrees(radians)

    def readAccel(self):
        self.accel = self.sensor.readAccel()
        ax = self.accel["x"]
        ay = self.accel["y"]
        az = self.accel["z"]
        return ax, ay, az

    def readGyro(self):
        self.gyro = self.sensor.readGyro()
        gx = self.gyro["x"] - self.gx_offset
        gy = self.gyro["y"] - self.gy_offset
        gz = self.gyro["z"] - self.gz_offset
        return gx, gy, gz

    def filtered(self):
        '''Complimentry Filter'''

        time.sleep(dt - 0.005)

        ax, ay, az = self.readAccel()
        gx, gy, gz = self.readGyro()

        self.gx_delta = gx * dt
        self.gy_delta = gy * dt

        rotation_x = self.get_x_rotation(ax, ay, az)
        rotation_y = self.get_y_rotation(ax, ay, az)

        self.last_x = alpha * (self.last_x + self.gx_delta) + (beta * rotation_x)
        self.last_y = alpha * (self.last_y + self.gy_delta) + (beta * rotation_y)
        r = round(self.last_y, 2)
        p = round(self.last_x, 2)
        r += (3.3)
        p -= (3.6)
        return p, r  #Pitch/Roll
