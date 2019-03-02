import math
import time
import sys

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.I2C as I2C

from sensor.filter import KalmanFilter
import settings

address = 0x6a

class LSM6DS3:

    def __init__(self, address=0x6a, debug=0, pause=0.8):
        self.i2c = I2C.get_i2c_device(address)
        self.address = address
#        dataToWrite = 0
#        dataToWrite |= 0x03
#        dataToWrite |= 0x00
#        dataToWrite |= 0x10
#        self.i2c.write8(0X10, dataToWrite)


        # [FSXL BW_XL 4 bits 0001]
        dataToWrite = 0
        dataToWrite |= 0x01 # 2g [FS_XL 00] sample 200HZ BW_XL [01]
        dataToWrite |= 0x50   #208HZ sample
        self.i2c.write8(0X10, dataToWrite)

        # Settinggyro sensitivity to + / - 2000 deg / 
        dataToWrite = 0
        #00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps and sample it at 208HZ 
        dataToWrite |= 0x5E # 2g [FS_XL 00] sample 200HZ BW_XL [01]
        self.i2c.write8(0X11, dataToWrite)

        time.sleep(1)

        self.gyro = {
            'y': 0,
            'z': 0
        }
        self.accel = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.balance_angle = 0
        self.balance_gyro = 0
        self.turn_gyro = 0

        self.kalman = KalmanFilter()


    def update(self):
        self.gyro['y'] = self.i2c.readS16(0X24)
        print(self.gyro['y'])
        self.gyro['z'] = self.i2c.readS16(0X26)
        self.accel['x'] = self.i2c.readS16(0X28)
        self.accel['z'] = self.i2c.readS16(0X2C)

        if self.gyro['y'] > 32768:
            self.gyro['y'] -= 65536     
        if self.gyro['z'] > 32768:
            self.gyro['z'] -= 65536
        if self.accel['x'] > 32768:
            self.accel['x'] -= 65536
        if self.accel['z'] > 32768:
            self.accel['z'] -= 65536

        self.accel['y'] = int(math.atan2(self.accel['x'], self.accel['z']) * 180 / 3.1415926)   
        self.gyro['y'] /= -16.4    
        self.gyro['z'] /= 16.4

        self.gyro['y'] += settings.gyro_offset['y']
        self.gyro['z'] += settings.gyro_offset['z']
        self.accel['x'] += settings.accel_offset['x']
        self.accel['y'] += settings.accel_offset['y']
        self.accel['z'] += settings.accel_offset['z']

        self.balance_angle = self.kalman.filter(self.accel['y'], self.gyro['y'])   
        self.balance_gyro = self.gyro['y']
        self.turn_gyro = self.gyro['z']
