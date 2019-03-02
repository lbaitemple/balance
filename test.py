from motor import Motor
import settings
import time
from RotaryEncoder import  RotaryEncoder as Encoder
from sensor.lsm6ds3 import  LSM6DS3 


motor_left = Motor(settings.PINS['motor']['left'])
motor_right = Motor(settings.PINS['motor']['right'])
encoder_left = Encoder.Worker(settings.PINS['encoder']['left'])
encoder_left.start()
encoder_right = Encoder.Worker(settings.PINS['encoder']['right'])
encoder_right.start()
mpu = LSM6DS3(0x6b)


while True:
#    motor_left.set_pwm(200)
#    motor_right.set_pwm(200)
    time.sleep(1)
    mpu.update()
    print("working", encoder_left.speed, encoder_right.speed)
    print (mpu.balance_angle, mpu.balance_gyro)

#    motor_left.stop()
#    motor_right.stop()
#    time.sleep(2)
