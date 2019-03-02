import time
import datetime
from motor import Motor
from sensor.lsm6ds3 import LSM6DS3 as MPU6050
from RotaryEncoder import  RotaryEncoder as Encoder
from pid import PID
import settings


class Robot:
    def __init__(self):
        self.motor_left = Motor(settings.PINS['motor']['left'])
        self.motor_right = Motor(settings.PINS['motor']['right'])
        self.encoder_left = Encoder.Worker(settings.PINS['encoder']['left'])
        self.encoder_right = Encoder.Worker(settings.PINS['encoder']['right'])
        self.mpu = MPU6050(0x6b)
        self.pid = PID()
        self.encoder_left.start()
        self.encoder_right.start()

    def run(self):
        pwm_velocity = pwm_turn = 0
        counter_remote = counter_velocity = counter_turn = 0
        while True:
            begin_time = datetime.datetime.now()
            self.mpu.update()
            pwm_balance = self.pid.get_balance_pwm(settings.pid_params['balance'],
                                            self.mpu.balance_angle + 1,
                                            self.mpu.balance_gyro)


            if counter_velocity >= 2:
                pwm_velocity = self.pid.get_velocity_pwm(settings.pid_params['velocity'],
                                                             self.encoder_left.speed,
                                                             -self.encoder_right.speed)
                print(self.encoder_left.speed, self.encoder_right.speed)
                self.encoder_left.speed = 0
                self.encoder_right.speed = 0
                counter_velocity = 0
            counter_velocity += 1

            if self.mpu.balance_angle > 40 or self.mpu.balance_angle < -40:
                    self.pause()
                    continue

            pwm_left = int(pwm_balance + pwm_velocity + pwm_turn)
            pwm_right = int(pwm_balance + pwm_velocity - pwm_turn)
            self.motor_left.set_pwm(pwm_left)
            self.motor_right.set_pwm(pwm_right)

            dt = (datetime.datetime.now() - begin_time).total_seconds()
            dt = (0.005 - dt) if dt < 0.005 else 0
            time.sleep(dt)

    def pause(self):
        self.motor_left.stop()
        self.motor_right.stop()

        self.encoder_left.reset()
        self.encoder_right.reset()

        self.pid.clean()

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()
        self.encoder_left.cancel()
        self.encoder_right.cancel()


if __name__ == '__main__':
    my_robot = Robot()
    try:
        my_robot.run()
    except:
        my_robot.stop()
