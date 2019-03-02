import wiringpi
from settings import wiringport

class Motor:

    def __init__(self, motorpins, pwm_range=255):
        self.pi = wiringpi.GPIO(wiringpi.GPIO.WPI_MODE_PINS)
        self.pins = motorpins
        self.speed = 0
        self.pwm_range = pwm_range
        for pin in motorpins:
            print (wiringport[pin])
            self.pi.softPwmCreate(wiringport[pin], 0, pwm_range)

    def set_pwm_pin(self, pin, pwm):
        self.pi.softPwmWrite(wiringport[pin], pwm)

    def set_pwm(self, pwm):
        pwm = int(-self.pwm_range if pwm < -self.pwm_range else self.pwm_range if pwm > self.pwm_range else pwm)

        if pwm < 0:
            self.set_pwm_pin(self.pins[0], -pwm)
            self.set_pwm_pin(self.pins[1], 0)
        else:
            self.set_pwm_pin(self.pins[0], 0)
            self.set_pwm_pin(self.pins[1], pwm)

    def stop(self):
        for pin in self.pins:
            self.set_pwm_pin(pin, 0)
