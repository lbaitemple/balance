class PID:

    def __init__(self, limit_velocity=1700, limit_turn=150):
        self.velocity = {
            'encoder': 0,
            'encoder_integral': 0
        }
        self.limit_velocity = limit_velocity
        self.limit_turn = limit_turn

    def get_balance_pwm(self, pid_params, angle, gyro):
        return pid_params['k'] * (pid_params['p'] * angle + gyro * pid_params['d'])

    def get_velocity_pwm(self, pid_params, encoder_left, encoder_right):
        encoder_least = encoder_left + encoder_right - 0 
        self.velocity['encoder'] *= 0.7     
        self.velocity['encoder'] += encoder_least * 0.3     
        self.velocity['encoder_integral'] += self.velocity['encoder']   
        self.velocity['encoder_integral'] = self.velocity['encoder_integral'] 
        print(self.velocity['encoder_integral'])
        if self.velocity['encoder_integral'] > 300:
            self.velocity['encoder_integral'] -= 100
        elif self.velocity['encoder_integral'] < -300:
            self.velocity['encoder_integral'] += 100

        if self.velocity['encoder_integral'] > self.limit_velocity:
            self.velocity['encoder_integral'] = self.limit_velocity
        elif self.velocity['encoder_integral'] < -self.limit_velocity:
            self.velocity['encoder_integral'] = -self.limit_velocity

        return self.velocity['encoder'] * pid_params['p'] + self.velocity['encoder_integral'] * pid_params['i']

    def get_turn_pwm(self, pid_params, gyro, turn):
        if turn > self.limit_turn:
            turn = self.limit_turn
        elif turn < -self.limit_turn:
            turn = -self.limit_turn
        return turn * pid_params['p'] + gyro * pid_params['d']

    def clean(self):
        self.velocity['encoder'] = 0
        self.velocity['encoder_integral'] = 0   
