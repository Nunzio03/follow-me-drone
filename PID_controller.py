import time


class PIDController:
    def __init__(self, identifier):
        self.kp, self.ki, self.kd = 0, 0, 0
        self.identifier = identifier
        self.previous_error = 0
        self.integral = 0
        self.start_time = time.time()

    def increase_gain(self, parameter, value):
        if parameter == "p":
            self.kp += value
        elif parameter == "i":
            self.ki += value
        elif parameter == "d":
            self.kd += value

    def set_gain(self, parameter, value):
        if parameter == "p":
            self.kp = value
        elif parameter == "i":
            self.ki = value
        elif parameter == "d":
            self.kd = value

    def compute_action(self, error):
        delay_pid = time.time() - self.start_time
        self.start_time = time.time()

        self.integral = self.integral + error * delay_pid

        if self.integral > 100:
            self.integral = 100
        elif self.integral < -100:
            self.integral = -100

        derivative = (error - self.previous_error) / delay_pid

        output_x = self.kp * error + self.ki * self.integral + self.kd * derivative

        # outx:setpoint = mapx : 100
        self.previous_error = error
        # bounded_output = int(100 * output_x / set_point)
        return output_x

    def __str__(self):
        return f'{self.identifier} :[ P:{round(self.kp, 2)}, I:{round(self.ki,2)}, D:{round(self.kd,2)} ]'
