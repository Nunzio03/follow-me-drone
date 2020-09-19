import time


def same_sign(a, b):
    if a == 0 or b == 0:
        return True
    elif a/abs(a) == b/abs(b):
        return True
    else:
        return False


class PIDController:
    def __init__(self, identifier):
        self.kp, self.ki, self.kd = 0, 0, 0
        self.identifier = identifier
        self.previous_error = 0
        self.integral = 0
        self.start_time = time.time()
        self.is_in_saturation = False
        self.previous_output = 0

    def increase_gain(self, parameter, value):
        if parameter == "p":
            self.kp += value
        elif parameter == "i":
            self.ki += value
        elif parameter == "d":
            self.kd += value

    def set_PID_safeopt(self, param):
        self.kp = param[0]
        self.ki = param[1]
        self.kd = param[2]


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

        if not(self.is_in_saturation and same_sign(error, self.previous_output)):
            self.integral = self.integral + error * delay_pid

        derivative = (error - self.previous_error) / delay_pid

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.previous_error = error
        if output > 100:
            output = 100
            self.is_in_saturation = True
        elif output < -100:
            output = -100
            self.is_in_saturation = True
        else:
            self.is_in_saturation = False
            self.previous_output = output
        return output

    def __str__(self):
        return f'{self.identifier} :[ P:{round(self.kp, 2)}, I:{round(self.ki,2)}, D:{round(self.kd,2)} ]'
