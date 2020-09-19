from controllers.PID_controller import PIDController


class PIDTuner:
    def __init__(self, PIDx: PIDController, PIDy: PIDController, PIDz: PIDController):
        self.PIDx, self.PIDy, self.PIDz = PIDx, PIDy, PIDz
        self.current_pid = PIDx
        self.current_parameter = 'p'

    def get_pid(self, key):
        if key & 0xFF == ord("x"):
            self.current_pid = self.PIDx

        elif key & 0xFF == ord("y"):
            self.current_pid = self.PIDy

        elif key & 0xFF == ord("z"):
            self.current_pid = self.PIDz
        return self.current_pid

    def get_parameter(self, key):
        if key & 0xFF == ord("p"):

            self.current_parameter = 'p'

        elif key & 0xFF == ord("i"):
            self.current_parameter = 'i'

        elif key & 0xFF == ord("d"):
            self.current_parameter = 'd'
        return self.current_parameter

    def tune(self, key):
        if key & 0xFF == ord("0"):
            self.current_pid.set_gain(self.current_parameter, 0)
        elif key & 0xFF == ord("8"):
            self.current_pid.increase_gain(self.current_parameter, 0.01)
        elif key & 0xFF == ord("2"):
            self.current_pid.increase_gain(self.current_parameter, -0.01)
