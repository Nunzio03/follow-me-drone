class BangBangController:
    def __init__(self, setpoint, tolerance, speed):
        self.setpoint = setpoint
        self.tolerance = tolerance
        self.speed = speed

    def compute_action(self, error):
        if error> self.tolerance:
            output = -self.speed
            print("vai a sx")
        elif error < - self.tolerance:
            print("vai a dx")
            output = self.speed
        else:
            output = 0
            print("ok x")
        return output
