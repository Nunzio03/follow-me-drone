import time


class MeasurementAssistant:
    def __init__(self, name, tolerance, t_threshold):
        localtime = time.localtime()
        self.filename = name+str(localtime.tm_hour)+str(localtime.tm_min)+".csv"
        self.name = name
        self.tolerance = tolerance
        self.t_threshold = t_threshold
        self.x, self.y = list(), list()
        self.in_position = False
        self.n_overshoots = 0
        self.last_sign = None
        self.start_round_time = time.time()

    def write_measurement(self, value):
        t = time.time()
        self.x.append(t)
        self.y.append(value)
        f = open("measurement logs/"+self.filename, "a")
        f.write(str(t)+";"+ str(value)+ '\n')
        f.close()

        self.count_oveshoots(value)

        self.arrived_routine(value, t)

    def new_round(self, parameters):
        f = open("measurement logs/" + self.filename, "a")
        f.write("______________________NEW ROUND________________________"+'\n')
        f.write(str(parameters) + '\n')
        f.close()
        self.x, self.y = list(), list()
        self.start_round_time = time.time()

    def compute_abs_mean(self):

        s = 0
        if len(self.y) > 0:
            for n in self.y:
                s += abs(n)
            s = s/len(self.y)
        return s

    def compute_abs_sum(self):
        s = 0
        if len(self.y) > 0:
            for n in self.y:
                s += abs(n)
        return s

    def is_in_setpoint(self):
        if self.in_position:
            print("elapsed:", time.time() - self.start_arrived_time)
            if time.time() - self.start_arrived_time >= self.t_threshold:
                self.in_position = False
                return True
        else:
            return False

    def arrived_routine(self, value, t):
        if abs(value) < self.tolerance:
            if not self.in_position:
                self.start_arrived_time = t
                self.in_position = True

        else:
            self.in_position = False

    def count_oveshoots(self, value):
        sign = value/abs(value)
        if sign != self.last_sign:
            self.n_overshoots +=1
        self.last_sign = sign

    def fitness(self):
        elapsed_time = time.time() - self.start_round_time
