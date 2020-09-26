import time
import os


class MeasurementAssistant:
    def __init__(self, name, tolerance, t_threshold, parameters0):
        localtime = time.localtime()
        self.folder = name+str(localtime.tm_mday)+str(localtime.tm_mon)+"_"+str(localtime.tm_hour)+str(localtime.tm_min)
        self.name = name
        self.tolerance = tolerance
        self.t_threshold = t_threshold
        self.previousParameters = parameters0

        self.x, self.y = list(), list()
        self.in_position = False
        self.n_overshoots = 0
        self.last_sign = None
        self.start_round_time = time.time()
        self.time_normalization = self.start_round_time
        self.round_counter = 0
        self.filename = "round" + str(self.round_counter)
        self.parameter_fitness_filename = "plot.csv"
        print(os.getcwd())
        try:
            os.chdir("measurement logs")
            print(os.getcwd())
            print("i'm in logs")

        except:
            pass
        try:
            os.mkdir(self.folder)
            print(os.getcwd())
            print("i'm in my folder")
        except:
            pass
        try:
            os.chdir("..")
            print(os.getcwd())
            print("i'm in the project folder")
        except:
            pass

    def write_measurement(self, value):
        t = time.time()
        self.x.append(t)
        self.y.append(value)

        f = open("measurement logs/"+self.folder+"/"+self.filename+".csv", "a")
        f.write(str(t-self.time_normalization)+";"+ str(value)+ '\n')
        f.close()

        self.count_oveshoots(value)

        self.arrived_routine(value, t)

    def new_round(self, parameters, fitness):

        f = open("measurement logs/"+self.folder+"/"+self.parameter_fitness_filename, "a")
        f.write(str(self.previousParameters[0])+","+str(self.previousParameters[1])+","+str(self.previousParameters[2])
                +","+str(fitness)+';'+'\n')
        f = open("measurement logs/"+self.folder+"/"+self.filename+".csv", "a")

        f.write("Fitness:" +str(fitness)+'\n')

        f.close()
        self.round_counter += 1
        self.filename = "round" + str(self.round_counter)
        f = open("measurement logs/" + self.folder + "/" + self.filename + ".csv", "a")
        f.write("______________________NEW ROUND________________________"+'\n')
        f.write(str(parameters) + '\n')
        f.close()
        self.x, self.y = list(), list()
        self.start_round_time = time.time()
        self.previousParameters = parameters

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
        if value != 0:
            sign = value/abs(value)
            if sign != self.last_sign:
                self.n_overshoots +=1
            self.last_sign = sign

    def fitness(self):
        elapsed_time = time.time() - self.start_round_time
        a = 0.7*elapsed_time+0.2*self.n_overshoots+0.1*self.compute_abs_sum()
        return 8000/(80+a)
