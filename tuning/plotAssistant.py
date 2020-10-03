import matplotlib.pyplot as plt
from drawnow import *
import threading
import time
import math


class PlotMeasurements (threading.Thread):
    def __init__(self, nome, t, x, y, z, ref):
        threading.Thread.__init__(self)
        self.t = t
        self.x = x
        self.y = y
        self.z = z
        self.ref = ref
        self.nome = nome
        plt.ion()
        font = {'family': 'normal',
                'weight': 'bold',
                'size': 12}

        plt.rc('font', **font)


    def plot(self):

        plt.plot(self.t, self.x, color=(1,0.1,0.1), label="x axis")
        plt.plot(self.t, self.y, color=(0.1,0.5,0.2,1), label="y_axis")
        plt.plot(self.t, self.z, color="blue", label="z_axis")
        plt.plot(self.t, self.ref, color="black", label="reference")
        plt.xlabel("Time [s]")
        plt.ylabel("Error [cm]")
        plt.legend(loc='best')

    def run(self):

        while True:

            drawnow(self.plot)


class PlotAssistant:
    def __init__(self):
        self.t, self.x, self.y, self.z, self.ref = list(), list(), list(), list(), list()
        self.threadPlot = PlotMeasurements("plot measurements", self.t, self.x, self.y, self.z, self.ref)
        self.threadPlot.start()
        self.start = time.time()

    def insertMeasurement(self, ex, ey, ez):
        t = time.time() - self.start
        self.t.append(t)
        self.x.append(ex)
        self.y.append(ey)
        self.z.append(ez)
        self.ref.append(0)
