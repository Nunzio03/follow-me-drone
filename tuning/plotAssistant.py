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

    def plot(self):

        plt.plot(self.t, self.x, color="red", label="x axis")
        plt.plot(self.t, self.y, color="yellow", label="y_axis")
        plt.plot(self.t, self.z, color="blue", label="z_axis")
        plt.plot(self.t, self.ref, color="black", label="reference")
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
