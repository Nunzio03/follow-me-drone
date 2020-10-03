import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  # dictionary selection
ID = 42  # ID selection

img = aruco.drawMarker(aruco_dict,ID, 150)  # Marker generation

plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
plt.savefig("./marker42.jpg")  # saving a printable copy of the marker
plt.show()
