# import numpy as np
import cv2
from cv2 import aruco
# import matplotlib.pyplot as plt
# import matplotlib as mpl
video_capture = cv2.VideoCapture(0)
while True:
    ret, frame = video_capture.read()

    cv2.imshow("video", frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    width = 2400
    dim = (width, int(width*3/4))
    frame_markers = cv2.resize(frame_markers, dim, interpolation=cv2.INTER_AREA)


    cv2.imshow("markers", frame_markers)


    if cv2.waitKey(1) & 0xFF == ord('q'):  # quit from script
        break

