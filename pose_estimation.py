import numpy as np
import cv2, PIL, os
from cv2 import aruco
import matplotlib.pyplot as plt
from tellopy import Tello
import math


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)

###### identification

#video_capture = cv2.VideoCapture(0)
drone = Tello()
drone.connect()
drone.streamon()


pc_mtx = np.array([[1.73223258e+03, 0.00000000e+00, 1.27300230e+03],
       [0.00000000e+00, 1.73223258e+03, 1.03042217e+03],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

pc_dist = np.array([[-3.89477169e+00],
       [-9.72135202e-02],
       [ 1.04299558e-02],
       [ 8.40170760e-05],
       [-2.17736443e+00],
       [-3.89139953e+00],
       [-1.90794581e-01],
       [-1.85298591e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00],
       [ 0.00000000e+00]])

drone_mtx = np.array([[1.74213359e+03, 0.00000000e+00, 1.27150514e+03],
 [0.00000000e+00, 1.74213359e+03, 1.02516982e+03],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

drone_dist = np.array([[-1.69684883e+00],
 [-6.85717812e+00],
 [ 9.93624014e-03],
 [ 6.20144084e-04],
 [-1.18739065e+01],
 [-1.69460711e+00],
 [-6.99110211e+00],
 [-1.13633464e+01],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00]])


mtx, dist = drone_mtx, drone_dist
while True:

    #ret, frame = video_capture.read()
    frame = drone.get_frame_read().frame

    #frame = cv2.undistort(src = frame, cameraMatrix = mtx, distCoeffs = dist)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters)
    # SUB PIXEL DETECTION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    for corner in corners:
        cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    size_of_marker =  0.15 # side lenght of the marker in meter
    rvecs,tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

    length_of_axis = 0.1
    imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


    if tvecs is not None:
        for i in range(len(tvecs)):
            imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)
    try:
        x = (corners[0][0][0][0] + corners[0][0][2][0])/2
        y = (corners[0][0][0][1] + corners[0][0][2][1])/2
        square_side_dimension = math.sqrt(math.pow(corners[0][0][3][1] - corners[0][0][0][1], 2) +
                                          math.pow(corners[0][0][3][0] - corners[0][0][0][0], 2))

        cv2.circle(imaxis, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), 12, (255, 255, 0), 1)
        cv2.circle(imaxis, (int(corners[0][0][3][0]), int(corners[0][0][3][1])), 12, (0, 0, 255), 1)
        cv2.circle(imaxis, (int(x), int(y)), 12, (0, 255, 255), 2)
        print(square_side_dimension)

        distance_cm_pc = 3317 * math.pow(square_side_dimension, -0.7468) + (-45.95)
        distance_cm_drone = 1.129e+04 * math.pow(square_side_dimension, -0.9631) + (-11.26)

        distance_cm = distance_cm_drone
        imaxis = cv2.putText(imaxis, str(distance_cm), (100, 200), 5, 5, (50, 255, 100))
    except:
        print("non vedo")


    width = 2400/4
    ratio = 4/3
    dim = (int(width), int(width / ratio))
    imaxis = cv2.resize(imaxis, dim, interpolation=cv2.INTER_AREA)




    cv2.imshow("markers", imaxis)


    if cv2.waitKey(1) & 0xFF == ord('q'):  # quit from script
        break