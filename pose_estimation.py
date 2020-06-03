import numpy as np
import cv2, PIL, os
from cv2 import aruco
from tellopy import Tello
import math

TOLERANCE_X = 30
TOLERANCE_Y = 30
TOLERANCE_Z = 20
SLOWDOWN_THRESHOLD_X = 50
SLOWDOWN_THRESHOLD_Y = 50
SLOWDONW_THRESHOLD_Z = 60
DRONE_SPEED_X = 20
DRONE_SPEED_Y = 20
DRONE_SPEED_Z = 10
SET_POINT_X = 960/2
SET_POINT_Y = 720/2
SET_POINT_Z = 150

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()

# video source and calibration parameters setup

video_capture = cv2.VideoCapture(0)
#drone = Tello()
#drone.connect()
#drone.streamon()


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

# loop start

while True:

    ret, frame = video_capture.read()

    #frame = drone.get_frame_read().frame

    #frame = cv2.undistort(src = frame, cameraMatrix = mtx, distCoeffs = dist)

    #frame = cv2.flip(frame, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters)
    # SUB PIXEL DETECTION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    for corner in corners:
        cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    size_of_marker = 0.15  # side lenght of the marker in meters
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

    length_of_axis = 0.1
    imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    if tvecs is not None:
        for i in range(len(tvecs)):
            imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)
            dst, jacobian = cv2.Rodrigues(rvecs)
            #print("phi:", math.atan2(dst[2][0], dst[2][1])*180/math.pi)
            #print("theta:", math.acos(dst[2][2])*180/math.pi)
            #print("psi:", -math.atan2(dst[0][2], dst[1][2])*180/math.pi)
            x = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
            y = (corners[0][0][0][1] + corners[0][0][2][1]) / 2
            square_side_dimension = math.sqrt(math.pow(corners[0][0][3][1] - corners[0][0][0][1], 2) +
                                              math.pow(corners[0][0][3][0] - corners[0][0][0][0], 2))

            cv2.circle(imaxis, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), 12, (255, 255, 0), 1)
            cv2.circle(imaxis, (int(corners[0][0][3][0]), int(corners[0][0][3][1])), 12, (0, 0, 255), 1)
            cv2.circle(imaxis, (int(x), int(y)), 12, (0, 255, 255), 2)
    try:

        #print(square_side_dimension)

        distance_cm_pc = 3317 * math.pow(square_side_dimension, -0.7468) + (-45.95)
        distance_cm_drone = 1.129e+04 * math.pow(square_side_dimension, -0.9631) + (-11.26)

        distance_cm = distance_cm_drone

        imaxis = cv2.putText(imaxis, str(distance_cm), (100, 200), 5, 5, (250, 255, 250))

    except:
        print("non vedo")
        x, y, distance_cm = None, None, None

    cv2.circle(imaxis, (int(960/2), int(720/2)), 12, (0, 0, 255), 3)

    if x is not None:
        error_x = x - SET_POINT_X
        error_y = y - SET_POINT_Y
        error_z = distance_cm - SET_POINT_Z

        if error_x < -TOLERANCE_X:
            # print("sposta il drone alla sua SX")
            right_left_velocity = - DRONE_SPEED_X

        elif error_x > TOLERANCE_X:
            # print("sposta il drone alla sua DX")
            right_left_velocity = DRONE_SPEED_X
        else:
            # print("OK")
            right_left_velocity = 0

        if error_y < -TOLERANCE_Y:
            # print("sposta il drone in ALTO")
            up_down_velocity = DRONE_SPEED_Y
        elif error_y > TOLERANCE_Y:
            # print("sposta il drone in BASSO")
            up_down_velocity = - DRONE_SPEED_Y

        else:
            # print("OK")
            up_down_velocity = 0

        if error_z < -TOLERANCE_Z:
            # print("sposta il drone INDIETRO")
            front_back_velocity = - DRONE_SPEED_Z
        elif error_z > TOLERANCE_Z:
            # print("sposta il drone in AVANTI ")
            front_back_velocity = DRONE_SPEED_Z

        else:
            # print("OK")
            front_back_velocity = 0

    else:
        right_left_velocity, up_down_velocity, front_back_velocity = 0, 0, 0

    width = 2400/4
    ratio = 4/3
    dim = (int(width), int(width / ratio))
    imaxis = cv2.resize(imaxis, dim, interpolation=cv2.INTER_AREA)
    cv2.imshow("markers", imaxis)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # quit from script
        break

