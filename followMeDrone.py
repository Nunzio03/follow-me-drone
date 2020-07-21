import numpy as np
import cv2, PIL, os
import time
from cv2 import aruco
from djitellopy import Tello
from gui_drawer import GuiDrawer

from PID_controller import PIDController as PID
from marker_detector import MarkerDetector

TOLERANCE_X = 20
TOLERANCE_Y = 20
TOLERANCE_Z = 20

DRONE_SPEED_X = 25
DRONE_SPEED_Y = 30
DRONE_SPEED_Z = 25

SET_POINT_X = 960 / 2
SET_POINT_Y = 720 / 2
SET_POINT_Z_cm = 180

# pid section
pidX = PID('x')
pidY = PID('y')
pidZ = PID('z')

# pid keys
current_PID_parameter = 'p'
current_axis = 'x'
current_pid = pidX

drawer = GuiDrawer()

DELAY = 0.0002
start_time = 0


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

# video source and calibration parameters setup

# video_capture = cv2.VideoCapture(0)
drone = Tello()
drone.connect()
battery_level = drone.get_battery()
drone.streamon()

# detection

pc_mtx = np.array([[1.73223258e+03, 0.00000000e+00, 1.27300230e+03],
                   [0.00000000e+00, 1.73223258e+03, 1.03042217e+03],
                   [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

pc_dist = np.array([[-3.89477169e+00],
                    [-9.72135202e-02],
                    [1.04299558e-02],
                    [8.40170760e-05],
                    [-2.17736443e+00],
                    [-3.89139953e+00],
                    [-1.90794581e-01],
                    [-1.85298591e+00],
                    [0.00000000e+00],
                    [0.00000000e+00],
                    [0.00000000e+00],
                    [0.00000000e+00],
                    [0.00000000e+00],
                    [0.00000000e+00]])

drone_mtx = np.array([[1.74213359e+03, 0.00000000e+00, 1.27150514e+03],
                      [0.00000000e+00, 1.74213359e+03, 1.02516982e+03],
                      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

drone_dist = np.array([[-1.69684883e+00],
                       [-6.85717812e+00],
                       [9.93624014e-03],
                       [6.20144084e-04],
                       [-1.18739065e+01],
                       [-1.69460711e+00],
                       [-6.99110211e+00],
                       [-1.13633464e+01],
                       [0.00000000e+00],
                       [0.00000000e+00],
                       [0.00000000e+00],
                       [0.00000000e+00],
                       [0.00000000e+00],
                       [0.00000000e+00]])

mtx, dist = drone_mtx, drone_dist

detector = MarkerDetector(aruco_dict, parameters, mtx, dist)

# loop start
# drone.takeoff()
while True:

    # ret, frame = video_capture.read()

    frame = drone.get_frame_read().frame

    image, horizontal_error, vertical_error, frontal_error = detector.detect_and_compute_error_values(frame,
                                                                                                      SET_POINT_X,
                                                                                                      SET_POINT_Y,
                                                                                                      SET_POINT_Z_cm)

    if horizontal_error is not None:
        drawer.draw_errors(image, horizontal_error, vertical_error, frontal_error, frontal_error+SET_POINT_Z_cm)
        action_x = pidX.compute_action(horizontal_error)
        action_y = pidY.compute_action(vertical_error)
        action_z = pidZ.compute_action(frontal_error)
        drawer.draw_PID_output(image, action_x, action_y, action_z)

        # drone.send_rc_control(0, front_back_velocity, up_down_velocity, right_left_velocity)  # turn with yaw

    drawer.draw_current_PID(image, current_pid, current_PID_parameter)
    drawer.draw_setpoint(image, SET_POINT_X, SET_POINT_Y)
    if time.time() - start_time >= 5:
        battery_level = drone.get_battery()
        start_time = time.time()
    drawer.draw_battery_level(image, battery_level)

    width = 2400/3
    ratio = 16 / 9
    dim = (int(width), int(width / ratio))
    image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    cv2.imshow("markers", image)

    # key hadling
    key_pressed = cv2.waitKey(1)

    if key_pressed & 0xFF == ord('q'):  # quit from script
        drone.land()
        drone.get_battery()
        break

    elif key_pressed & 0xFF == ord("p"):
        current_PID_parameter = 'p'
    elif key_pressed & 0xFF == ord("i"):
        current_PID_parameter = 'i'
    elif key_pressed & 0xFF == ord("d"):
        current_PID_parameter = 'd'

    elif key_pressed & 0xFF == ord("x"):
        current_axis = 'x'
        current_pid = pidX
    elif key_pressed & 0xFF == ord("y"):
        current_axis = 'y'
        current_pid = pidY
    elif key_pressed & 0xFF == ord("z"):
        current_axis = 'z'
        current_pid = pidZ

    elif key_pressed & 0xFF == ord("8"):
        if current_axis == 'x':
            pidX.increase_gain(current_PID_parameter, 0.01)
        if current_axis == 'y':
            pidY.increase_gain(current_PID_parameter, 0.01)
        if current_axis == 'z':
            pidZ.increase_gain(current_PID_parameter, 0.01)

    elif key_pressed & 0xFF == ord("2"):
        if current_axis == 'x':
            pidX.increase_gain(current_PID_parameter, -0.01)
        if current_axis == 'y':
            pidY.increase_gain(current_PID_parameter, -0.01)
        if current_axis == 'z':
            pidZ.increase_gain(current_PID_parameter, -0.01)

    elif key_pressed & 0xFF == ord("0"):
        if current_axis == 'x':
            pidX.set_gain(current_PID_parameter, 0)
        if current_axis == 'y':
            pidY.set_gain(current_PID_parameter, 0)
        if current_axis == 'z':
            pidZ.set_gain(current_PID_parameter, 0)
