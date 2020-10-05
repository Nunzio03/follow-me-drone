import numpy as np
import cv2
import time
from cv2 import aruco
from djitellopy import Tello
from GUI.gui_drawer import GuiDrawer

from controllers.PID_controller import PIDController as PID
from feedback import MarkerDetector
from PID_parameters_handler import PIDTuner
from controllers.BangBang_controller import BangBangController
from tuning.plotAssistant import PlotAssistant

start_time = time.time()

measurementplot= PlotAssistant()

DRONE_SPEED_X = 25
DRONE_SPEED_Y = 30
DRONE_SPEED_Z = 25


SET_POINT_X = 960 / 2
SET_POINT_Y = 720 / 2
SET_POINT_Z_cm = 320

# pid section
pidX = PID('x')
pidY = PID('y')
pidZ = PID('z')

# pid setup
pidX.set_PID_safeopt([0.57531093, 0.02, 0.17326167])
pidY.set_PID_safeopt([0.7, 0.02, 0.27326167])
pidZ.set_PID_safeopt([0.5, 0.02, 0.15])
# auxiliary controllers
cx = BangBangController(SET_POINT_X, 20, 30)
cy = BangBangController(SET_POINT_Y, 20, 30)
cz = BangBangController(SET_POINT_Z_cm, 20, 25)
# pid keys
tuner = PIDTuner(pidX, pidY, pidZ)
drawer = GuiDrawer()
current_pid, current_parameter = pidX, 'p'

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# video source and calibration parameters setup

video_capture = cv2.VideoCapture(0)
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

detector = MarkerDetector(aruco_dict, mtx, dist)

# loop start
drone.takeoff()
while True:
    action_z, action_y, action_x = 0, 0, 0
    # ret, frame = video_capture.read()

    frame = drone.get_frame_read().frame

    image, horizontal_error, vertical_error, frontal_error = detector.detect_and_compute_error_values(frame,
                                                                                                      SET_POINT_X,
                                                                                                      SET_POINT_Y,
                                                                                                      SET_POINT_Z_cm,
                                                                                                      42)

    if horizontal_error is not None:
        measurementplot.insertMeasurement(horizontal_error, vertical_error, frontal_error)
        drawer.draw_errors(image, horizontal_error, vertical_error, frontal_error, frontal_error+SET_POINT_Z_cm)
        action_x = int(-pidX.compute_action(horizontal_error))
        action_y = int(pidY.compute_action(vertical_error))
        action_z = int(pidZ.compute_action(frontal_error))
        # action_x = cx.compute_action(horizontal_error)
        # action_y = cy.compute_action(-vertical_error)
        # action_z = cz.compute_action(-frontal_error)
        drawer.draw_controller_output(image, action_x, action_y, action_z)

    drone.send_rc_control(0, action_z, action_y, action_x)  # turn with yaw

    drawer.draw_current_PID(image, current_pid, current_parameter)
    drawer.draw_setpoint(image, SET_POINT_X, SET_POINT_Y, SET_POINT_Z_cm)
    if time.time() - start_time >= 5:
        battery_level = drone.get_battery()
        start_time = time.time()
    drawer.draw_battery_level(image, battery_level)

    window_dimension = 0.3

    width = 2400*window_dimension
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

    else:
        current_pid = tuner.get_pid(key_pressed)
        current_parameter = tuner.get_parameter(key_pressed)
        tuner.tune(key_pressed)
