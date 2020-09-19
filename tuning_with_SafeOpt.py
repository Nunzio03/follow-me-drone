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
import measurementAssistant
import SafeOptTunerAssistant

start_time = time.time()

DRONE_SPEED_X = 25
DRONE_SPEED_Y = 30
DRONE_SPEED_Z = 25



SET_POINT_X = 960 / 2
SET_POINT_Y = 720 / 2
SET_POINT_Z_cm = 110

# pid section
pidX = PID('x')
pidY = PID('y')
pidZ = PID('z')

# pid setup
pid_safeopt_param = [0.8, 0, 0]
pidX.set_PID_safeopt(pid_safeopt_param)

# auxiliary controllers
cx = BangBangController(SET_POINT_X, 20, 30)
cy = BangBangController(SET_POINT_Y, 20, 30)
cz = BangBangController(SET_POINT_Z_cm, 20, 30)
# pid keys
pidSetter = PIDTuner(pidX, pidY, pidZ)
drawer = GuiDrawer()
current_pid, current_parameter = pidX, 'p'

# measurements
measure42 = measurementAssistant.MeasurementAssistant("x_axis42",5,3)
measure33 = measurementAssistant.MeasurementAssistant("x_axis33",5,3)
# safeopt assistant

safeoptassistant = SafeOptTunerAssistant.Tuner(0.025 ** 2, pid_safeopt_param[0], pid_safeopt_param[1], pid_safeopt_param[2])
target = 33

def switchTarget():
    if target == 33:
        return 42
    elif target == 42:
        return 33


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)




# video source and calibration parameters setup

#video_capture = cv2.VideoCapture(0)
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
#drone.takeoff()
while True:

    #ret, frame = video_capture.read()

    frame = drone.get_frame_read().frame

    image, horizontal_error, vertical_error, frontal_error = detector.detect_and_compute_error_values(frame,
                                                                                                      SET_POINT_X,
                                                                                                      SET_POINT_Y,
                                                                                                      SET_POINT_Z_cm, target)
    action_x, action_y, action_z = 0,0,0

    if horizontal_error is not None:
        drawer.draw_errors(image, horizontal_error, vertical_error, frontal_error, frontal_error+SET_POINT_Z_cm)
        #action_x = int(-pidX.compute_action(horizontal_error))
        #action_y = int(pidY.compute_action(vertical_error))
        #action_z = int(pidZ.compute_action(frontal_error))
        action_x = cx.compute_action(horizontal_error)
        action_y = cy.compute_action(-vertical_error)
        action_z = cz.compute_action(-frontal_error)

        if target == 33:
            measure33.write_measurement(horizontal_error)
            if measure33.is_in_setpoint():
                target = switchTarget()
                measure33.new_round(pid_safeopt_param)

        elif target == 42:
            measure42.write_measurement(horizontal_error)
            if measure42.is_in_setpoint():
                safeoptassistant.optimize(100000 / (0.00001 + measure42.compute_abs_sum()))
                pid_safeopt_param = safeoptassistant.get_param()
                pidX.set_PID_safeopt(pid_safeopt_param)
                target = switchTarget()
                measure42.new_round(pid_safeopt_param)

    #drone.send_rc_control(0, action_z, action_y, action_x)  # turn with yaw

    drawer.draw_controller_output(image, action_x, action_y, action_z)
    drawer.draw_current_PID(image, current_pid, current_parameter)
    if target == 42:
        drawer.draw_fitness_value(image,measure42.fitness())
    elif target == 33:
        drawer.draw_fitness_value(image, measure33.fitness())
    drawer.draw_setpoint(image, SET_POINT_X, SET_POINT_Y)

    if time.time() - start_time >= 5:
        battery_level = drone.get_battery()
        start_time = time.time()
    drawer.draw_battery_level(image, battery_level)

    width = 2200/2
    ratio = 16 / 9
    dim = (int(width), int(width / ratio))
    image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    cv2.imshow("markers", image)

    # key hadling
    key_pressed = cv2.waitKey(1)

    if key_pressed & 0xFF == ord('q'):  # quit from script
        #drone.land()
        #drone.get_battery()
        print(safeoptassistant.get_best_param())
        break

    else:
        current_pid = pidSetter.get_pid(key_pressed)
        current_parameter = pidSetter.get_parameter(key_pressed)
        pidSetter.tune(key_pressed)
