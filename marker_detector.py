import numpy as np
import cv2, PIL, os
from cv2 import aruco
import math


class MarkerDetector:

    def __init__(self, aruco_dict, parameters, mtx, dist):
        self.aruco_dict = aruco_dict
        self.parameters = parameters
        self.mtx = mtx
        self.dist = dist

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict,
                                                              parameters=self.parameters)
        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

        # frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        size_of_marker = 0.15  # side lenght of the marker in meters
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker, self.mtx, self.dist)

        length_of_axis = 0.1
        imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        if tvecs is not None:
            for i in range(len(tvecs)):  # TO DO MARKER ID IDENTIFICATION
                imaxis = aruco.drawAxis(imaxis, self.mtx, self.dist, rvecs[i], tvecs[i], length_of_axis)
                # dst, jacobian = cv2.Rodrigues(rvecs[i])
                # print("phi:", math.atan2(dst[2][0], dst[2][1])*180/math.pi)
                # print("theta:", math.acos(dst[2][2])*180/math.pi)
                # print("psi:", -math.atan2(dst[0][2], dst[1][2])*180/math.pi)
                x = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
                y = (corners[0][0][0][1] + corners[0][0][2][1]) / 2
                square_side_dimension_px = math.sqrt(math.pow(corners[0][0][3][1] - corners[0][0][0][1], 2) +
                                                     math.pow(corners[0][0][3][0] - corners[0][0][0][0], 2))

                return imaxis, x, y, square_side_dimension_px
        else:
            return frame, None, None, None

    @staticmethod
    def compute_error_values(x, y, square_side_dimension_px, set_point_x, set_point_y, set_point_z):

        # distance_cm_pc = 3317 * math.pow(square_side_dimension_px, -0.7468) + (-45.95)
        frontal_distance_cm_drone = 1.129e+04 * math.pow(square_side_dimension_px, -0.9631) + (-11.26)

        frontal_distance_cm = int(frontal_distance_cm_drone)
        cm_pix_ratio = 15 / square_side_dimension_px
        horizontal_error = -int((x - set_point_x) * cm_pix_ratio)
        vertical_error = int((y - set_point_y) * cm_pix_ratio)
        frontal_error = frontal_distance_cm - set_point_z
        return horizontal_error, vertical_error, frontal_error

    def detect_and_compute_error_values(self, frame, set_point_x, set_point_y, set_point_z):

        imaxis, x, y, square_side_dimension_px = self.detect(frame)
        if x is not None:
            horizontal_error, vertical_error, frontal_error = self.compute_error_values(x, y, square_side_dimension_px,
                                                                                        set_point_x, set_point_y,
                                                                                        set_point_z)
            return imaxis, horizontal_error, vertical_error, frontal_error
        else:
            return imaxis, None, None, None
