import cv2
from cv2 import aruco
import math


class MarkerDetector:

    def __init__(self, aruco_dict, mtx, dist):
        self.aruco_dict = aruco_dict
        self.parameters = aruco.DetectorParameters_create()
        self.mtx = mtx
        self.dist = dist

    def detect(self, frame, target):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

        size_of_marker = 0.15  # side lenght of the marker in meters
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker, self.mtx, self.dist)

        length_of_axis = 0.1
        imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        #print(ids, corners)
        if tvecs is not None:
            target_id = None
            for i in range(len(ids)):
                if ids[i] == target:
                    target_id = i

            if target_id is not None:
                imaxis = aruco.drawAxis(imaxis, self.mtx, self.dist, rvecs[target_id], tvecs[target_id], length_of_axis)
                x = (corners[target_id][0][0][0] + corners[target_id][0][2][0]) / 2
                y = (corners[target_id][0][0][1] + corners[target_id][0][2][1]) / 2
                square_side_dimension_px = math.sqrt(math.pow(corners[target_id][0][3][1] - corners[target_id][0][0][1], 2) + math.pow(corners[target_id][0][3][0] - corners[target_id][0][0][0], 2))

                return imaxis, x, y, square_side_dimension_px
            else:
                return frame, None, None, None
        else:
            return frame, None, None, None

    def detect_and_compute_error_values(self, frame, set_point_x, set_point_y, set_point_z, target):

        imaxis, x, y, square_side_dimension_px = self.detect(frame, target)
        if x is not None:
            horizontal_error, vertical_error, frontal_error = compute_error_values(x, y, square_side_dimension_px, set_point_x, set_point_y, set_point_z)
            return imaxis, horizontal_error, vertical_error, frontal_error
        else:
            return imaxis, None, None, None


def distance_estimator(square_side_dimension_px):
    return 1.129e+04 * math.pow(square_side_dimension_px, -0.9631) + (-11.26)


def compute_error_values(x, y, square_side_dimension_px, set_point_x, set_point_y, set_point_z):

    frontal_distance_cm = int(distance_estimator(square_side_dimension_px))
    cm_pix_ratio = 15 / square_side_dimension_px  # 15 [cm] / marker dimension [pixel]
    horizontal_error = int((set_point_x - x) * cm_pix_ratio)
    vertical_error = int((set_point_y - y) * cm_pix_ratio)
    frontal_error = frontal_distance_cm - set_point_z
    return horizontal_error, vertical_error, frontal_error
