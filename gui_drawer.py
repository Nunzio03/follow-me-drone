import cv2


class GuiDrawer:
    def __init__(self):
        pass

    @staticmethod
    def draw_errors(image, horizontal_error, vertical_error, frontal_error, distance):

        cv2.putText(image, "error x:" + str(horizontal_error), (50, 20), 5, 1, (0, 0, 250))
        cv2.putText(image, "error y:" + str(vertical_error), (50, 40), 5, 1, (0, 0, 250))
        cv2.putText(image, "error z:" + str(frontal_error), (50, 60), 5, 1, (0, 0, 250))
        cv2.putText(image, "distance:" + str(distance), (50, 80), 5, 1, (0, 125, 255))

    @staticmethod
    def draw_setpoint(image, setpointX, setpointY):
        cv2.circle(image, (int(setpointX), int(setpointY)), 12, (0, 0, 255), 3)

    @staticmethod
    def draw_current_PID(image, current_pid, current_parameter):
        cv2.putText(image, "PID:" + str(current_pid)+ " editing "+current_parameter, (10, 200), 5, 1, (0, 255, 0))

    @staticmethod
    def draw_PID_output(image, out_x, out_y, out_z):
        cv2.putText(image, "action x:" + str(round(out_x, 3)), (450, 20), 5, 1, (250, 50, 50))
        cv2.putText(image, "action y:" + str(round(out_y, 3)), (450, 40), 5, 1, (250, 50, 50))
        cv2.putText(image, "action z:" + str(round(out_z, 3)), (450, 60), 5, 1, (250, 50, 50))

    @staticmethod
    def draw_battery_level(image, battery):
        cv2.putText(image, "battery:" + str(battery).strip("\r\n")+"%", (10, 700), 5, 1, (0, 255, 0))
