import cv2


class GuiDrawer:
    def __init__(self):
        pass

    @staticmethod
    def draw_errors(image, horizontal_error, vertical_error, frontal_error):

        image = cv2.putText(image, "error x:" + str(horizontal_error), (50, 20), 5, 1, (0, 0, 250))
        image = cv2.putText(image, "error y:" + str(vertical_error), (50, 40), 5, 1, (0, 0, 250))
        image = cv2.putText(image, "error z:" + str(frontal_error), (50, 60), 5, 1, (0, 0, 250))

        return image

    @staticmethod
    def draw_setpoint(image, setpointX, setpointY):
        return cv2.circle(image, (int(setpointX), int(setpointY)), 12, (0, 0, 255), 3)

    @staticmethod
    def draw_current_PID(image, current_pid):
        return cv2.putText(image, "PID:" + str(current_pid), (10, 200), 5, 1, (0, 255, 0))

    @staticmethod
    def draw_PID_output(image, out_x, out_y, out_z):
        image = cv2.putText(image, "action x:" + str(round(out_x, 3)), (50, 420), 5, 1, (250, 0, 0))
        image = cv2.putText(image, "action y:" + str(round(out_y, 3)), (50, 440), 5, 1, (250, 0, 0))
        image = cv2.putText(image, "action z:" + str(round(out_z, 3)), (50, 460), 5, 1, (250, 0, 0))

        return image
