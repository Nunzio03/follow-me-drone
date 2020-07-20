
import cv2

name = "videodrone.mp4"
rootname = name.split(".")[0]
cap = cv2.VideoCapture(name)
counter = 0
each = 3
length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
for i in range(length):
    ret, frame = cap.read()
    if i % each == 0:
        cv2.imwrite("./videophotodump/img"+"_{0}".format(i) + ".png", frame)

cap.release()