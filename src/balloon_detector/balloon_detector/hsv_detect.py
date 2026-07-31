import os
os.environ["QT_LOGGING_RULES"] = "qt.qpa.fonts.warning=false"

import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

cv2.namedWindow("Trackbars")
cv2.createTrackbar("H_Low", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("H_High", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S_Low", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("S_High", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V_Low", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V_High", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hl = cv2.getTrackbarPos("H_Low", "Trackbars")
    hh = cv2.getTrackbarPos("H_High", "Trackbars")
    sl = cv2.getTrackbarPos("S_Low", "Trackbars")
    sh = cv2.getTrackbarPos("S_High", "Trackbars")
    vl = cv2.getTrackbarPos("V_Low", "Trackbars")
    vh = cv2.getTrackbarPos("V_High", "Trackbars")

    lower = np.array([hl, sl, vl])
    upper = np.array([hh, sh, vh])

    mask = cv2.inRange(hsv, lower, upper)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = frame.copy()
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow("Result", result)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
