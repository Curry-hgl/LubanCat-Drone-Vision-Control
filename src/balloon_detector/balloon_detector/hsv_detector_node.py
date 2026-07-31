import cv2
import numpy as np

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# ========== 固定 HSV 阈值（根据你的环境调好后再写死）==========
# 示例：检测红色
lower = np.array([0, 130, 100])
upper = np.array([10, 255, 255])
# ============================================================

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 直接用固定阈值生成掩码
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