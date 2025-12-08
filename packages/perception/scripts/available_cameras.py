import cv2
import time

cv2.setLogLevel(0)

for i in range(10):
    cap = cv2.VideoCapture(i)
    opened = cap.isOpened()

    time.sleep(0.1)

    ret, frame = cap.read()
    if ret and frame is not None:
        print(f"Index {i} open: ({int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}, {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))})")
    
    cap.release()
    
    if not opened: break
