import cv2

for i in range(10):
    cap = cv2.VideoCapture(i)
    opened = cap.isOpened()

    if opened:
        print(f"Index {i} open: ({int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}, {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))})")
    
    cap.release()
    
    if not opened: break
