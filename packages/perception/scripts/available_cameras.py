import cv2

for i in range(5):
    cap = cv2.VideoCapture(i)
    print(f"Index {i}: opened={cap.isOpened()}")
    cap.release()