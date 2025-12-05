from typing import Optional

import cv2
import threading
import numpy as np
from numpy.typing import NDArray

class VideoCapture:
    def __init__(self, camera_port: int, frame_shape: Optional[tuple[int, int]] = None) -> None:
        self.cap = cv2.VideoCapture(camera_port)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera")
        
        if frame_shape is not None:
            height, width = frame_shape
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    
        self.stopped = False

        self.lock = threading.Lock()
        self.frame_ready = threading.Event()

        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def _reader(self) -> None:
        while not self.stopped:
            with self.lock:
                ret = self.cap.grab()

            if ret:
                self.frame_ready.set()

    def read(self) -> tuple[bool, NDArray[np.uint8]]:
        self.frame_ready.wait()

        with self.lock:
            ret, frame = self.cap.retrieve()
            self.frame_ready.clear()

        return ret, frame

    def release(self) -> bool:
        try:
            self.stopped = True
            self.frame_ready.set()
            self.thread.join()
            self.cap.release()
        except:
            return False
        
        return True
