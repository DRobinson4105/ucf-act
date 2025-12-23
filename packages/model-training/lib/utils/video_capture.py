import threading
from typing import Optional

import cv2
import numpy as np
from numpy.typing import NDArray

class VideoCapture:
    def __init__(self, camera_port: int, frame_shape: Optional[tuple[int, int]] = None) -> None:
        self.camera_port = camera_port
        self.frame_shape = frame_shape

        self.cap = self.open_capture()
        if self.cap is None:
            raise RuntimeError(f"Could not open camera index {camera_port}")
    
        self.lock = threading.Lock()
        self.frame_ready = threading.Event()
        
        self.stop_event = threading.Event()

        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def open_capture(self):
        cap = cv2.VideoCapture(self.camera_port)
        if cap is None or not cap.isOpened():
            return None
        
        cap.set(cv2.CAP_PROP_FPS, 10)

        if self.frame_shape is not None:
            height, width = self.frame_shape
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

        return cap

    def _reader(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                ret = self.cap.grab()

            if ret:
                self.frame_ready.set()
            else:
                with self.lock:
                    try:
                        self.cap.release()
                    except:
                        pass
                    self.cap = None
                self.frame_ready.clear()

                while not self.stop_event.is_set():
                    cap = self.open_capture()

                    if cap is not None:
                        with self.lock:
                            self.cap = cap
                        break

    def read(self) -> tuple[bool, NDArray[np.uint8]]:
        if not self.frame_ready.wait(timeout=1):
            return False, None
        if self.stop_event.is_set():
            return False, None

        with self.lock:
            ret, frame = self.cap.retrieve()
            self.frame_ready.clear()

        return ret, frame

    def release(self) -> bool:
        try:
            self.stop_event.set()
            self.frame_ready.set()
            self.thread.join(timeout=2)
            with self.lock:
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None

        except:
            return False
        
        return True
