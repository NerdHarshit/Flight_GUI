from PyQt6.QtCore import QThread, pyqtSignal
import cv2
import numpy as np


class VideoSaver(QThread):

    finished = pyqtSignal()
    error = pyqtSignal(str)

    def __init__(self, frames, filename="flight.mp4"):
        super().__init__()
        self.frames = frames
        self.filename = filename

    def run(self):
        try:
            if not self.frames:
                self.error.emit("No frames to save")
                return

            h = self.frames[0].height()
            w = self.frames[0].width()

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(self.filename, fourcc, 30, (w, h))

            for img in self.frames:
                ptr = img.bits()
                ptr.setsize(img.sizeInBytes())
                frame = np.array(ptr).reshape(h, w, 4)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                out.write(frame)

            out.release()

            self.finished.emit()

        except Exception as e:
            self.error.emit(str(e))