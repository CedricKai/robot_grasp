import cv2
import time
from threading import Thread


class LoadStreams:  # multiple IP or RTSP cameras
    def __init__(self, *sources):
        sources = [str(x) for x in sources]
        n = len(sources)
        self.imgs = [None] * n

        for i, s in enumerate(sources):
            # Start the thread to read frames from the video stream
            print(f'{i + 1}/{n}: {s}... ', end='')
            cap = cv2.VideoCapture(eval(s) if s.isnumeric() else s)
            assert cap.isOpened(), f'Failed to open {s}'
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = cap.get(cv2.CAP_PROP_FPS) % 100

            _, self.imgs[i] = cap.read()  # guarantee first frame
            thread = Thread(target=self.update, args=([i, cap]), daemon=True)
            print(f' success ({w}x{h} at {self.fps:.2f} FPS).')
            thread.start()
        print('')  # newline

    def update(self, index, cap):
        # Read next stream frame in a daemon thread
        n = 0
        while cap.isOpened():
            n += 1
            # _, self.imgs[index] = cap.read()
            cap.grab()
            if n == 4:  # read every 4th frame
                success, im = cap.retrieve()
                self.imgs[index] = im if success else self.imgs[index] * 0
                n = 0
            time.sleep(1 / self.fps)  # wait time

    def __iter__(self):
        return self

    def __next__(self):
        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration
        return self.imgs

    def __len__(self):
        return 0  # 1E12 frames = 32 streams at 30 FPS for 30 years


for img in (dataset := LoadStreams(1)):
    cv2.imshow("video", img[0])
    cv2.waitKey(1)
