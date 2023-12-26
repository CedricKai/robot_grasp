import cv2
import time
from threading import Thread


class LoadStreams:  # multiple IP or RTSP cameras
    def __init__(self, *sources):                # init cameras index
        sources = [str(x) for x in sources]      # camera index
        n = len(sources)                         # cameras size
        self.imgs = [None] * n                   # get per vaild camera img
        self.start_time = [time.time()] * n      # init time
        self.frame_count = [0] * n               # count frame num

        for i, s in enumerate(sources):
            # Start the thread to read frames from the video stream
            print(f'{i + 1}/{n}: {s}... ', end='')
            cap = cv2.VideoCapture(eval(s) if s.isnumeric() else s)
            assert cap.isOpened(), f'Failed to open {s}'

            # attitude
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))              # width
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))             # height
            cap.set(cv2.CAP_PROP_FPS, 60)                           # set fps
            self.fps = cap.get(cv2.CAP_PROP_FPS) % 100              # get fps

            # per thread
            _, self.imgs[i] = cap.read()  # guarantee first frame
            thread = Thread(target=self.update, args=([i, cap]), daemon=True)
            print(f' success ({w}x{h} at {self.fps:.2f} FPS).')
            thread.start()
        print('')  # newline

    def update(self, index, cap):
        # Read next stream frame in a daemon thread
        while cap.isOpened():
            self.frame_count[index] += 1
            cap.grab()            # 捕捉帧
            success, im = cap.retrieve()           # 将帧数据转换位图像数据
            self.imgs[index] = im if success else self.imgs[index] * 0

            if (spend_time := time.time() - self.start_time[index]) != 0:  # 实时显示帧数
                fps = round(self.frame_count[index] / spend_time, 1)
                cv2.putText(self.imgs[index], f"FPS: {fps}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                self.frame_count[index] = 0
                self.start_time[index] = time.time()

    def __iter__(self):
        # define classes as iterable object
        return self

    def __next__(self):
        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration
        return self.imgs, self.fps

    def __len__(self):
        return 0  # 1E12 frames = 32 streams at 30 FPS for 30 years


if __name__ == "__main__":
    for img in (dataset := LoadStreams(1, 2)):
        for i in range(len(img)):
            cv2.imshow(f"video_{i+1}", img[i])
        cv2.waitKey(1)
