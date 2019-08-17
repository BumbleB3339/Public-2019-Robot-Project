import os
import cv2
import urllib.request
import numpy as np
from networktables import NetworkTables
import time

from collections import deque
from threading import Thread
from queue import Queue

""" This code was written very roughly and quickly to try and improve camera
latency by running all stream manipulations on the Driver Station and not on
the roboRIO. It does indeed help!
Also, it records all match video streams and saves them on the Driver Station.
Please notice that for the reasons described above, this code contains some bugs
and is not completely stable, so it is highly recommended to learn it and then
rewrite parts of it. """


MATCH_TYPE = {0: 'NA', 1: 'PM', 2: 'QM', 3: 'EM'}


class MatchRecorder:
    def __init__(self, fps, timeout=1.0):
        self.timeout = timeout
        self.fps = fps
        self.thread = None
        self.stopped = False
        self.queue = None

        self.last_prefix = ""
        self.capture_index = 0

    def start(self, output_path, fourcc, fps, frame):
        self.running = True
        self.queue = Queue()
        os.makedirs("captures", exist_ok=True)
        self.writer = cv2.VideoWriter(
            output_path, fourcc, fps, (frame.shape[1], frame.shape[0]))
        self.thread = Thread(target=self.run, args=())
        self.thread.daemon = True
        self.thread.start()
        self.last_time = time.time()
        return self

    def run(self):
        while self.running:
            if not self.queue.empty():
                frame = self.queue.get()
                self.writer.write(frame)
            else:
                time.sleep(self.timeout)

    def update(self, frame, is_match):
        if not self.running and is_match:
            match_type = table.getNumber('FMSInfo/MatchType', 0)
            match_number = table.getNumber('FMSInfo/MatchNumber', 0)
            replay_number = table.getNumber('FMSInfo/ReplayNumber', 0)

            prefix = "{}{}-{}".format(MATCH_TYPE[match_type],
                                      match_number, replay_number)
            if prefix != self.last_prefix:
                self.capture_index = 0
                self.last_prefix = prefix
            path = "captures/{}_{}.avi".format(prefix, self.capture_index)
            self.start(
                path,
                cv2.VideoWriter_fourcc(
                    'M',
                    'J',
                    'P',
                    'G'),
                self.fps,
                frame)
            self.capture_index += 1

        if self.running and not is_match:
            self.finish()

        if self.running and time.time() - self.last_time >= 1. / self.fps:
            self.queue.put(frame)
            self.last_time = time.time()

    def flush(self):
        while not self.queue.empty():
            frame = self.queue.get()
            self.writer.write(frame)

    def finish(self):
        self.running = False
        self.thread.join()
        self.flush()
        self.writer.release()


class VideoStream:
    def __init__(self, path, max_delta=1.0):
        self.cap = cv2.VideoCapture(path)
        (self.grabbed, self.frame) = self.cap.read()

        self.max_delta = max_delta
        self.start_time = 0

    def start(self):
        self.running = True
        Thread(target=self.run, args=()).start()
        return self

    def run(self):
        while self.running:
            self.start_time = time.time()
            (self.grabbed, self.frame) = self.cap.read()

    def read(self):
        flag = time.time() - self.start_time > self.max_delta and self.max_delta > 0

        return self.frame, flag

    def stop(self):
        self.stopped = True


# ---------- MAIN CODE ----------
WINDOW_NAME = '3339 Camera Feed'
WINDOW_WIDTH = 320 * 2
WINDOW_HEIGHT = 240 * 2

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240

# ARROW PARAMS
ARROW_LENGTH = 40  # in pixels
PRINT_DISTANCE_FROM_SIDE = 20  # in pixels
PRINT_DISTANCE_FROM_TOP = 10  # in pixels

LEFT_ARROW_TOP = (PRINT_DISTANCE_FROM_SIDE, PRINT_DISTANCE_FROM_TOP)
LEFT_ARROW_DOWN = (
    PRINT_DISTANCE_FROM_SIDE,
    PRINT_DISTANCE_FROM_TOP +
    ARROW_LENGTH)

RIGHT_ARROW_TOP = (
    SCREEN_WIDTH -
    PRINT_DISTANCE_FROM_SIDE,
    PRINT_DISTANCE_FROM_TOP)
RIGHT_ARROW_DOWN = (
    SCREEN_WIDTH -
    PRINT_DISTANCE_FROM_SIDE,
    PRINT_DISTANCE_FROM_TOP +
    ARROW_LENGTH)

ARROW_THICKNESS = 4

FORWARD_COLOR = (0, 255, 0)
BACKWARD_COLOR = (0, 0, 255)

# GEAR PARAMS
LEFT_GEAR_POINT = (
    PRINT_DISTANCE_FROM_SIDE -
    15,
    SCREEN_HEIGHT -
    PRINT_DISTANCE_FROM_TOP)

GEAR_THICKNESS = 4

POWER_COLOR = (210, 44, 44)
SPEED_COLOR = (239, 10, 247)

STRAM_PORT = "5810"
STREAM_IP = "10.33.39.2"


print('Connecting to network tables...')
NetworkTables.initialize(STREAM_IP)  # router
# NetworkTables.initialize('169.254.223.10') #roborio
table = NetworkTables.getTable('SmartDashboard')
print('Connected to network tables...')

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

STREAM_ADDRESS = 'http://' + STREAM_IP + ':' + STRAM_PORT + '/stream.mjpg'
print('Connecting to camera stream..')
stream = VideoStream(STREAM_ADDRESS).start()
print('Connected to camera stream.')

fps = 0
FRAME_SAMPLE = 5

last_time = time.time()
last_frame = 0

# Capture
CAPTURE_FPS = 15

try:
    recorder = MatchRecorder(CAPTURE_FPS)
except Exception as e:
    print(e)

table.putBoolean('RecordKeyEvent', False)

while True:
    frame, flag = stream.read()

    direction = table.getString('Game/Direction', 'FORWARD')
    gear = table.getString('Game/Gear', 'SPEED_GEAR')

    if direction == 'BACKWARD':
        # frame = cv2.flip(frame, 0)
        cv2.arrowedLine(
            frame,
            LEFT_ARROW_TOP,
            LEFT_ARROW_DOWN,
            BACKWARD_COLOR,
            ARROW_THICKNESS)
        cv2.arrowedLine(
            frame,
            RIGHT_ARROW_TOP,
            RIGHT_ARROW_DOWN,
            BACKWARD_COLOR,
            ARROW_THICKNESS)
    elif direction == 'FORWARD':
        frame = cv2.flip(frame, -1)
        cv2.arrowedLine(
            frame,
            LEFT_ARROW_DOWN,
            LEFT_ARROW_TOP,
            FORWARD_COLOR,
            ARROW_THICKNESS)
        cv2.arrowedLine(
            frame,
            RIGHT_ARROW_DOWN,
            RIGHT_ARROW_TOP,
            FORWARD_COLOR,
            ARROW_THICKNESS)

    if gear == 'SPEED_GEAR':
        cv2.putText(
            frame,
            'S',
            LEFT_GEAR_POINT,
            cv2.FONT_HERSHEY_SIMPLEX,
            1.690,
            SPEED_COLOR,
            GEAR_THICKNESS)
    elif gear == 'POWER_GEAR':
        cv2.putText(
            frame,
            'P',
            LEFT_GEAR_POINT,
            cv2.FONT_HERSHEY_SIMPLEX,
            1.690,
            POWER_COLOR,
            GEAR_THICKNESS)

    # Calculate FPS
    current_frame = stream.cap.get(cv2.CAP_PROP_POS_FRAMES)
    if current_frame - last_frame >= FRAME_SAMPLE:
        current_time = time.time()
        fps = (current_frame - last_frame) / (current_time - last_time)
        last_time = current_time
        last_frame = current_frame

    cv2.putText(frame, str(round(fps, 2)), (285, 230),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255))

    try:
        cv2.imshow(WINDOW_NAME, frame)
    except BaseException:
        stream.stop()
        stream = VideoStream(STREAM_ADDRESS).start()

        try:
            recorder.finish()
        except Exception as e:
            print(e)

    if table.getBoolean('Recording/VisionEnabled', False):
        cv2.circle(frame, (140, 10), 5, (0, 255, 0), -1)
    else:
        cv2.circle(frame, (140, 10), 5, (255, 255, 255), -1)

    timestamp = table.getNumber('Recording/Timestamp', 0)
    cv2.putText(frame, "{0:0=3d}".format(int(timestamp)),
                (155, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))

    try:
        recorder.update(frame, table.getBoolean('Recording/IsMatch', True))
    except Exception as e:
        print(e)

        try:
            recorder.finish()
        except Exception as e:
            print(e)

    if cv2.waitKey(1) == 27:
        exit(0)
