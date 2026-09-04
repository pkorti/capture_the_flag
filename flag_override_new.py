import os
import subprocess
import time

import depthai as dai
from ultralytics import YOLO


# ============================================================
# YOLO
# ============================================================

# Put your trained YOLO weights in ~/gpscar and name them best.pt,
# OR set YOLO_MODEL_PATH before starting manage.py.
YOLO_MODEL_PATH = os.environ.get("YOLO_MODEL_PATH", "best.pt")
YOLO_CONFIDENCE = 0.50

VALID_FLAG_CLASSES = {
    "PINK",
    "YELLOW",
    "ORANGE",
    "BLUE",
}

model = YOLO(YOLO_MODEL_PATH)


# ============================================================
# FLAG MISSION PARAMETERS
# ============================================================

MIN_AREA = 250
TRIGGER_AREA = 1000
SPOTTED_FRAMES = 3

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER = IMAGE_WIDTH // 2
CENTER_DEADBAND = 35

APPROACH_THROTTLE = 0.18
HIT_THROTTLE = 0.18
REVERSE_THROTTLE = -0.15

STEERING_GAIN = 0.0020
MAX_STEERING = 0.35

HIT_START_AREA = 30000
LOST_FRAMES_FOR_KNOCKDOWN = 8
LOST_FRAMES_BEFORE_ABORT = 30
MAX_HIT_TIME = 2.0
REVERSE_TIME = 3.0

SPEAKER_DEVICE = "plughw:2,0"


# ============================================================
# AUDIO
# ============================================================

def speak(text):
    print(f"SPEAKING: {text}")

    try:
        espeak_process = subprocess.Popen(
            ["espeak", "--stdout", text],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )

        subprocess.Popen(
            ["aplay", "-D", SPEAKER_DEVICE],
            stdin=espeak_process.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        espeak_process.stdout.close()

    except Exception as e:
        print("Speech error:", e)


# ============================================================
# YOLO FLAG DETECTOR
# ============================================================

def detect_flag(frame):
    """
    Run YOLO on one OAK-D BGR frame.

    Returns the same dictionary format that the existing mission
    state machine already expects:
        color, cx, cy, x, y, w, h, area, confidence

    Only the four mission flag classes are accepted.
    """

    results = model.predict(
        source=frame,
        conf=YOLO_CONFIDENCE,
        verbose=False
    )

    best_detection = None
    best_confidence = 0.0

    for result in results:
        if result.boxes is None:
            continue

        for box in result.boxes:
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])

            color_name = str(model.names[class_id]).strip().upper()

            if color_name not in VALID_FLAG_CLASSES:
                continue

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0].tolist()
            )

            w = max(0, x2 - x1)
            h = max(0, y2 - y1)
            area = w * h

            if area < MIN_AREA:
                continue

            if confidence > best_confidence:
                best_confidence = confidence

                best_detection = {
                    "color": color_name,
                    "cx": (x1 + x2) // 2,
                    "cy": (y1 + y2) // 2,
                    "x": x1,
                    "y": y1,
                    "w": w,
                    "h": h,
                    "area": area,
                    "confidence": confidence
                }

    return best_detection


# ============================================================
# FLAG MISSION
# ============================================================

class FlagMissionPart:

    def __init__(self):

        self.car_x = None
        self.car_y = None
        self.running = True

        self.flag_active = False
        self.flag_steering = 0.0
        self.flag_throttle = 0.0

        self.GPS = "GPS"
        self.TRACKING = "TRACKING"
        self.HIT = "HIT"
        self.REVERSE = "REVERSE"

        self.state = self.GPS

        self.candidate_color = None
        self.candidate_frames = 0

        self.active_color = None
        self.lost_frames = 0

        self.hit_start_time = None
        self.reverse_start_time = None

        print("")
        print("====================================")
        print(" YOLO FLAG MISSION PART INITIALIZED")
        print(" GPS -> TRACK -> HIT -> REVERSE -> GPS")
        print("====================================")
        print("")
        print(f"YOLO MODEL: {YOLO_MODEL_PATH}")
        print(f"YOLO CONFIDENCE: {YOLO_CONFIDENCE}")
        print("")

        # ----------------------------------------------------
        # OAK-D CAMERA
        # ----------------------------------------------------

        self.pipeline = dai.Pipeline()

        camera = self.pipeline.createColorCamera()

        camera.setPreviewSize(
            IMAGE_WIDTH,
            IMAGE_HEIGHT
        )

        camera.setInterleaved(False)

        camera.setColorOrder(
            dai.ColorCameraProperties.ColorOrder.BGR
        )

        camera.setFps(30)

        xout = self.pipeline.createXLinkOut()

        xout.setStreamName(
            "flag_video"
        )

        camera.preview.link(
            xout.input
        )

        self.device = dai.Device(
            self.pipeline
        )

        self.camera_queue = self.device.getOutputQueue(
            name="flag_video",
            maxSize=4,
            blocking=False
        )

    # ========================================================
    # CAMERA THREAD
    # ========================================================

    def update(self):

        while self.running:

            img = self.camera_queue.tryGet()

            if img is None:
                time.sleep(0.005)
                continue

            frame = img.getCvFrame()

            detection = detect_flag(frame)

            if detection is not None:
                print(
                    f"YOLO DETECT {detection['color']} | "
                    f"conf={detection['confidence']:.2f} | "
                    f"area={int(detection['area'])} | "
                    f"cx={detection['cx']}"
                )

            self.process_detection(
                detection
            )

    # ========================================================
    # STEERING
    # ========================================================

    def steering_to_center(self, cx):

        error = cx - IMAGE_CENTER

        if abs(error) < CENTER_DEADBAND:
            return 0.0

        steering = (
            error * STEERING_GAIN
        )

        steering = max(
            -MAX_STEERING,
            min(
                MAX_STEERING,
                steering
            )
        )

        return steering

    # ========================================================
    # RETURN TO GPS
    # ========================================================

    def return_to_gps(self):

        self.flag_active = False
        self.flag_steering = 0.0
        self.flag_throttle = 0.0

        self.active_color = None
        self.candidate_color = None
        self.candidate_frames = 0
        self.lost_frames = 0

        self.hit_start_time = None
        self.reverse_start_time = None

        self.state = self.GPS

    # ========================================================
    # STATE MACHINE
    # ========================================================

    def process_detection(self, detection):

        # ----------------------------------------------------
        # GPS
        # ----------------------------------------------------

        if self.state == self.GPS:

            self.flag_active = False
            self.flag_steering = 0.0
            self.flag_throttle = 0.0

            if (
                detection is None
                or detection["area"] < TRIGGER_AREA
            ):
                self.candidate_color = None
                self.candidate_frames = 0
                return

            color = detection["color"]

            if color == self.candidate_color:
                self.candidate_frames += 1

            else:
                self.candidate_color = color
                self.candidate_frames = 1

            if (
                self.candidate_frames
                >= SPOTTED_FRAMES
            ):

                self.active_color = color

                print("")
                print("============================")
                print(
                    f"{self.active_color} FLAG SPOTTED"
                )
                print("============================")
                print("")

                speak(
                    f"{self.active_color.lower()} flag spotted"
                )

                self.flag_active = True
                self.state = self.TRACKING

                self.candidate_color = None
                self.candidate_frames = 0
                self.lost_frames = 0

            return

        # ----------------------------------------------------
        # TRACKING
        # ----------------------------------------------------

        elif self.state == self.TRACKING:

            self.flag_active = True

            if (
                detection is not None
                and detection["color"]
                == self.active_color
            ):

                self.lost_frames = 0

                cx = detection["cx"]
                area = detection["area"]

                steering = (
                    self.steering_to_center(cx)
                )

                self.flag_steering = steering
                self.flag_throttle = (
                    APPROACH_THROTTLE
                )

                print(
                    f"FLAG TRACK {self.active_color} | "
                    f"cx={cx} | "
                    f"steer={steering:.3f} | "
                    f"area={int(area)}"
                )

                if (
                    area >= HIT_START_AREA
                    and abs(
                        cx - IMAGE_CENTER
                    ) <= CENTER_DEADBAND
                ):

                    print("")
                    print("============================")
                    print(
                        f"HITTING {self.active_color} FLAG"
                    )
                    print("============================")
                    print("")

                    self.flag_steering = 0.0
                    self.flag_throttle = (
                        HIT_THROTTLE
                    )

                    self.lost_frames = 0
                    self.hit_start_time = (
                        time.time()
                    )

                    self.state = self.HIT

            else:

                self.lost_frames += 1
                self.flag_steering = 0.0
                self.flag_throttle = (
                    APPROACH_THROTTLE
                )

                if (
                    self.lost_frames
                    >= LOST_FRAMES_BEFORE_ABORT
                ):

                    print("")
                    print(
                        f"LOST {self.active_color} "
                        "BEFORE CONTACT"
                    )
                    print(
                        "RETURNING TO GPS"
                    )
                    print("")

                    self.return_to_gps()

            return

        # ----------------------------------------------------
        # HIT
        # ----------------------------------------------------

        elif self.state == self.HIT:

            self.flag_active = True
            self.flag_steering = 0.0
            self.flag_throttle = (
                HIT_THROTTLE
            )

            if (
                detection is not None
                and detection["color"]
                == self.active_color
            ):

                self.lost_frames = 0

            else:

                self.lost_frames += 1

            hit_elapsed = (
                time.time()
                - self.hit_start_time
                if self.hit_start_time
                is not None
                else 0.0
            )

            if (
                self.lost_frames
                >= LOST_FRAMES_FOR_KNOCKDOWN
                or hit_elapsed
                >= MAX_HIT_TIME
            ):

                print("")
                print("============================")
                print(
                    f"{self.active_color} FLAG DOWN"
                )
                print("REVERSING")
                print("============================")
                print("")

                speak(
                    f"{self.active_color.lower()} flag captured"
                )

                self.flag_steering = 0.0
                self.flag_throttle = (
                    REVERSE_THROTTLE
                )

                self.reverse_start_time = (
                    time.time()
                )

                self.state = self.REVERSE

            return

        # ----------------------------------------------------
        # REVERSE
        # ----------------------------------------------------

        elif self.state == self.REVERSE:

            self.flag_active = True
            self.flag_steering = 0.0
            self.flag_throttle = (
                REVERSE_THROTTLE
            )

            if (
                self.reverse_start_time
                is not None
                and time.time()
                - self.reverse_start_time
                >= REVERSE_TIME
            ):

                print("")
                print("============================")
                print("REVERSE COMPLETE")
                print("RETURNING CONTROL TO GPS")
                print("============================")
                print("")

                self.return_to_gps()

            return

    # ========================================================
    # DONKEYCAR THREADED OUTPUT
    # ========================================================

    def run_threaded(
        self,
        car_x,
        car_y
    ):

        self.car_x = car_x
        self.car_y = car_y

        return (
            self.flag_active,
            self.flag_steering,
            self.flag_throttle
        )

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def shutdown(self):

        print("")
        print(
            "Shutting down YOLO flag detector."
        )

        self.running = False

        try:
            self.device.close()

        except Exception:
            pass


# ============================================================
# GPS / FLAG CONTROL SELECTOR
# ============================================================

class FlagDriveSelector:

    def run(
        self,
        user_mode,
        gps_steering,
        gps_throttle,
        flag_active,
        flag_steering,
        flag_throttle
    ):

        # Flag controller is ONLY allowed to control
        # the vehicle during full autonomous LOCAL mode.

        if (
            user_mode == "local"
            and flag_active
        ):

            return (
                flag_steering,
                flag_throttle
            )

        return (
            gps_steering,
            gps_throttle
        )
