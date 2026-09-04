import depthai as dai
import subprocess
import time

from ultralytics import YOLO


# ============================================================
# SETTINGS
# ============================================================

TRIGGER_AREA = 1000
SPOTTED_FRAMES = 3

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER = IMAGE_WIDTH // 2

CENTER_DEADBAND = 35

APPROACH_THROTTLE = 0.18
HIT_THROTTLE = 0.18

# Stop for 3 seconds immediately before hitting the flag
STOP_TIME = 3.0

REVERSE_THROTTLE = -0.15
REVERSE_TIME = 3.0

STEERING_GAIN = 0.0020
MAX_STEERING = 0.35

# When the YOLO bounding box reaches this size and is centered,
# stop for 3 seconds before hitting.
HIT_START_AREA = 30000

LOST_FRAMES_FOR_KNOCKDOWN = 8
MAX_HIT_TIME = 2.0

LOST_FRAMES_BEFORE_ABORT = 30

SPEAKER_DEVICE = "plughw:2,0"


# ============================================================
# YOLO
# ============================================================

YOLO_MODEL_PATH = "/home/team7/gpscar/weights.pt"

YOLO_CONF = 0.25

YOLO_MODEL = YOLO(YOLO_MODEL_PATH)


# ============================================================
# SPEAKER
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
# YOLO FLAG DETECTION
#
# Model classes:
#
# 0 = blue_flag
# 1 = orange_flag
# 2 = pink_flag
# 3 = yellow_flag
#
# If multiple detections are visible, use the largest box.
# ============================================================

def detect_flag(frame):

    results = YOLO_MODEL.predict(
        source=frame,
        conf=YOLO_CONF,
        verbose=False
    )

    best_detection = None
    best_area = 0

    for result in results:

        if result.boxes is None:
            continue

        for box in result.boxes:

            confidence = float(box.conf[0])

            if confidence < YOLO_CONF:
                continue

            class_id = int(box.cls[0])
            class_name = YOLO_MODEL.names[class_id]

            color = (
                class_name
                .replace("_flag", "")
                .upper()
            )

            x1, y1, x2, y2 = (
                box.xyxy[0]
                .cpu()
                .numpy()
            )

            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            w = max(0, x2 - x1)
            h = max(0, y2 - y1)

            cx = x1 + (w // 2)
            cy = y1 + (h // 2)

            area = w * h

            if area > best_area:

                best_area = area

                best_detection = {
                    "color": color,
                    "cx": cx,
                    "cy": cy,
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
        self.STOP = "STOP"
        self.HIT = "HIT"
        self.REVERSE = "REVERSE"

        self.state = self.GPS

        self.candidate_color = None
        self.candidate_frames = 0

        self.active_color = None
        self.lost_frames = 0

        self.stop_start_time = None
        self.hit_start_time = None
        self.reverse_start_time = None

        print("")
        print("====================================")
        print(" FLAG MISSION PART INITIALIZED")
        print(" YOLO DETECTOR ACTIVE")
        print(" GPS -> TRACK -> STOP -> HIT -> REVERSE -> GPS")
        print("====================================")
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
        camera.initialControl.setManualExposure(2000, 400)

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

            self.process_detection(detection)


    # ========================================================
    # STEERING
    # ========================================================

    def steering_to_center(self, cx):

        error = cx - IMAGE_CENTER

        if abs(error) < CENTER_DEADBAND:

            return 0.0

        steering = error * STEERING_GAIN

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

        self.stop_start_time = None
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

                steering = self.steering_to_center(cx)

                self.flag_steering = steering
                self.flag_throttle = APPROACH_THROTTLE

                print(
                    f"FLAG TRACK {self.active_color} | "
                    f"cx={cx} | "
                    f"steer={steering:.3f} | "
                    f"area={int(area)}"
                )

                if (
                    area >= HIT_START_AREA
                ):

                    print("")
                    print("============================")
                    print(
                        f"STOPPING BEFORE {self.active_color} FLAG"
                    )
                    print("WAITING 3 SECONDS")
                    print("============================")
                    print("")

                    self.flag_steering = 0.0
                    self.flag_throttle = 0.0

                    self.stop_start_time = time.time()

                    self.state = self.STOP

            else:

                self.lost_frames += 1

                self.flag_steering = 0.0
                self.flag_throttle = APPROACH_THROTTLE

                if (
                    self.lost_frames
                    >= LOST_FRAMES_BEFORE_ABORT
                ):

                    print("")
                    print(
                        f"LOST {self.active_color} "
                        "BEFORE CONTACT"
                    )
                    print("RETURNING TO GPS")
                    print("")

                    self.return_to_gps()

            return


        # ----------------------------------------------------
        # STOP FOR 3 SECONDS
        # ----------------------------------------------------

        elif self.state == self.STOP:

            self.flag_active = True

            self.flag_steering = 0.0
            self.flag_throttle = 0.0

            if (
                self.stop_start_time is not None
                and (
                    time.time()
                    - self.stop_start_time
                ) >= STOP_TIME
            ):

                print("")
                print("============================")
                print("3 SECOND STOP COMPLETE")
                print(
                    f"HITTING {self.active_color} FLAG"
                )
                print("============================")
                print("")

                self.flag_steering = 0.0
                self.flag_throttle = HIT_THROTTLE

                self.lost_frames = 0

                self.hit_start_time = time.time()

                self.state = self.HIT

            return


        # ----------------------------------------------------
        # HIT
        # ----------------------------------------------------

        elif self.state == self.HIT:

            self.flag_active = True

            self.flag_steering = 0.0
            self.flag_throttle = HIT_THROTTLE

            if (
                detection is not None
                and detection["color"]
                == self.active_color
            ):

                self.lost_frames = 0

            else:

                self.lost_frames += 1

            if self.hit_start_time is not None:

                hit_elapsed = (
                    time.time()
                    - self.hit_start_time
                )

            else:

                hit_elapsed = 0.0

            if (
                self.lost_frames
                >= LOST_FRAMES_FOR_KNOCKDOWN
                or hit_elapsed >= MAX_HIT_TIME
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
                    f"{self.active_color.lower()} "
                    "flag captured"
                )

                self.flag_steering = 0.0
                self.flag_throttle = REVERSE_THROTTLE

                self.reverse_start_time = time.time()

                self.state = self.REVERSE

            return


        # ----------------------------------------------------
        # REVERSE
        # ----------------------------------------------------

        elif self.state == self.REVERSE:

            self.flag_active = True

            self.flag_steering = 0.0
            self.flag_throttle = REVERSE_THROTTLE

            if (
                self.reverse_start_time is not None
                and (
                    time.time()
                    - self.reverse_start_time
                ) >= REVERSE_TIME
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
    # DONKEYCAR OUTPUT
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
        print("Shutting down flag detector.")

        self.running = False

        try:

            self.device.close()

        except Exception:

            pass


# ============================================================
# GPS / FLAG SELECTOR
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

        # Flag controller can ONLY override GPS
        # during full autonomous LOCAL mode.

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
