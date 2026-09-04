import cv2
import depthai as dai
import numpy as np
import subprocess
import time


# ============================================================
# FLAG SETTINGS
# ============================================================

MIN_AREA = 400
SPOTTED_FRAMES = 5

CAPTURE_AREA = 30000
LOST_FRAMES_FOR_CAPTURE = 8
LOST_FRAMES_BEFORE_ABORT = 30

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER = IMAGE_WIDTH // 2

CENTER_DEADBAND = 35

APPROACH_THROTTLE = 0.10
CAPTURE_THROTTLE = 0.18

STEERING_GAIN = 0.0020
MAX_STEERING = 0.35

# Amount of time to continue straight after capture
DRIVE_THROUGH_TIME = 1.5

SPEAKER_DEVICE = "plughw:2,0"


# ============================================================
# COLOR RANGES
# ============================================================

COLOR_RANGES = {

    "RED": [
        (
            np.array([0, 100, 100]),
            np.array([10, 255, 255])
        ),
        (
            np.array([170, 100, 100]),
            np.array([180, 255, 255])
        )
    ],

    "ORANGE": [
        (
            np.array([10, 100, 100]),
            np.array([25, 255, 255])
        )
    ],

    "YELLOW": [
        (
            np.array([20, 70, 70]),
            np.array([42, 255, 255])
        )
    ],

    "BLUE": [
        (
            np.array([95, 100, 80]),
            np.array([130, 255, 255])
        )
    ],

    "PURPLE": [
        (
            np.array([130, 70, 70]),
            np.array([155, 255, 255])
        )
    ],

    "PINK": [
        (
            np.array([155, 70, 80]),
            np.array([170, 255, 255])
        )
    ]
}


# ============================================================
# SPEECH
# ============================================================

def speak(text):

    print(f"SPEAKING: {text}")

    try:
        espeak_process = subprocess.Popen(
            [
                "espeak",
                "--stdout",
                text
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )

        subprocess.Popen(
            [
                "aplay",
                "-D",
                SPEAKER_DEVICE
            ],
            stdin=espeak_process.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        espeak_process.stdout.close()

    except Exception as e:
        print("Speech error:", e)


# ============================================================
# MASK
# ============================================================

def make_mask(hsv, ranges):

    mask = np.zeros(
        hsv.shape[:2],
        dtype=np.uint8
    )

    for lower, upper in ranges:

        color_mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        mask = cv2.bitwise_or(
            mask,
            color_mask
        )

    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    return mask


# ============================================================
# DETECTION
# ============================================================

def detect_flag(frame):

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    best_detection = None
    best_area = 0

    for color_name, ranges in COLOR_RANGES.items():

        mask = make_mask(
            hsv,
            ranges
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < MIN_AREA:
                continue

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

            if len(approx) != 3:
                continue


            if area > best_area:

                x, y, w, h = cv2.boundingRect(contour)

                cx = x + (w // 2)
                cy = y + (h // 2)

                best_area = area

                best_detection = {
                    "color": color_name,
                    "cx": cx,
                    "cy": cy,
                    "area": area
                }

    return best_detection


# ============================================================
# FLAG MISSION PART
# ============================================================

class FlagMissionPart:

    def __init__(self):

        self.running = True

        self.flag_active = False
        self.flag_steering = 0.0
        self.flag_throttle = 0.0

        self.SEARCHING = "SEARCHING"
        self.TRACKING = "TRACKING"
        self.CLOSE_TO_FLAG = "CLOSE_TO_FLAG"
        self.DRIVE_THROUGH = "DRIVE_THROUGH"

        self.state = self.SEARCHING

        self.candidate_color = None
        self.candidate_frames = 0

        self.active_color = None
        self.lost_frames = 0

        self.captured_flags = set()

        self.drive_through_start = None

        print("")
        print("====================================")
        print(" FLAG MISSION PART INITIALIZED")
        print("====================================")
        print("")

        # --------------------------------------------
        # DepthAI 2.x pipeline
        # --------------------------------------------

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
        xout.setStreamName("flag_video")

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
    # THREADED LOOP
    # ========================================================

    def update(self):

        while self.running:

            img = self.camera_queue.tryGet()

            if img is None:
                time.sleep(0.005)
                continue

            frame = img.getCvFrame()

            detection = detect_flag(
                frame
            )

            self.process_detection(
                detection
            )


    # ========================================================
    # STATE MACHINE
    # ========================================================

    def process_detection(self, detection):

        # ----------------------------------------------------
        # SEARCHING
        # ----------------------------------------------------

        if self.state == self.SEARCHING:

            self.flag_active = False
            self.flag_steering = 0.0
            self.flag_throttle = 0.0

            if detection is None:

                self.candidate_color = None
                self.candidate_frames = 0
                return


            color = detection["color"]


            if color in self.captured_flags:

                self.candidate_color = None
                self.candidate_frames = 0
                return


            if color == self.candidate_color:

                self.candidate_frames += 1

            else:

                self.candidate_color = color
                self.candidate_frames = 1


            if self.candidate_frames >= SPOTTED_FRAMES:

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


        # ----------------------------------------------------
        # TRACKING
        # ----------------------------------------------------

        elif self.state == self.TRACKING:

            self.flag_active = True


            if (
                detection is not None
                and
                detection["color"] == self.active_color
            ):

                self.lost_frames = 0

                cx = detection["cx"]
                area = detection["area"]

                error = (
                    cx -
                    IMAGE_CENTER
                )


                if abs(error) < CENTER_DEADBAND:

                    steering = 0.0

                else:

                    steering = (
                        error *
                        STEERING_GAIN
                    )


                steering = max(
                    -MAX_STEERING,
                    min(
                        MAX_STEERING,
                        steering
                    )
                )


                self.flag_steering = steering
                self.flag_throttle = APPROACH_THROTTLE


                print(
                    f"FLAG APPROACH {self.active_color} | "
                    f"cx={cx} | "
                    f"steer={steering:.3f} | "
                    f"area={int(area)}"
                )


                if area >= CAPTURE_AREA:

                    print("")
                    print("============================")
                    print(
                        f"CLOSE TO {self.active_color}"
                    )
                    print("============================")
                    print("")

                    self.state = self.CLOSE_TO_FLAG
                    self.lost_frames = 0


            else:

                self.lost_frames += 1

                # Brief dropout: creep straight
                self.flag_steering = 0.0
                self.flag_throttle = APPROACH_THROTTLE


                if (
                    self.lost_frames
                    >=
                    LOST_FRAMES_BEFORE_ABORT
                ):

                    print("")
                    print(
                        f"LOST {self.active_color} FLAG"
                    )
                    print("RETURNING TO GPS")
                    print("")

                    self.flag_active = False
                    self.flag_steering = 0.0
                    self.flag_throttle = 0.0

                    self.active_color = None
                    self.lost_frames = 0

                    self.state = self.SEARCHING


        # ----------------------------------------------------
        # CLOSE / CAPTURE
        # ----------------------------------------------------

        elif self.state == self.CLOSE_TO_FLAG:

            self.flag_active = True


            if (
                detection is not None
                and
                detection["color"] == self.active_color
            ):

                self.lost_frames = 0

                cx = detection["cx"]

                error = (
                    cx -
                    IMAGE_CENTER
                )


                if abs(error) < CENTER_DEADBAND:

                    steering = 0.0

                else:

                    steering = (
                        error *
                        STEERING_GAIN
                    )


                steering = max(
                    -MAX_STEERING,
                    min(
                        MAX_STEERING,
                        steering
                    )
                )


                self.flag_steering = steering
                self.flag_throttle = CAPTURE_THROTTLE


                print(
                    f"FLAG CAPTURE {self.active_color} | "
                    f"cx={cx} | "
                    f"steer={steering:.3f} | "
                    f"area={int(detection['area'])}"
                )


            else:

                self.lost_frames += 1

                self.flag_steering = 0.0
                self.flag_throttle = CAPTURE_THROTTLE


                if (
                    self.lost_frames
                    >=
                    LOST_FRAMES_FOR_CAPTURE
                ):

                    print("")
                    print("============================")
                    print(
                        f"{self.active_color} FLAG CAPTURED"
                    )
                    print("============================")
                    print("")

                    speak(
                        f"{self.active_color.lower()} flag captured"
                    )

                    self.captured_flags.add(
                        self.active_color
                    )

                    self.drive_through_start = time.time()

                    self.state = self.DRIVE_THROUGH


        # ----------------------------------------------------
        # DRIVE THROUGH
        # ----------------------------------------------------

        elif self.state == self.DRIVE_THROUGH:

            self.flag_active = True

            self.flag_steering = 0.0
            self.flag_throttle = CAPTURE_THROTTLE


            if (
                time.time()
                -
                self.drive_through_start
                >=
                DRIVE_THROUGH_TIME
            ):

                print("")
                print(
                    "FLAG CAPTURE COMPLETE"
                )
                print(
                    "RETURNING CONTROL TO GPS"
                )
                print("")

                print(
                    "Captured flags:",
                    self.captured_flags
                )

                self.flag_active = False
                self.flag_steering = 0.0
                self.flag_throttle = 0.0

                self.active_color = None
                self.lost_frames = 0

                self.state = self.SEARCHING


    # ========================================================
    # VEHICLE OUTPUT
    # ========================================================

    def run_threaded(self):

        return (
            self.flag_active,
            self.flag_steering,
            self.flag_throttle
        )


    def shutdown(self):

        print("")
        print("Shutting down flag detector.")

        self.running = False

        try:
            self.device.close()
        except Exception:
            pass


# ============================================================
# DRIVE SELECTOR
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

        # IMPORTANT:
        # Flag override ONLY happens in full autonomous mode.
        #
        # Manual "user" mode remains untouched.
        # local_angle remains untouched.
        # Only "local" can activate flag driving.

        if (
            user_mode == "local"
            and
            flag_active
        ):

            return (
                flag_steering,
                flag_throttle
            )


        return (
            gps_steering,
            gps_throttle
        )
