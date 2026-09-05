# Team 7 Autonomous GPS + YOLO Flag Detection Robot

UC San Diego MAE/ECE 148 Team 7 project integrating RTK GPS path following, DonkeyCar autonomous control, OAK-D flag detection, and audio announcement on a Raspberry Pi 5 vehicle.

## Project Information

**Project:** Autonomous Capture the Flag RoboCar  
**Course:** UC San Diego MAE/ECE 148  
**Team:** Team 7  
**Term:** Summer Session II 2026  
**Repository:** https://github.com/pkorti/capture_the_flag

### Team Members

| Team Member | Major |
|---|---|
| Maisarah Fahmy | ECE |
| Grisvian Tandy | ECE |
| Pragnya Korti | MAE |

## Project Goal

This project focuses on developing an autonomous RoboCar capable of searching a course for a designated colored flag, navigating toward it, and capturing it by driving through and knocking it down. Using an OAK-D camera with a YOLO flag color-detection model, the RoboCar searches for the target while completing GPS-guided laps. The system integrates perception, navigation, vehicle control, and synchronized audio feedback so that a speaker announces when the flag has been captured.

The overall goal is to combine **GPS-based global navigation** with **camera-based local flag detection and approach behavior** on a single autonomous vehicle.

## Project Requirements

### Must Haves

The final project requirements were to:

1. Autonomously complete GPS-guided laps while searching for the flag.
2. Detect the designated flag color using the OAK-D camera and YOLO.
3. Navigate toward the detected flag.
4. Drive through and knock down the flag.
5. Play a synchronized voice announcement after capture.
6. Use generative AI for code development, debugging, and integration.

### Nice to Haves

The team also identified the following stretch goals:

| Nice-to-have feature | Final project status |
|---|---|
| Return to the starting point after all captures and stop automatically | Not completed; return-to-home remained future work |
| Remember previously searched areas | Not completed; identified as a future search-behavior improvement |
| Detection across different lighting conditions, including night | Partially addressed; YOLO improved reliability across changing outdoor lighting, but full night performance was not established |
| Support multiple designated flag colors | Implemented in the detector with PINK, YELLOW, ORANGE, and BLUE classes |
| Add obstacle detection with LiDAR | Not implemented |

## Main Contributions

The project combines several subsystems into one autonomous capture-the-flag workflow:

- **RTK GPS path following:** Record a route and use DonkeyCar path following to complete autonomous laps.
- **YOLO flag detection:** Replace the original fixed-threshold OpenCV detector with a trained YOLO model for more robust flag recognition under changing outdoor conditions.
- **Autonomous flag approach:** Use the detected flag position and apparent size to steer toward the target and initiate the capture maneuver.
- **GPS / flag control arbitration:** Allow the flag mission to temporarily override normal GPS driving during a capture attempt.
- **Audio feedback:** Trigger a spoken announcement when the flag-capture action occurs.
- **Integrated testing and debugging:** Test navigation, detection, capture, and voice components separately before combining them into the final vehicle pipeline.

## System Overview

The system combines:

- Raspberry Pi 5
- DonkeyCar
- RTK GPS
- VESC motor/steering controller
- Logitech F710 gamepad
- OAK-D camera
- Jabra SPEAK 510 USB speaker
- GPS path recording and autonomous path following
- YOLO-based four-color flag detection
- Automatic flag approach, hit, reverse, and return to GPS

Mission flow:

```text
FOLLOW GPS PATH
      |
      v
YOLO DETECTS FLAG
      |
      v
TRACK / CENTER FLAG
      |
      v
APPROACH FLAG
      |
      v
   HIT FLAG
      |
      v
   REVERSE
      |
      v
RETURN TO GPS PATH
```

## Hardware

| Component | Purpose |
|---|---|
| Raspberry Pi 5 | Main onboard computer |
| Point One RTK GPS | Vehicle localization |
| VESC | Steering and throttle control |
| Logitech F710 | Manual driving and mission controls |
| OAK-D | Camera for YOLO flag detection |
| Jabra SPEAK 510 | Audio announcements |

Typical Team 7 device assignments:

```text
Point One runner:       /dev/ttyUSB0
DonkeyCar GPS NMEA:     /dev/ttyUSB1
VESC:                   /dev/ttyACM0
Logitech F710:          /dev/input/js0
```

Device numbering can change after reconnecting hardware.

---

## Software Environments

### Point One RTK Runner

```bash
cd ~/quectel/p1_runner
conda activate py37
python3 bin/runner.py --device-id <DEVICE_ID> --polaris <POLARIS_PASSWORD> --device-port /dev/ttyUSB0
```

Keep this terminal running while recording or following GPS paths.

### DonkeyCar

From another SSH terminal:

```bash
conda deactivate
source ~/env/bin/activate
cd ~/gpscar
python3 manage.py drive
```


---

## Important DonkeyCar Configuration

```python
GPS_SERIAL = "/dev/ttyUSB1"
GPS_SERIAL_BAUDRATE = 460800
GPS_DEBUG = True
HAVE_GPS = True
GPS_NMEA_PATH = None

DRIVE_TRAIN_TYPE = "VESC"
VESC_SERIAL_PORT = "/dev/ttyACM0"
VESC_BAUDRATE = 115200
VESC_MAX_SPEED_PERCENT = 0.2
VESC_STEERING_SCALE = 0.5
VESC_STEERING_OFFSET = 0.5

USE_JOYSTICK_AS_DEFAULT = True
JOYSTICK_MAX_THROTTLE = 0.2
CONTROLLER_TYPE = "F710"
JOYSTICK_DEVICE_FILE = "/dev/input/js0"

CAMERA_TYPE = "MOCK"
```

Working GPS path-following settings:

```python
PID_P = 0.15
PID_I = 0.0
PID_D = 0.30
PID_THROTTLE = 0.15

PATH_SEARCH_LENGTH = 10
PATH_LOOK_AHEAD = 1
PATH_LOOK_BEHIND = 1
```

These values are specific to Team 7's vehicle and test setup.

---

## Logitech F710 Controls

| Button | Function |
|---|---|
| B | Reset GPS origin |
| LB | Toggle path recording |
| RB | Save path |
| A | Load path |
| Y | Erase path |
| START | Change DonkeyCar driving mode |

The controller should be in **X mode** using the physical X/D switch.

---

## DonkeyCar Driving Modes

### `user`

Manual steering and throttle.

Use this mode to:

- Position the vehicle
- Reset GPS origin
- Record a GPS path
- Safely initialize the system

### `local_angle`

Autonomous steering is enabled.

The vehicle can steer automatically, but the user retains throttle control.

### `local`

Full autonomous operation.

The vehicle can automatically control both steering and throttle.

The flag controller is only allowed to override GPS control while the vehicle is in `local`.

---

## Recording a GPS Path

Start the Point One runner first.

Then start DonkeyCar:

```bash
cd ~/gpscar
python3 manage.py drive
```

Remain in `user` mode.

At the exact physical starting point:

1. Press **B once** to reset the GPS origin.
2. Verify `pos/x` and `pos/y` are near zero.
3. Press **LB once** to begin recording.
4. Manually drive one smooth lap.
5. Return to the starting point.
6. Press **LB once** to stop recording.
7. Press **RB once** to save the path.

The path is saved as:

```text
~/gpscar/donkey_path.csv
```

The current successful Team 7 path contains approximately **352 points**.

---

## Running the Saved GPS Path

Place the vehicle at the same physical starting point used during recording.

While in `user` mode:

1. Press **B once** at the physical start point.
2. Verify `pos/x` and `pos/y` are near zero.
3. Make sure the saved path is loaded.
4. Press **START once** to enter `local_angle`.
5. Verify autonomous steering behaves correctly.
6. Press **START again** to enter `local`.

The car should now follow the recorded GPS path.

---

# YOLO Flag Detection

The flag detector uses the OAK-D camera and a YOLO model.

The mission contains four flag classes:

```text
PINK
YELLOW
ORANGE
BLUE
```

YOLO replaces the previous HSV color-thresholding and contour-based OpenCV detector.

The new architecture is:

```text
OAK-D CAMERA
     |
     v
YOLO MODEL
     |
     v
CLASS + CONFIDENCE + BOUNDING BOX
     |
     v
FLAG DETECTION DICTIONARY
     |
     v
FlagMissionPart
     |
     v
TRACK -> HIT -> REVERSE
     |
     v
FlagDriveSelector
     |
     v
VESC
```

## YOLO Output Format

The important part of the YOLO integration is converting each detection into the same format already expected by `FlagMissionPart`.

```python
{
    "color": color_name,
    "cx": center_x,
    "cy": center_y,
    "x": x1,
    "y": y1,
    "w": x2 - x1,
    "h": y2 - y1,
    "area": (x2 - x1) * (y2 - y1)
}
```

Because the rest of the mission already consumes this format, the GPS and flag-control logic does not need to be rewritten.

---

## YOLO Detection Logic

A starting confidence threshold is:

```python
YOLO_CONFIDENCE = 0.50
```

Conceptually:

```python
def detect_flag(frame):
    results = model(frame)

    best_detection = None
    best_confidence = 0.0

    for result in results:
        for box in result.boxes:

            confidence = float(box.conf[0])

            if confidence < YOLO_CONFIDENCE:
                continue

            class_id = int(box.cls[0])

            color_name = model.names[class_id].upper()

            if color_name not in {
                "PINK",
                "YELLOW",
                "ORANGE",
                "BLUE"
            }:
                continue

            x1, y1, x2, y2 = map(
                int,
                box.xyxy[0].tolist()
            )

            if confidence > best_confidence:

                best_confidence = confidence

                best_detection = {
                    "color": color_name,
                    "cx": (x1 + x2) // 2,
                    "cy": (y1 + y2) // 2,
                    "x": x1,
                    "y": y1,
                    "w": x2 - x1,
                    "h": y2 - y1,
                    "area": (x2 - x1) * (y2 - y1)
                }

    return best_detection
```

The exact model-loading code depends on the exported YOLO model format, such as:

```text
.pt
.onnx
.blob
```

---

# Flag Mission State Machine

The YOLO model handles detection only.

The existing flag mission handles vehicle behavior:

```text
GPS
 |
 | flag detected
 v
TRACKING
 |
 | close + centered
 v
HIT
 |
 | flag disappears / timeout
 v
REVERSE
 |
 | reverse complete
 v
GPS
```

## Mission Parameters

```python
MIN_AREA = 250
TRIGGER_AREA = 1000
SPOTTED_FRAMES = 3

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER = 320
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
REVERSE_TIME = 3

YOLO_CONFIDENCE = 0.50

SPEAKER_DEVICE = "plughw:2,0"
```

YOLO bounding-box area replaces the old OpenCV contour area:

```python
area = (x2 - x1) * (y2 - y1)
```

This means the existing distance/approach logic can still use apparent flag size.

---

# Flag Override Integration

Current integrated flag logic:

```text
~/gpscar/flag_override_new.py
```

`manage.py` imports:

```python
from flag_override_new import FlagMissionPart, FlagDriveSelector
```

The mission part is added to the DonkeyCar pipeline:

```python
flag_mission = FlagMissionPart()

V.add(
    flag_mission,
    inputs=['pos/x', 'pos/y'],
    outputs=[
        'flag/active',
        'flag/steering',
        'flag/throttle'
    ],
    threaded=True
)
```

The selector receives both GPS control and flag control:

```python
V.add(
    FlagDriveSelector(),
    inputs=[
        'user/mode',
        'base/steering',
        'base/throttle',
        'flag/active',
        'flag/steering',
        'flag/throttle'
    ],
    outputs=['steering', 'throttle']
)
```

Selector behavior:

```python
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
```

Therefore:

```text
NO FLAG
   |
   v
GPS CONTROLS CAR


FLAG ACTIVE + LOCAL MODE
   |
   v
FLAG CONTROLLER OVERRIDES GPS
```

When the flag mission finishes reversing:

```text
flag_active = False
```

and control automatically returns to GPS.

---

# Audio Output

The Jabra SPEAK 510 is used for flag announcements.

Example:

```bash
espeak --stdout "blue flag captured" | aplay -D plughw:2,0
```

Speaker device:

```text
plughw:2,0
```

The ALSA device number can change after reconnecting USB hardware.

---


# Project Results

The sections above describe the implemented system and its intended mission flow. Final outdoor testing showed that several major subsystems worked, while full capture-and-path-recovery behavior was still sensitive to real-world conditions.

## What Worked

### GPS Path Following

The RoboCar successfully followed the recorded GPS route and completed autonomous laps when the path was not interrupted.

### Autonomous Search / Flag Approach

While completing GPS-guided laps, the RoboCar searched for the flag. Once YOLO detected a target, the integrated control logic could change steering toward the detected flag.

### YOLO Flag Detection

YOLO flag detection was more reliable across changing outdoor lighting conditions than the team's original OpenCV color-thresholding approach. The original method depended on fixed color thresholds, which were too sensitive to sunlight, shadows, and other environmental changes.

### Voice Integration

The combined scripts successfully triggered a voice announcement when the RoboCar performed the flag-capture action.

## Challenges and What Did Not Work Reliably

### OpenCV Color Detection

The original OpenCV detector used fixed color thresholds and did not detect the flags reliably enough under real outdoor conditions. It was replaced by the YOLO model.

### Changing Outdoor Lighting

Sunlight, shadows, weather, and time-of-day changes altered the appearance of the flags throughout testing and reduced detection consistency.

### Camera Configuration

The OAK-D camera was sometimes overexposed or produced inconsistent resolution/configuration behavior, which made reliable detection and capture more difficult.

### Driving Oscillation and RTK Fix

When the vehicle lost a stable RTK fix, it could oscillate or stray away from the intended course. Even with PID tuning, this could cause the target flag to leave the camera's field of view.

### Physical Flag Design

The physical flag had to resist wind while still being easy enough for the RoboCar to knock down. The original supports were not consistently stable, so the mechanical design also affected capture reliability.

### Capture and GPS Path Recovery

Driving through the flag changed the RoboCar's heading. Although the software architecture returns control to GPS after the capture sequence, final testing showed that the vehicle did **not consistently recover back onto its original GPS path after capture**.

This is an important distinction between the implemented control flow and the reliability observed during final outdoor testing.

# Lessons Learned

The final project produced four main takeaways:

1. **Real-world conditions matter.** Lighting, weather, GPS accuracy, camera behavior, and physical course conditions significantly affected autonomous performance.
2. **Test components separately.** Testing navigation, detection, voice, and capture independently made integration and debugging easier.
3. **Choose robust solutions.** YOLO detection was more reliable than fixed OpenCV color thresholds for this outdoor application.
4. **Prioritize reliability.** Consistent performance in real conditions was more valuable than perfect results under controlled conditions.

# Future Improvements

With additional development time, the team would focus on:

- Expanding the YOLO training dataset with images collected under different lighting conditions, distances, and viewing angles.
- Improving OAK-D exposure and resolution settings.
- Improving the RoboCar's search behavior so it can scan more of the surrounding environment.
- Tracking previously searched areas to reduce repeated searching.
- Further tuning the GPS PID controller.
- Adjusting throttle during sharp corrections and adding steering smoothing to reduce oscillation.
- Improving search and approach logic so the vehicle can recover from temporary missed detections.
- Redesigning the physical capture mechanism and flag supports for better wind resistance and repeatability.
- Performing additional full-system outdoor testing.
- Developing reliable return-to-home behavior after the mission is complete.

---

# Troubleshooting

## Huge CTE When Starting Autonomous Mode

If GPS positions look like:

```text
477976, 3638157
```

while the saved path contains coordinates near:

```text
0, 0
```

the GPS origin was not reset.

Return to `user`, place the vehicle at the original starting point, and press:

```text
B
```

once.

Then verify `pos/x` and `pos/y` are near zero before entering autonomous mode.

---

## Car Detects Flag but Stops Moving

The flag controller replaces GPS throttle while tracking.

If:

```python
APPROACH_THROTTLE
```

is below the drivetrain's usable throttle range, the vehicle may detect and steer toward the flag but not physically move.

The current value is:

```python
APPROACH_THROTTLE = 0.18
```

---

## Debugging Flag Detection

A useful temporary debug message inside the camera thread is:

```python
if detection is not None:
    print(
        f"CAM DETECT {detection['color']} | "
        f"area={int(detection['area'])} | "
        f"cx={detection['cx']}"
    )
```

Expected output:

```text
CAM DETECT BLUE | area=15447 | cx=289
```

This verifies that the camera and detector are producing detections before the vehicle-control logic is considered.

---

## F710 Stops Working

Check the physical **X/D** switch.

Team 7 uses:

```text
X mode
```

The controller normally appears at:

```text
/dev/input/js0
```

---

## OAK-D / DepthAI API Errors

The DonkeyCar `(env)` environment uses the DepthAI 2.x API.

Use:

```bash
conda deactivate
source ~/env/bin/activate
cd ~/gpscar
```

before running the integrated camera/DonkeyCar software.

---

# Repository Structure

```text
gpscar/
├── manage.py
├── myconfig.py
├── flag_override.py
├── flag_override_new.py
├── donkey_path.csv
└── models/
    └── <YOLO_MODEL_FILE>
```

---

# Steps

- [x] Point One RTK GPS receiving corrections
- [x] GPS position available inside DonkeyCar
- [x] F710 manual vehicle control
- [x] GPS origin reset
- [x] GPS path recording and saving
- [x] Autonomous GPS path following
- [x] OAK-D camera integration
- [x] Four mission flag classes: PINK, YELLOW, ORANGE, BLUE
- [x] Flag approach / hit / reverse state machine
- [x] Jabra audio announcements
- [x] GPS/flag control selector integration
- [x] Install/export final YOLO model on Raspberry Pi
- [x] Replace HSV `detect_flag()` with YOLO inference
- [x] Validate YOLO detections inside `manage.py`
- [x] Final end-to-end GPS + YOLO flag mission validation


