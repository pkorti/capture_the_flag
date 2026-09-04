# Team 7 Autonomous GPS + YOLO Flag Detection Robot

UC San Diego MAE/ECE 148 Team 7 project integrating RTK GPS path following, DonkeyCar autonomous control, OAK-D flag detection, and audio feedback on a Raspberry Pi 5 vehicle.

## System Overview

The system combines:

- Raspberry Pi 5
- DonkeyCar
- Point One Navigation RTK GPS
- VESC motor/steering controller
- Logitech F710 gamepad
- Luxonis OAK-D camera
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

# Security

Do **not** commit:

- Point One device credentials
- Polaris passwords
- API keys
- Roboflow API keys
- Private tokens
- Other team or infrastructure secrets

Example `.gitignore`:

```gitignore
.env
*.log
__pycache__/
*.pyc
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

# Team

**UC San Diego MAE/ECE 148 - Team 7**
