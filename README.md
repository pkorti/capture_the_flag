# Team 7 Autonomous GPS + Flag Detection Robot

UC San Diego MAE/ECE 148 Team 7 project integrating RTK GPS path
following, DonkeyCar autonomous control, OAK-D flag detection, and audio
feedback on a Raspberry Pi 5 vehicle.

> **Safety:** This repository controls a physical vehicle. Always test
> with adequate clearance, keep an emergency stop method available, and
> verify the current DonkeyCar mode before applying throttle.
> `local_angle` can command steering. `local` can command both steering
> and throttle.

## System Overview

The system combines:

-   Raspberry Pi 5
-   DonkeyCar
-   Point One Navigation RTK GPS
-   VESC motor/steering controller
-   Logitech F710 gamepad
-   Luxonis OAK-D camera
-   Jabra SPEAK 510 USB speaker
-   GPS path recording and autonomous path following
-   Yellow triangular flag detection and temporary flag-approach
    override

Mission flow:

``` text
FOLLOW GPS PATH
      |
      v
FLAG SPOTTED
      |
      v
APPROACH FLAG
      |
      v
CAPTURE FLAG
      |
      v
DRIVE THROUGH
      |
      v
RETURN TO GPS PATH
```

The flag system does not directly command the VESC. It supplies steering
and throttle values to a selector that temporarily overrides the GPS
follower when a flag mission is active.

## Hardware

  Component           Purpose
  ------------------- -------------------------------------
  Raspberry Pi 5      Main onboard computer
  Point One RTK GPS   Vehicle localization
  VESC                Steering and throttle control
  Logitech F710       Manual driving and mission controls
  OAK-D               Flag detection
  Jabra SPEAK 510     Audio announcements

Typical Team 7 device assignments:

``` text
Point One runner:       /dev/ttyUSB0
DonkeyCar GPS NMEA:     /dev/ttyUSB1
VESC:                   /dev/ttyACM0
Logitech F710:          /dev/input/js0
```

Device numbering can change after reconnecting hardware.

## Software Environments

### Point One RTK runner

``` bash
cd ~/quectel/p1_runner
conda activate py37
python3 bin/runner.py --device-id <DEVICE_ID> --polaris <POLARIS_PASSWORD> --device-port /dev/ttyUSB0
```

Keep this terminal running while recording or following GPS paths.

**Do not commit Point One credentials to GitHub.** Store them privately
or use environment variables/secrets.

A healthy RTK connection should show a valid RTK solution, such as
`RTKFloat`, and incoming correction data.

### DonkeyCar

From another SSH terminal:

``` bash
conda deactivate
source ~/env/bin/activate
cd ~/gpscar
python3 manage.py drive
```

Starting `manage.py drive` activates the vehicle control software. Keep
the vehicle in `user` mode until intentionally beginning an autonomous
test.

## Important DonkeyCar Configuration

``` python
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

``` python
PID_P = 0.15
PID_I = 0.0
PID_D = 0.30
PID_THROTTLE = 0.15

PATH_SEARCH_LENGTH = 10
PATH_LOOK_AHEAD = 1
PATH_LOOK_BEHIND = 1
```

These values are specific to Team 7's vehicle and test setup. Retune
cautiously if the mechanical configuration, speed, course, or steering
geometry changes.

## Logitech F710 Controls

  Button   Function
  -------- -------------------------------
  B        Reset GPS origin
  LB       Toggle path recording
  RB       Save path
  A        Load path
  Y        Erase path
  START    Change DonkeyCar driving mode

The controller should be in **X mode** using the physical X/D switch.

## DonkeyCar Driving Modes

### `user`

Manual steering and throttle. Use this to position the vehicle, reset
the GPS origin, and record the course.

### `local_angle`

Autonomous steering is enabled.

**The steering can move automatically.** Use this as an intermediate
safety check before autonomous throttle.

### `local`

Full autonomous operation.

**The vehicle can steer and drive forward automatically.**

START cycles through the configured modes. Confirm the displayed mode
instead of assuming which mode is active.

## Recording a GPS Path

Start the Point One runner first and leave it running. Then start
DonkeyCar:

``` bash
cd ~/gpscar
python3 manage.py drive
```

Remain in `user` mode.

At the exact physical starting point:

1.  Press **B once** to reset the GPS origin.
2.  Verify `pos/x` and `pos/y` are near zero.
3.  Press **LB once** to begin recording.
4.  Manually drive one smooth lap.
5.  Return to the starting point and stop.
6.  Press **LB once** to stop recording.
7.  Press **RB once** to save the path.

The path is saved as:

``` text
~/gpscar/donkey_path.csv
```

A successful Team 7 recording contained approximately 352 points. The
exact number can vary.

A safe way to check the saved point count from another terminal is:

``` bash
wc -l ~/gpscar/donkey_path.csv
```

## Running the Saved GPS Path

Place the vehicle at the same physical starting point used during
recording.

While in `user` mode:

1.  Press **B once** at the physical start point.
2.  Confirm `pos/x` and `pos/y` are near zero.
3.  Make sure the saved path is loaded. Use **A** when a manual path
    load is needed.
4.  Press **START once** to enter `local_angle`.
5.  Check that autonomous steering behaves reasonably.
6.  Press **START again** to enter `local` only when the area is clear
    and the vehicle is ready for autonomous motion.

## Why the B Reset Matters

The GPS receiver reports absolute UTM coordinates, while the recorded
DonkeyCar path is origin-relative.

Pressing **B** while `manage.py` is running resets DonkeyCar's local
origin. The Point One terminal will continue displaying absolute
coordinates. That is normal.

Always reset the origin at the same physical starting point before
autonomous playback.

## Flag Detection

The current integrated detector is configured for a **yellow triangular
flag**.

Example HSV range:

``` python
COLOR_RANGES = {
    "YELLOW": [
        (
            np.array([25, 100, 100]),
            np.array([38, 255, 255])
        )
    ]
}
```

Triangle filtering:

``` python
peri = cv2.arcLength(contour, True)
approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

if len(approx) != 3:
    continue
```

## Flag Mission Parameters

``` python
MIN_AREA = 800
SPOTTED_FRAMES = 5
CAPTURE_AREA = 30000
LOST_FRAMES_FOR_CAPTURE = 8
LOST_FRAMES_BEFORE_ABORT = 30

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER = 320
CENTER_DEADBAND = 35

APPROACH_THROTTLE = 0.10
CAPTURE_THROTTLE = 0.18
STEERING_GAIN = 0.0020
MAX_STEERING = 0.35
DRIVE_THROUGH_TIME = 1.5

SPEAKER_DEVICE = "plughw:2,0"
```

## Flag Override Architecture

Integrated flag logic is in:

``` text
~/gpscar/flag_override.py
```

The selector behavior is:

``` python
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
```

The flag mission therefore overrides GPS steering/throttle only when
DonkeyCar is in `local` mode and a flag mission is active. Otherwise,
GPS retains control.

## Audio Output

The Jabra SPEAK 510 can be tested with:

``` bash
espeak --stdout "yellow flag captured" | aplay -D plughw:2,0
```

The ALSA device number can change after USB devices are reconnected.

## Troubleshooting

### `path is none; cannot calculate nearest points`

If the terminal repeatedly shows:

``` text
ERROR:root:path is none; cannot calculate nearest points
INFO:root:no nearest point ...
INFO:root:CTE: 0.0 steer: 0.0 throttle: 0.15
```

the running path follower does not have a usable path.

Check that the path was actually recorded, saved, and loaded. A file
existing on disk does not necessarily mean it contains a complete lap.

### `donkey_path.csv` exists but contains almost nothing

Check:

``` bash
wc -l ~/gpscar/donkey_path.csv
```

One failed recording produced a 65-byte file containing only one point:

``` text
0.011470377037767321, -0.4490042310208082, 3.051850947599719e-05
```

That was not a usable lap. The successful rerecording contained 352
lines.

### Car immediately diverges from the path

Do not immediately retune the PID. First verify:

-   Correct path is loaded
-   GPS origin was reset with B at the correct physical start
-   Vehicle is positioned and oriented correctly
-   Path contains a complete lap
-   Manual steering direction is correct
-   VESC steering scale and offset are correct

During development, several PID changes failed to solve a bad run.
Rerecording a clean GPS path with the correct origin procedure restored
successful path following.

### Huge CTE immediately after autonomous mode begins

This usually indicates a coordinate-frame/origin mismatch.

Return to `user`, place the vehicle at the recorded starting point, and
reset the local GPS origin with **B** before autonomous operation.

### F710 suddenly stops behaving correctly

Check the physical **X/D** switch. Team 7 uses **X mode**.

The controller normally appears at:

``` text
/dev/input/js0
```

If changing modes does not take effect, stop DonkeyCar before
reconnecting the receiver or power-cycling the controller.

### START button does not change mode

If buttons such as B still work, DonkeyCar is receiving some controller
input. Check the F710 mode and button mapping before changing GPS or PID
settings.

### OAK-D / DepthAI API errors

The DonkeyCar environment uses a DepthAI 2.x API. A standalone script
written against another DepthAI API can fail even when the integrated
detector works.

Avoid modifying a known-working integrated detector solely to make an
older standalone test compatible.

## Known Working GPS Sequence

``` text
Start Point One RTK runner
        |
        v
Start manage.py
        |
        v
USER MODE
        |
        v
B: reset origin
        |
        v
LB: start recording
        |
        v
Drive smooth manual lap
        |
        v
LB: stop recording
        |
        v
RB: save path
        |
        v
Return to exact start
        |
        v
B: reset origin again
        |
        v
START: local_angle
        |
        v
Verify steering
        |
        v
START: local
        |
        v
Autonomous GPS path following
```

## Repository Structure

``` text
gpscar/
├── manage.py
├── myconfig.py
├── flag_override.py
└── donkey_path.csv
```

## Security

Do **not** commit:

-   Point One device credentials
-   Polaris passwords
-   API keys
-   Private tokens
-   Other team or infrastructure secrets

Example `.gitignore` additions:

``` gitignore
.env
*.log
__pycache__/
*.pyc
```

## Current Status

-   [x] Point One RTK GPS receiving corrections
-   [x] GPS position available inside DonkeyCar
-   [x] F710 manual vehicle control
-   [x] GPS origin reset
-   [x] GPS path recording and saving
-   [x] Autonomous GPS path following
-   [x] OAK-D yellow flag detection
-   [x] Triangle filtering
-   [x] Flag approach/capture state machine
-   [x] Jabra audio announcements
-   [x] GPS/flag control selector integration
-   [ ] Final end-to-end GPS + flag mission validation

## Team

**UC San Diego MAE/ECE 148 - Team 7**
