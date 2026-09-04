#!/usr/bin/env python3

"""
Usage:
    manage.py (drive) [--js] [--log=INFO] [--camera=(single|stereo)]

Options:
    -h --help                     Show this screen.
    --js                          Use physical joystick.
    --log=INFO                    Logging level.
    --camera=(single|stereo)      Camera type.
"""

from distutils.log import debug

import os

import logging
import subprocess
import tty
import termios
import sys
import threading

import time

try:

    import cv2

except:

    pass

from docopt import docopt

import donkeycar as dk

from donkeycar.parts.controller import JoystickController

from donkeycar.parts.path import (
    CsvThrottlePath,
    PathPlot,
    CTE,
    PID_Pilot,
    PlotCircle,
    PImage,
    OriginOffset,
)

from donkeycar.parts.transform import PIDController

from donkeycar.parts.kinematics import TwoWheelSteeringThrottle

from donkeycar.templates.complete import (
    add_odometry,
    add_camera,
    add_user_controller,
    add_drivetrain,
    add_simulator,
    add_imu,
    DriveMode,
    UserPilotCondition,
    ToggleRecording,
)

from donkeycar.parts.logger import LoggerPart

from donkeycar.parts.transform import Lambda

from donkeycar.parts.explode import ExplodeDict

from flag_override_new import FlagMissionPart, FlagDriveSelector

logger = logging.getLogger(__name__)

logging.basicConfig(level=logging.INFO)

def drive(cfg, use_joystick=False, camera_type='single'):

    '''

    Construct a working robotic vehicle from many parts.

    Each part runs as a job in the Vehicle loop, calling either

    it's run or run_threaded method depending on the constructor flag

    `threaded`.  All parts are updated one after another at the framerate given

    in cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely

    manner. Parts may have named outputs and inputs. The framework handles

    passing named outputs to parts requesting the same named input.

    '''

    is_differential_drive = cfg.DRIVE_TRAIN_TYPE.startswith("DC_TWO_WHEEL")

    V = dk.vehicle.Vehicle()

    if cfg.HAVE_SOMBRERO:

        from donkeycar.utils import Sombrero

        s = Sombrero()

    if cfg.HAVE_CONSOLE_LOGGING:

        logger.setLevel(logging.getLevelName(cfg.LOGGING_LEVEL))

        ch = logging.StreamHandler()

        ch.setFormatter(logging.Formatter(cfg.LOGGING_FORMAT))

        logger.addHandler(ch)

    if cfg.HAVE_MQTT_TELEMETRY:

        from donkeycar.parts.telemetry import MqttTelemetry

        tel = MqttTelemetry(cfg)

    add_simulator(V, cfg)

    add_imu(V, cfg)

    if cfg.HAVE_ODOM:

        add_odometry(V, cfg)

    else:

        cfg.WHEEL_ODOM_CALIB = None

        class NoOdom():

            def run(self):

                return 0.0

        V.add(NoOdom(), outputs=['enc/vel_m_s'])

    if cfg.HAVE_T265:

        from donkeycar.parts.realsense2 import RS_T265

        if cfg.HAVE_ODOM and not os.path.exists(cfg.WHEEL_ODOM_CALIB):

            print("You must supply a json file when using odom with T265. "

                  "There is a sample file in templates.")

            print("cp donkeycar/donkeycar/templates/calibration_odometry.json .")

            exit(1)

        rs = RS_T265(image_output=False, calib_filename=cfg.WHEEL_ODOM_CALIB, device_id=cfg.REALSENSE_T265_ID)

        V.add(rs, inputs=['enc/vel_m_s'], outputs=['rs/pos', 'rs/vel', 'rs/acc'], threaded=True)

        class PosStream:

            def run(self, pos):

                return -pos.z, -pos.x

        V.add(PosStream(), inputs=['rs/pos'], outputs=['pos/x', 'pos/y'])

    gps_player = add_gps(V, cfg)

    add_camera(V, cfg, camera_type)

    has_input_controller = hasattr(cfg, "CONTROLLER_TYPE") and cfg.CONTROLLER_TYPE != "mock"

    ctr = add_user_controller(V, cfg, use_joystick, input_image = 'map/image')

    V.add(ExplodeDict(V.mem, "web/"), inputs=['web/buttons'])

    origin_reset = OriginOffset(cfg.PATH_DEBUG)

    V.add(origin_reset, inputs=['pos/x', 'pos/y', 'cte/closest_pt'], outputs=['pos/x', 'pos/y', 'cte/closest_pt'])

    V.add(UserPilotCondition(),

          inputs=['user/mode', "cam/image_array", "cam/image_array"],

          outputs=['run_user', "run_pilot", "ui/image_array"])

    path = CsvThrottlePath(min_dist=cfg.PATH_MIN_DIST)

    V.add(path, inputs=['recording', 'pos/x', 'pos/y', 'user/throttle'], outputs=['path', 'throttles'])

    def save_path():

        if path.length() > 0:

            if path.save(cfg.PATH_FILENAME):

                print("That path was saved to ", cfg.PATH_FILENAME)

                if gps_player:

                    gps_player.nmea.save()

            else:

                print("The path could NOT be saved; check the PATH_FILENAME in myconfig.py to make sure it is a legal path")

        else:

            print("There is no path to save; try recording the path.")

    def load_path():

        if os.path.exists(cfg.PATH_FILENAME) and path.load(cfg.PATH_FILENAME):

           print("The path was loaded was loaded from ", cfg.PATH_FILENAME)

           if gps_player:

               gps_player.stop().nmea.load()

               gps_player.start()

        else:

           print("path _not_ loaded; make sure you have saved a path.")

    def erase_path():

        origin_reset.reset_origin()

        if path.reset():

            print("The origin and the path were reset; you are ready to record a new path.")

            if gps_player:

                gps_player.stop().nmea.reset()

        else:

            print("The origin was reset; you are ready to record a new path.")

    def reset_origin():

        """

        Reset effective pose to (0, 0)

        """

        origin_reset.reset_origin()

        print("The origin was reset to the current position.")

        if gps_player:

            gps_player.start()

    if os.path.exists(cfg.PATH_FILENAME):

        load_path()

    img = PImage(clear_each_frame=True)

    V.add(img, outputs=['map/image'])

    plot = PathPlot(scale=cfg.PATH_SCALE, offset=cfg.PATH_OFFSET)

    V.add(plot, inputs=['map/image', 'path'], outputs=['map/image'])

    cte = CTE(look_ahead=cfg.PATH_LOOK_AHEAD, look_behind=cfg.PATH_LOOK_BEHIND, num_pts=cfg.PATH_SEARCH_LENGTH)

    V.add(cte, inputs=['path', 'pos/x', 'pos/y', 'cte/closest_pt'], outputs=['cte/error', 'cte/closest_pt'], run_condition='run_pilot')

    pid = PIDController(p=cfg.PID_P, i=cfg.PID_I, d=cfg.PID_D)

    pilot = PID_Pilot(pid, cfg.PID_THROTTLE, cfg.USE_CONSTANT_THROTTLE, min_throttle=cfg.PID_THROTTLE)

    V.add(pilot, inputs=['cte/error', 'throttles', 'cte/closest_pt'], outputs=['pilot/steering', 'pilot/throttle'], run_condition="run_pilot")

    def dec_pid_d():

        pid.Kd -= cfg.PID_D_DELTA

        logging.info("pid: d- %f" % pid.Kd)

    def inc_pid_d():

        pid.Kd += cfg.PID_D_DELTA

        logging.info("pid: d+ %f" % pid.Kd)

    def dec_pid_p():

        pid.Kp -= cfg.PID_P_DELTA

        logging.info("pid: p- %f" % pid.Kp)

    def inc_pid_p():

        pid.Kp += cfg.PID_P_DELTA

        logging.info("pid: p+ %f" % pid.Kp)

    recording_control = ToggleRecording(cfg.AUTO_RECORD_ON_THROTTLE, cfg.RECORD_DURING_AI)

    V.add(recording_control, inputs=['user/mode', "recording"], outputs=["recording"])

    have_joystick = ctr is not None and isinstance(ctr, JoystickController)

    if cfg.SAVE_PATH_BTN:

        print(f"Save path button is {cfg.SAVE_PATH_BTN}")

        if cfg.SAVE_PATH_BTN.startswith("web/w"):

            V.add(Lambda(lambda: save_path()), run_condition=cfg.SAVE_PATH_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.SAVE_PATH_BTN, save_path)

    if cfg.LOAD_PATH_BTN:

        print(f"Load path button is {cfg.LOAD_PATH_BTN}")

        if cfg.LOAD_PATH_BTN.startswith("web/w"):

            V.add(Lambda(lambda: load_path()), run_condition=cfg.LOAD_PATH_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.LOAD_PATH_BTN, load_path)

    if cfg.ERASE_PATH_BTN:

        print(f"Erase path button is {cfg.ERASE_PATH_BTN}")

        if cfg.ERASE_PATH_BTN.startswith("web/w"):

            V.add(Lambda(lambda: erase_path()), run_condition=cfg.ERASE_PATH_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.ERASE_PATH_BTN, erase_path)

    if cfg.RESET_ORIGIN_BTN:

        print(f"Reset origin button is {cfg.RESET_ORIGIN_BTN}")

        if cfg.RESET_ORIGIN_BTN.startswith("web/w"):

            V.add(Lambda(lambda: reset_origin()), run_condition=cfg.RESET_ORIGIN_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.RESET_ORIGIN_BTN, reset_origin)

    if cfg.TOGGLE_RECORDING_BTN:

        print(f"Toggle recording button is {cfg.TOGGLE_RECORDING_BTN}")

        if cfg.TOGGLE_RECORDING_BTN.startswith("web/w"):

            V.add(Lambda(lambda: recording_control.toggle_recording()), run_condition=cfg.TOGGLE_RECORDING_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.TOGGLE_RECORDING_BTN, recording_control.toggle_recording)

    if cfg.DEC_PID_P_BTN and cfg.PID_P_DELTA:

        print(f"Decrement PID P button is {cfg.DEC_PID_P_BTN}")

        if cfg.DEC_PID_P_BTN.startswith("web/w"):

            V.add(Lambda(lambda: dec_pid_p()), run_condition=cfg.DEC_PID_P_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.DEC_PID_P_BTN, dec_pid_p)

    if cfg.INC_PID_P_BTN and cfg.PID_P_DELTA:

        print(f"Increment PID P button is {cfg.INC_PID_P_BTN}")

        if cfg.INC_PID_P_BTN.startswith("web/w"):

            V.add(Lambda(lambda: inc_pid_p()), run_condition=cfg.INC_PID_P_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.INC_PID_P_BTN, inc_pid_p)

    if cfg.DEC_PID_D_BTN and cfg.PID_D_DELTA:

        print(f"Decrement PID D button is {cfg.DEC_PID_D_BTN}")

        if cfg.DEC_PID_D_BTN.startswith("web/w"):

            V.add(Lambda(lambda: dec_pid_d()), run_condition=cfg.DEC_PID_D_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.DEC_PID_D_BTN, dec_pid_d)

    if cfg.INC_PID_D_BTN and cfg.PID_D_DELTA:

        print(f"Increment PID D button is {cfg.INC_PID_D_BTN}")

        if cfg.INC_PID_D_BTN.startswith("web/w"):

            V.add(Lambda(lambda: inc_pid_d()), run_condition=cfg.INC_PID_D_BTN)

        elif have_joystick:

            ctr.set_button_down_trigger(cfg.INC_PID_D_BTN, inc_pid_d)

    V.add(DriveMode(cfg.AI_THROTTLE_MULT),

          inputs=['user/mode', 'user/steering', 'user/throttle',

                  'pilot/steering', 'pilot/throttle'],

          outputs=['base/steering', 'base/throttle'])

    flag_mission = FlagMissionPart()

    V.add(

        flag_mission,

        inputs=['pos/x', 'pos/y'],

        outputs=['flag/active', 'flag/steering', 'flag/throttle'],

        threaded=True

    )

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

    if is_differential_drive:

        V.add(TwoWheelSteeringThrottle(),

            inputs=['throttle', 'steering'],

            outputs=['left/throttle', 'right/throttle'])

    add_drivetrain(V, cfg)

    if cfg.USE_SSD1306_128_32:

        from donkeycar.parts.oled import OLEDPart

        auto_record_on_throttle = cfg.USE_JOYSTICK_AS_DEFAULT and cfg.AUTO_RECORD_ON_THROTTLE

        oled_part = OLEDPart(cfg.SSD1306_128_32_I2C_ROTATION, cfg.SSD1306_RESOLUTION, auto_record_on_throttle)

        V.add(oled_part, inputs=['recording', 'tub/num_records', 'user/mode'], outputs=[], threaded=True)

    if ctr is not None and isinstance(ctr, JoystickController):

        ctr.print_controls()

    loc_plot = PlotCircle(scale=cfg.PATH_SCALE, offset=cfg.PATH_OFFSET, color = "blue")

    V.add(loc_plot, inputs=['map/image', 'pos/x', 'pos/y'], outputs=['map/image'], run_condition='run_pilot')

    loc_plot = PlotCircle(scale=cfg.PATH_SCALE, offset=cfg.PATH_OFFSET, color = "green")

    V.add(loc_plot, inputs=['map/image', 'pos/x', 'pos/y'], outputs=['map/image'], run_condition='run_user')


    def keyboard_speaker():
        messages = {
            "1": "pink flag spotted",
            "2": "pink flag captured",
            "3": "blue flag spotted",
            "4": "blue flag captured",
            "5": "yellow flag spotted",
            "6": "yellow flag captured",
            "7": "orange flag spotted",
            "8": "orange flag captured",
        }

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while True:
                key = sys.stdin.read(1)

                if key in messages:
                    message = messages[key]
                    print("SPEAKER:", message)

                    espeak_process = subprocess.Popen(
                        ["espeak", "--stdout", message],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL
                    )

                    subprocess.Popen(
                        ["aplay", "-D", "plughw:2,0"],
                        stdin=espeak_process.stdout,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )

                    espeak_process.stdout.close()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    threading.Thread(target=keyboard_speaker, daemon=True).start()

    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,

        max_loop_count=cfg.MAX_LOOPS)

def add_gps(V, cfg):

    if cfg.HAVE_GPS:

        from donkeycar.parts.serial_port import SerialPort, SerialLineReader

        from donkeycar.parts.gps import GpsNmeaPositions, GpsLatestPosition, GpsPlayer

        from donkeycar.parts.pipe import Pipe

        from donkeycar.parts.text_writer import CsvLogger

        serial_port = SerialPort(cfg.GPS_SERIAL, cfg.GPS_SERIAL_BAUDRATE)

        nmea_reader = SerialLineReader(serial_port)

        V.add(nmea_reader, outputs=['gps/nmea'], threaded=True)

        nmea_player = None

        if cfg.GPS_NMEA_PATH:

            nmea_writer = CsvLogger(cfg.GPS_NMEA_PATH, separator='\t', field_count=2)

            V.add(nmea_writer, inputs=['recording', 'gps/nmea'], outputs=['gps/recorded/nmea'])

            nmea_player = GpsPlayer(nmea_writer)

            V.add(nmea_player, inputs=['run_pilot', 'gps/nmea'], outputs=['gps/playing', 'gps/nmea'])

        gps_positions = GpsNmeaPositions(debug=cfg.GPS_DEBUG)

        V.add(gps_positions, inputs=['gps/nmea'], outputs=['gps/positions'])

        gps_latest_position = GpsLatestPosition(debug=cfg.GPS_DEBUG)

        V.add(gps_latest_position, inputs=['gps/positions'], outputs=['gps/timestamp', 'gps/utm/longitude', 'gps/utm/latitude'])

        V.add(Pipe(), inputs=['gps/utm/longitude', 'gps/utm/latitude'], outputs=['pos/x', 'pos/y'])

        return nmea_player

if __name__ == '__main__':

    args = docopt(__doc__)

    cfg = dk.load_config()

    log_level = args['--log'] or "INFO"

    numeric_level = getattr(logging, log_level.upper(), None)

    if not isinstance(numeric_level, int):

        raise ValueError('Invalid log level: %s' % log_level)

    logging.basicConfig(level=numeric_level)

    if args['drive']:

        drive(cfg, use_joystick=args['--js'], camera_type=args['--camera'])

        
