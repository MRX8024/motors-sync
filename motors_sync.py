import os, logging, time
import numpy as np
from . import adxl345


DATA_FOLDER = '/tmp'        # Folder where csv are generate
DELAY = 0.05                # Delay between checks csv in /tmp in sec
OPEN_DELAY = 0.1            # Delay between open csv in sec
TIMER = 5.00                # Exit program time in sec
MEDIAN_FILTER_WINDOW = 3    # Number of window lines
static_data = ''
z_axis = ''


class MotorsSync:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.force_move = self.printer.load_object(config, 'force_move')
        self.stepper_en = self.printer.load_object(config, 'stepper_enable')
        self.printer.register_event_handler("klippy:connect", self.handler)
        # Read config
        self.accel_chip = self.config.get('accel_chip', (self.config.getsection('resonance_tester').get('accel_chip')))
        self.microsteps = self.config.getint('microsteps', default=16, minval=2, maxval=32)
        self.steps_threshold = self.config.getint('steps_threshold', default=1000000, minval=5000, maxval=100000)
        self.fast_threshold = self.config.getint('fast_threshold', default=None, minval=0, maxval=100000)
        self.retry_tolerance = self.config.getint('retry_tolerance', default=None, minval=0, maxval=100000)
        self.max_retries = self.config.getint('retries', default=0, minval=0, maxval=10)
        self.respond = self.config.getboolean('respond', default=True)
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SYNC_MOTORS', self.cmd_RUN_SYNC, desc='Start 4WD synchronization')
        # Variables
        self.move_len = 40 / 200 / self.microsteps

    def handler(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2

    def _send(self, func):
        self.gcode._process_commands([func], False)

    def _stepper_switch(self, stepper, mode):
        self.stepper_en.motor_debug_enable(stepper, mode)

    def _stepper_move(self, stepper, dist):
        self.force_move.manual_move(stepper, dist, 100, 5000)

    def _parse_axis(self, gcmd):
        try:
            raw_axis = gcmd.get('AXIS')
            if raw_axis is None:
                return ['x', 'y']
            raw_axis = raw_axis.lower()
            if raw_axis == 'xy':
                return ['x', 'y']
            if raw_axis in ['x', 'y']:
                return raw_axis
            split = raw_axis.split(',')
            if len(split) != 2:
                raise gcmd.error(f'Invalid format of axis {raw_axis}')
            return split
        except: return ['x', 'y']

    def _static_measure(self):
        global z_axis, static_data
        # Measure vibrations
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip} NAME=stand_still')
        time.sleep(0.25)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip} NAME=stand_still')
        # Init CSV file
        file = self._wait_csv('stand_still.csv')
        vect = np.mean(np.genfromtxt(file, delimiter=',', skip_header=1, usecols=(1, 2, 3)), axis=0)
        os.remove(file)
        # Calculate static and find z axis for future exclude
        z_axis = np.abs(vect[0:]).argmax()
        xy_vect = np.delete(vect, z_axis, axis=0)
        static_data = round(np.linalg.norm(xy_vect, axis=0), 2)

    def _wait_csv(self, name):
        timer = 0
        while True:
            time.sleep(DELAY)
            timer += 1
            for f in os.listdir(DATA_FOLDER):
                if f.endswith(name):
                    time.sleep(OPEN_DELAY)
                    return os.path.join(DATA_FOLDER, f)
                elif timer > TIMER / DELAY:
                    raise self.gcode.error(f'No CSV files found in the directory, aborting')
                else: continue

    def _buzz(self, stepper):
        lookup_sec_stepper = self.force_move._lookup_stepper({'STEPPER': stepper + '1'})
        self._stepper_switch(stepper, 0)
        for i in range(0, int(self.microsteps * 2.5)):
            dist = (1 - self.move_len * 2 * i) * 2
            self._stepper_move(lookup_sec_stepper, dist)
            self._stepper_move(lookup_sec_stepper, -dist)
        self._stepper_switch(stepper, 1)

    def _calc_magnitude(self):
        try:
            # Init CSV file
            file = self._wait_csv('.csv')
            vect = np.genfromtxt(file, delimiter=',', skip_header=1, usecols=range(1,4))
            os.remove(file)
            xy_vect = np.delete(vect, z_axis, axis=1)
            # Add window mean filter
            magnitude = []
            for i in range(int(MEDIAN_FILTER_WINDOW / 2), len(xy_vect) - int(MEDIAN_FILTER_WINDOW / 2)):
                filtered_xy_vect = (np.median([xy_vect[i-1], xy_vect[i], xy_vect[i+1]], axis=0))
                magnitude.append(np.linalg.norm(filtered_xy_vect))
            # Return avg of 5 max magnitudes with deduction static
            magnitude = np.mean(np.sort(magnitude)[-5:])
            return round(magnitude - static_data, 2)
        except Exception as e:
            self.gcode.error(f"Error processing generated CSV: {str(e)}")

    def _measure(self, axis, buzz=True):
        stepper = 'stepper_' + axis
        if buzz: self._buzz(stepper)
        self._stepper_switch(stepper, 0)
        time.sleep(0.25)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip}')
        time.sleep(0.25)
        self._stepper_switch(stepper, 1)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip}')
        return self._calc_magnitude()

    def _prestart(self):
        os.system(f'rm -f {DATA_FOLDER}/*.csv')
        now = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(now)
        self.center_x = (int(self.config.getsection('stepper_x').get('position_max'))
                         - int(self.config.getsection('stepper_x').get('position_min'))) / 2
        self.center_y = (int(self.config.getsection('stepper_y').get('position_max'))
                         - int(self.config.getsection('stepper_y').get('position_min'))) / 2
        if 'xy' not in kin_status['homed_axes']:
            self._send('G28 X Y')
        self.toolhead.manual_move([self.center_x,self.center_y , None], self.travel_speed)
        self.toolhead.wait_moves()

    def cmd_RUN_SYNC(self, gcmd):
        self.axes = self._parse_axis(gcmd)
        self.accel_chip = gcmd.get('ACCEL_CHIP', self.accel_chip)
        self.steps_threshold = gcmd.get_int('STEPS_THRESHOLD', self.steps_threshold, minval=5000, maxval=100000)
        self.fast_threshold = gcmd.get_int('FAST_THRESHOLD', self.fast_threshold, minval=0, maxval=100000)
        self.retry_tolerance = gcmd.get_int('RETRY_TOLERANCE', self.retry_tolerance, minval=0, maxval=100000)
        self.max_retries = gcmd.get_int('RETRIES', self.max_retries, minval=0, maxval=10)
        self._prestart()
        self._static_measure()
        total_info = []
        for axis in self.axes:
            axis = axis.lower()
            self.stepper = 'stepper_' + axis
            self.lookup_stepper = self.force_move._lookup_stepper({'STEPPER': self.stepper})
            if self.respond: self.gcode.respond_info(f'{axis.upper()} Motors synchronization...')

            # First measurement
            magnitude = self._measure(axis, True)
            actual_microsteps = 0
            magnitude_before_sync = magnitude
            if self.respond: self.gcode.respond_info(f'Initial magnitude: {magnitude}')
            running = True
            retries = 0
            while running:
                moving_microsteps = max(int(magnitude / self.steps_threshold), 1)
                self._stepper_move(self.lookup_stepper, moving_microsteps * self.move_len)
                actual_microsteps += moving_microsteps
                new_magnitude = self._measure(axis, True)
                if self.respond: self.gcode.respond_info(
                    f'New magnitude: {new_magnitude} '
                    f'on {moving_microsteps}/{self.microsteps} step move')

                # Determine movement direction
                if new_magnitude > magnitude:
                    move_dir = [-1, 'Backward']
                else:
                    move_dir = [1, 'Forward']
                self.gcode.respond_info(f'Movement direction: {move_dir[1]}')
                magnitude = new_magnitude

                while True:
                    buzz = False if self.fast_threshold and magnitude > self.fast_threshold else True
                    moving_microsteps = max(int(magnitude / self.steps_threshold), 1)
                    self._stepper_move(self.lookup_stepper, moving_microsteps * move_dir[0] * self.move_len)
                    actual_microsteps += moving_microsteps * move_dir[0]
                    new_magnitude = self._measure(axis, buzz)
                    if self.respond: self.gcode.respond_info(
                        f'New magnitude: {new_magnitude} '
                        f'on {move_dir[0] * moving_microsteps}/{self.microsteps} step move')
                    if new_magnitude > magnitude:
                        self._stepper_move(self.lookup_stepper, moving_microsteps * self.move_len * move_dir[0] * -1)
                        actual_microsteps += moving_microsteps * move_dir[0] * -1
                        if self.retry_tolerance and magnitude > self.retry_tolerance:
                            retries += 1
                            if retries > self.max_retries:
                                self.gcode.respond_info(
                                    f'{axis.upper()} Motors adjusted by {actual_microsteps}/{self.microsteps} step, '
                                    f'magnitude {magnitude_before_sync} --> {magnitude}')
                                raise self.gcode.error('Too many retries')
                            if self.respond: self.gcode.respond_info(
                                f'Retries: {retries}/{self.max_retries} Back on last magnitude: '
                                f'{magnitude} on {actual_microsteps}/'
                                f'{self.microsteps} step to reach {self.retry_tolerance}')
                            break
                        total_info.append(
                            f'{axis.upper()} Motors adjusted by {actual_microsteps}/{self.microsteps} step, '
                            f'magnitude {magnitude_before_sync} --> {magnitude}')
                        if self.respond: self.gcode.respond_info(total_info[-1])
                        running = False
                        break
                    magnitude = new_magnitude
        if self.respond:
            if len(total_info) > 1:
                self.gcode.respond_info(total_info[0])
        else:
            for text in total_info:
                self.gcode.respond_info(text)

def load_config(config):
    return MotorsSync(config)