import os, logging, time
import numpy as np
from . import adxl345


DATA_FOLDER = '/tmp'
DELAY = 1.00                # Delay between checks csv in tmp in sec
OPEN_DELAY = 0.25           # Delay between open csv in sec
TIMER = 20.00               # Exit program time in sec
MEDIAN_FILTER_WINDOW = 3
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
        self.microsteps = self.config.getint('microsteps', default=16, minval=2, maxval=16)
        self.steps_threshold = self.config.getint('steps_threshold', default=100000, minval=5000, maxval=100000)
        self.fast_threshold = self.config.getint('fast_threshold', default=0, minval=0, maxval=100000)
        self.force_threshold = self.config.getint('force_threshold', default=0, minval=0, maxval=100000)
        self.max_retries = self.config.getint('threshold_retries', default=0, minval=0, maxval=10)
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
            raise gcmd.error("Invalid format of axis '%s'" % (raw_axis,))
        return split

    def _find_z_axis(self, file_path):
        global z_axis
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip} NAME=stand_still')
        time.sleep(0.25)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip} NAME=stand_still')
        self._wait_csv()
        for f in os.listdir(file_path):
            if f.endswith('stand_still.csv'):
                with open(os.path.join(file_path, f), 'r') as file:
                    data = np.genfromtxt(file, delimiter=',', skip_header=1)
                    z_axis = np.abs(data[1:, 1:]).mean(axis=0).argmax() + 1
                    # self.gcode.respond_info(f'Z on "{z_axis}" colum')
                    os.remove(os.path.join(file_path, f))

    def _wait_csv(self):
        timer = 0
        while True:
            time.sleep(DELAY)
            timer += 1
            for f in os.listdir(DATA_FOLDER):
                if f.endswith('.csv'):
                    time.sleep(OPEN_DELAY)
                    return
                elif timer > TIMER / DELAY:
                    print('No CSV files found in the directory, aborting')
                    raise self.gcode.error(f'No CSV files found in the directory, aborting')
                else: continue

    def _buzz(self, stepper):
        lookup_sec_stepper = self.force_move._lookup_stepper({'STEPPER': stepper + '1'})
        self._stepper_switch(stepper, 0)
        for i in range(0, int(self.move_len * 3200)):
            dist = (1 - self.move_len * 2 * i) * 2
            self._stepper_move(lookup_sec_stepper, dist)
            self._stepper_move(lookup_sec_stepper, -dist)
        self._stepper_switch(stepper, 1)

    def _calc_magnitude(self):
        try:
            self._wait_csv()
            # Get the list of all CSV files in the directory
            csv_files = [f for f in os.listdir(DATA_FOLDER) if f.endswith('.csv')]
            if len(csv_files) > 1:
                self.gcode.error("More than one CSV file found! Aborting")
            file_name = csv_files[0]
            file_path = os.path.join(DATA_FOLDER, file_name)
            data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
            os.remove(file_path)
            accel_x_filtered = np.convolve(data[1:, 1], np.ones(MEDIAN_FILTER_WINDOW) / MEDIAN_FILTER_WINDOW, mode='same')
            accel_y_filtered = np.convolve(data[1:, 2], np.ones(MEDIAN_FILTER_WINDOW) / MEDIAN_FILTER_WINDOW, mode='same')
            accel_z_filtered = np.convolve(data[1:, 3], np.ones(MEDIAN_FILTER_WINDOW) / MEDIAN_FILTER_WINDOW, mode='same')
            magnitudes = np.linalg.norm(np.vstack((
                np.where(z_axis != 1, accel_x_filtered, np.zeros_like(0)),
                np.where(z_axis != 2, accel_y_filtered, np.zeros_like(0)),
                np.where(z_axis != 3, accel_z_filtered, np.zeros_like(0)))), axis=0)
            # Return average of 5 maximum magnitudes
            return round(np.mean(np.sort(magnitudes)[-5:]), 2)
        except Exception as e:
            self.gcode.error(f"Error processing generated CSV: {str(e)}")

    def _measure(self, axis, buzz=True):
        stepper = 'stepper_' + axis
        if buzz: self._buzz(stepper)
        self._stepper_switch(stepper, 0)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip}')
        time.sleep(0.25)
        self._stepper_switch(stepper, 1)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip}')
        return self._calc_magnitude()

    def prestart(self):
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
        self.force_threshold = gcmd.get_int('FORCE_THRESHOLD', self.force_threshold, minval=0, maxval=100000)
        self.max_retries = gcmd.get_int('THRESHOLD_RETRIES', self.max_retries, minval=0, maxval=10)
        self.prestart()
        self._find_z_axis(DATA_FOLDER)
        total_info = []
        for axis in self.axes:
            axis = axis.lower()
            self.stepper = 'stepper_' + axis
            self.lookup_stepper = self.force_move._lookup_stepper({'STEPPER': self.stepper})
            if self.respond: self.gcode.respond_info(f'{axis.upper()} Motors synchronization...')

            # First measurement
            self.magnitude = self._measure(axis, True)
            self.actual_microsteps = 0
            self.magnitude_before_sync = self.magnitude
            if self.respond: self.gcode.respond_info(f'Initial magnitude: {self.magnitude}')
            running = True
            retries = 0
            while running:
                self.moving_microsteps = max(int(self.magnitude / (self.steps_threshold)), 1)
                self._stepper_move(self.lookup_stepper, self.moving_microsteps * self.move_len)
                self.actual_microsteps += self.moving_microsteps
                self.new_magnitude = self._measure(axis, True)
                if self.respond: self.gcode.respond_info(
                    f'New magnitude: {self.new_magnitude} '
                    f'on {self.moving_microsteps}/{self.microsteps} step move')

                # Determine movement direction
                if self.new_magnitude > self.magnitude:
                    self.move_dir = [-1, 'Backward']
                else:
                    self.move_dir = [1, 'Forward']
                self.gcode.respond_info(f"Movement direction: {self.move_dir[1]}")
                self.magnitude = self.new_magnitude

                while True:
                    buzz = False if self.magnitude > self.fast_threshold and self.fast_threshold != 0 else True
                    self.moving_microsteps = max(int(self.magnitude / self.steps_threshold), 1)
                    self._stepper_move(self.lookup_stepper, self.moving_microsteps * self.move_dir[0] * self.move_len)
                    self.actual_microsteps += self.moving_microsteps
                    self.new_magnitude = self._measure(axis, buzz)
                    if self.respond: self.gcode.respond_info(
                        f'New magnitude: {self.new_magnitude} '
                        f'on {self.move_dir[0] * self.moving_microsteps}/{self.microsteps} step move')
                    if self.new_magnitude > self.magnitude:
                        self._stepper_move(self.lookup_stepper, self.moving_microsteps * self.move_len * self.move_dir[0] * -1)
                        self.actual_microsteps -= self.moving_microsteps
                        if self.force_threshold != 0 and self.new_magnitude > self.force_threshold:
                            if retries > self.max_retries:
                                raise self.gcode.error('Too many retries')
                            if self.respond: self.gcode.respond_info(
                                f'Retries: {retries}/{self.max_retries} Back on last magnitude: '
                                f'{self.magnitude} on {self.move_dir[0] * self.actual_microsteps}/'
                                f'{self.microsteps} step to reach {self.force_threshold}')
                            retries += 1
                            break
                        total_info.append(
                            f'{axis.upper()} Motors adjusted by {self.actual_microsteps}/{self.microsteps} step, '
                            f'magnitude {self.magnitude_before_sync} --> {self.magnitude}')
                        if self.respond: self.gcode.respond_info(total_info[-1])
                        running = False
                        break
                    self.magnitude = self.new_magnitude
        if self.respond:
            if len(total_info) > 1:
                self.gcode.respond_info(total_info[0])
        else:
            for text in total_info:
                self.gcode.respond_info(text)

def load_config(config):
    return MotorsSync(config)
