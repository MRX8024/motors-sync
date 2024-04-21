import os, logging, time
import requests
import pandas as pd
import numpy as np
from . import resonance_tester, adxl345, force_move, stepper_enable


DATA_FOLDER = '/tmp'
# DATA_FOLDER = 'Z:/chopper-resonance-tuner/chopper-resonance-tuner/tmp/'
DELAY = 1.00                # Delay between checks csv in tmp in sec
OPEN_DELAY = 0.25           # Delay between open csv in sec
TIMER = 20.00               # Exit program time in sec
MAGNITUDE_THRESHOLD = 7500  # Adjust to speed up the process.
MEDIAN_FILTER_WINDOW = 3
z_axis = ''
class MotorsSync:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.steppers = {}
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        # Read config
        self.accel_chip = self.config.get('accel_chip', default=(self.config.getsection('resonance_tester').get('accel_chip')))
        self.microsteps = self.config.getint('microsteps', default=16, minval=2, maxval=16)
        self.move_len = 40 / 200 / self.microsteps
        # self.force_move = force_move.ForceMove(config)
        # self.stepper_en = stepper_enable.PrinterStepperEnable(config)
        # self.printer.register_event_handler("klippy:connect", self.toolhead)
        # self.toolhead = self.printer.lookup_object('toolhead')
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('MOTORS_SYNC', self.cmd_RUN_SYNC, desc='Start 4WD synchronization')
        self.session = requests.Session()

    def _send(self, func):
        self.gcode._process_commands([func], False)

    # def _move(self, coord, speed):
    #     self.toolhead.manual_move(coord, speed)

    def _move(self, axis, times):
        self.gcode.respond_info(f'Move {axis.upper()}1 motor {times}/{self.microsteps} step')
        # self.force_move.manual_move(f'stepper_{axis}1', self.move_len * times, 100, 100)
        self._send(f'FORCE_MOVE STEPPER=stepper_{axis}1 DISTANCE={self.move_len * times} VELOCITY=100 ACCEL=100')
        return

    def _find_z_axis(self, file_path):
        global z_axis
        # adxl345.AccelCommandHelper.cmd_ACCELEROMETER_MEASURE(self, f'ACCELEROMETER_MEASURE CHIP=mpu9250 NAME=stand_still')
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip} NAME=stand_still')
        time.sleep(0.25)
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip} NAME=stand_still')
        self._wait_csv()
        for f in os.listdir(file_path):
            if f.endswith('stand_still.csv'):
                with open(os.path.join(file_path, f), 'r') as file:
                    data = pd.read_csv(file, delimiter=',')
                    z_axis = data.iloc[1:].mean().abs().idxmax()
                    self.gcode.respond_info(f'Z on "{z_axis}" colum')
                    os.remove(os.path.join(file_path, f))
        return

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

    def _buzz(self, axis):
        # self.stepper_en.cmd_SET_STEPPER_ENABLE(f'STEPPER=stepper_{axis} ENABLE=0')
        self._send(f'SET_STEPPER_ENABLE STEPPER=stepper_{axis} ENABLE=0')
        # for i in range(0, int(self.move_len * 6400)):
        #     self.distance = (1 - self.move_len * i) * 2
        #     self.force_move.manual_move(self, f'stepper_{axis}1', self.distance, 100, 100)
        #     self.force_move.manual_move(self, f'stepper_{axis}1', -self.distance, 100, 100)
        # self.stepper_en.cmd_SET_STEPPER_ENABLE(f'STEPPER=stepper_{axis} ENABLE=1')
        self._send(f'SET_STEPPER_ENABLE STEPPER=stepper_{axis} ENABLE=1')
        return

    def _calc_magnitude(self):
        try:
            self._wait_csv()
            # Get the list of all CSV files in the directory
            csv_files = [f for f in os.listdir(DATA_FOLDER) if f.endswith('.csv')]
            if len(csv_files) > 1:
                self.gcode.error("More than one CSV file found! Aborting")
            file_name = csv_files[0]
            file_path = os.path.join(DATA_FOLDER, file_name)
            data = pd.read_csv(file_path)
            # Apply median filter to accelerometer data, calculate magnitude for each row using filtered data
            data['magnitude'] = np.linalg.norm(
                                np.vstack((
                                    np.convolve(data['accel_x'], np.ones(MEDIAN_FILTER_WINDOW) / MEDIAN_FILTER_WINDOW, mode='same')
                                    if z_axis != 'accel_x' else np.zeros_like(data['accel_x']),
                                    np.convolve(data['accel_y'], np.ones(MEDIAN_FILTER_WINDOW) / MEDIAN_FILTER_WINDOW, mode='same')
                                    if z_axis != 'accel_y' else np.zeros_like(data['accel_y']),
                                    np.convolve(data['accel_z'], np.ones(MEDIAN_FILTER_WINDOW) / MEDIAN_FILTER_WINDOW, mode='same')
                                    if z_axis != 'accel_z' else np.zeros_like(data['accel_z']))), axis=0)

            # Find the 5 maximum magnitudes and calculate their average
            average_max_magnitude = data.nlargest(5, 'magnitude')['magnitude'].mean()
            os.remove(file_path)
            return average_max_magnitude
        except Exception as e:
            print(f"Error processing generated CSV: {str(e)}")
            return None

    def _measure(self, axis):
        self._buzz(axis)
        self._send(f'SET_STEPPER_ENABLE STEPPER=stepper_{axis} ENABLE=0')
        # self.stepper_en.cmd_SET_STEPPER_ENABLE(f'STEPPER=stepper_{axis} ENABLE=0')
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip}')
        time.sleep(0.25)
        # self.stepper_en.cmd_SET_STEPPER_ENABLE(f'STEPPER=stepper_{axis} ENABLE=1')
        self._send(f'SET_STEPPER_ENABLE STEPPER=stepper_{axis} ENABLE=1')
        self._send(f'ACCELEROMETER_MEASURE CHIP={self.accel_chip}')
        return self._calc_magnitude()

    def prestart(self):
        os.system(f'rm -f {DATA_FOLDER}/*.csv')
        self.gcode.respond_info('Homing...')
        # self.center_x = (self.steppers['x'][0] + self.steppers['x'][1]) / 2
        # self.center_y = (self.steppers['y'][0] + self.steppers['y'][1]) / 2
        self.center_x = (int(self.config.getsection('stepper_x').get('position_max'))
                         - int(self.config.getsection('stepper_x').get('position_min'))) / 2
        self.center_y = (int(self.config.getsection('stepper_y').get('position_max'))
                         - int(self.config.getsection('stepper_y').get('position_min'))) / 2
        self.gcode.respond_info(f'X{self.center_x} Y{self.center_y}')

        self._send('G28 X Y')
        self.printer.lookup_object('toolhead').wait_moves()
        self._send(f'G0 X{self.center_x} Y{self.center_y} F12000')
        self.printer.lookup_object('toolhead').wait_moves()
        # self.printer.lookup_object('toolhead').manual_move(f'X{self.center_x} Y{self.center_y}', 200)



    def cmd_RUN_SYNC(self, gcmd):
        self.prestart()
        self._find_z_axis(DATA_FOLDER)
        # self.axes = gcmd.get('AXES')
        # self.gcode.respond_info(self.axes)
        # if self.axes[0].lower() is None or self.axes[0].lower() == 'xy': self.axes = 'x y'
        self.axes = 'X Y'
        for self.axis in self.axes.split(' '):
            self.axis = self.axis.lower()
            self.gcode.respond_info(f'{self.axis.upper()} Motors synchronization')

            # First measurement
            self.magnitude = self._measure(self.axis)
            self.actual_microsteps = 0
            self.magnitude_before_sync = self.magnitude
            self.gcode.respond_info(f'Initial Magnitude: {self.magnitude}')
            self.moving_microsteps = max(int(self.magnitude / (MAGNITUDE_THRESHOLD * 2)), 1)
            self._move(self.axis, self.moving_microsteps)
            self.actual_microsteps += self.moving_microsteps
            self.new_magnitude = self._measure(self.axis)
            self.gcode.respond_info(f'New Magnitude: {self.new_magnitude}')

            # Determine movement direction
            if self.new_magnitude > self.magnitude:
                self.move_dir = [-1, 'forward']
            else:
                self.move_dir = [1, 'backward']
            self.gcode.respond_info(f"Movement Direction: {self.move_dir[1]}")
            self.magnitude = self.new_magnitude

            while True:
                self.moving_microsteps = max(int(self.magnitude / MAGNITUDE_THRESHOLD), 1)
                self._move(self.axis, self.moving_microsteps * self.move_dir[0])
                self.actual_microsteps += self.moving_microsteps
                self.new_magnitude = self._measure(self.axis)
                self.gcode.respond_info(f'New Magnitude: {self.new_magnitude}')
                if self.new_magnitude > self.magnitude:
                    self._move(self.axis, self.moving_microsteps * self.move_dir[0] * -1)
                    self.actual_microsteps -= self.moving_microsteps
                    self.gcode.respond_info(f'{self.axis.upper()} Motors synchronization completed.\n'
                                 f'Adjusted by {self.actual_microsteps}/{self.microsteps} step.\n'
                                 f'Initial magnitude = {self.magnitude_before_sync}, final magnitude = {self.magnitude}')
                    break
                self.magnitude = self.new_magnitude

# if __name__ == "__main__":
#     main()

def load_config(config):
    return MotorsSync(config)
