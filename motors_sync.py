# Motors synchronization script
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, time, itertools
import numpy as np
from . import z_tilt

MEASURE_DELAY = 0.25        # Delay between damped oscillations and measurement
AXES_LEVEL_DELTA = 2000     # Magnitude difference between axes
MEDIAN_FILTER_WINDOW = 3    # Number of window lines

class MotorsSync:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.force_move = self.printer.load_object(config, 'force_move')
        self.stepper_en = self.printer.load_object(config, 'stepper_enable')
        self.printer.register_event_handler("klippy:connect", self.handler)
        self.status = z_tilt.ZAdjustStatus(self.printer)
        # Read config
        self.accel_chip = self._get_chip()
        self.microsteps = self.config.getint('microsteps', default=16, minval=2, maxval=32)
        self.steps_coeff = self.config.getint('steps_coeff', default=999999, minval=5000, maxval=999999)
        self.fast_threshold = self.config.getint('fast_threshold', default=999999, minval=0, maxval=999999)
        self.retry_tolerance = self.config.getint('retry_tolerance', default=999999, minval=0, maxval=999999)
        self.max_retries = self.config.getint('retries', default=0, minval=0, maxval=10)
        self.respond = self.config.getboolean('respond', default=True)
        # Register commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_RUN_SYNC, desc='Start 4WD synchronization')
        # Variables
        rotation_distance = int(round(self.config.getsection('stepper_x').getfloat('rotation_distance')))
        steps_per_rotation = int(self.config.getsection('stepper_x').getint('full_steps_per_rotation', 200))
        self.move_len = rotation_distance / steps_per_rotation / self.microsteps

    def handler(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2

    def lookup_config(self, section, section_entries, force_back='empty'):
        out = []
        section = self.config.getsection(section)
        for value in section_entries:
            try:
                out.append(float(section.get(value, force_back)))
            except:
                out.append(section.get(value, force_back))
        if 'empty' in out: raise
        return out[0] if len(out) == 1 else out

    def _get_chip(self):
        try:
            return self.config.get('accel_chip')
        except:
            return self.lookup_config('resonance_tester', ['accel_chip'])

    def _send(self, params):
        self.gcode._process_commands([params], False)

    def _stepper_switch(self, stepper, mode):
        self.stepper_en.motor_debug_enable(stepper, mode)

    def _stepper_move(self, stepper, dist):
        self.force_move.manual_move(stepper, dist, 100, 5000)

    def _static_measure(self):
        # Measure static vibrations
        aclient = self.chip_config.start_internal_client()
        time.sleep(0.250)
        aclient.finish_measurements()
        # Init data
        vect = np.mean(np.array([
            [sample.accel_x,sample.accel_y,sample.accel_z]
            for sample in aclient.get_samples()]), axis=0)
        # Calculate static and find z axis for future exclude
        self.z_axis = np.abs(vect[0:]).argmax()
        xy_vect = np.delete(vect, self.z_axis, axis=0)
        self.static_data = round(np.linalg.norm(xy_vect, axis=0), 2)

    def _buzz(self, stepper):
        # Fading oscillations
        lookup_sec_stepper = self.force_move._lookup_stepper({'STEPPER': stepper + '1'})
        self._stepper_switch(stepper, 0)
        for i in reversed(range(0, int(0.4 / self.move_len))):
            dist = round((self.move_len * 4 * i), 4)
            self._stepper_move(lookup_sec_stepper, dist)
            self._stepper_move(lookup_sec_stepper, -dist)
        self._stepper_switch(stepper, 1)

    def _calc_magnitude(self, aclient):
        # Calculate impact magnitude
        vect = np.array([[sample.accel_x, sample.accel_y, sample.accel_z]
                         for sample in aclient.get_samples()])
        xy_vect = np.delete(vect, self.z_axis, axis=1)
        # Add window mean filter
        magnitude = []
        for i in range(int(MEDIAN_FILTER_WINDOW / 2), len(xy_vect) - int(MEDIAN_FILTER_WINDOW / 2)):
            filtered_xy_vect = (np.median([xy_vect[i-int(MEDIAN_FILTER_WINDOW / 2)],
                                xy_vect[i], xy_vect[i+int(MEDIAN_FILTER_WINDOW / 2)]], axis=0))
            magnitude.append(np.linalg.norm(filtered_xy_vect))
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitude)[-5:])
        return round(magnitude - self.static_data, 2)

    def _measure(self, stepper, buzz=True):
        # Measure the impact
        if buzz: self._buzz(stepper)
        self._stepper_switch(stepper, 0)
        time.sleep(MEASURE_DELAY)
        aclient = self.chip_config.start_internal_client()
        self._stepper_switch(stepper, 1)
        aclient.finish_measurements()
        return self._calc_magnitude(aclient)

    def _prestart(self):
        # Homing and going to center
        now = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(now)
        center = []
        for axis in self.axes:
            stepper = 'stepper_' + axis.lower()
            min, max = self.lookup_config(stepper, ['position_min', 'position_max'], 0)
            center.append(min + ((max - min) / 2))
        if ''.join(self.axes).lower() not in kin_status['homed_axes']:
            self._send(f"G28 {' '.join(self.axes).upper()}")
        self.toolhead.manual_move([center[0], center[1], None], self.travel_speed)
        self.toolhead.wait_moves()

    def _axes_level(self):
        # Axes leveling by magnitude
        main_axis = self.motion['max_axis']
        sec_axis = self.motion['min_axis']
        init_max_magnitude = self.motion[main_axis]['new_magnitude']
        init_min_magnitude = self.motion[sec_axis]['init_magnitude']
        delta = round(init_max_magnitude - init_min_magnitude, 2)
        if delta > AXES_LEVEL_DELTA:
            self.gcode.respond_info(f'Start axes level, delta: {delta}')
            m = self.motion[main_axis]
            force_exit = False
            while True:
                if not m['move_dir'][1]:
                    self._detect_move_dir(main_axis)
                buzz = False if self.fast_threshold and m['magnitude'] > self.fast_threshold else True
                delta = (m['magnitude'] - init_min_magnitude)
                m['moving_msteps'] = max(int(delta / self.steps_coeff), 1)
                self._stepper_move(m['lookup_stepper'], m['moving_msteps'] * self.move_len * m['move_dir'][0])
                m['actual_msteps'] += m['moving_msteps'] * m['move_dir'][0]
                m['new_magnitude'] = self._measure(m['stepper'], buzz)
                if self.respond:
                    self.gcode.respond_info(f"{main_axis.upper()}-New magnitude: {m['new_magnitude']} on "
                                            f"{m['moving_msteps'] * m['move_dir'][0]}/{self.microsteps} step move")
                if m['new_magnitude'] > m['magnitude']:
                    self._stepper_move(m['lookup_stepper'], m['moving_msteps'] * self.move_len * m['move_dir'][0] * -1)
                    m['actual_msteps'] -= m['moving_msteps'] * m['move_dir'][0]
                    if self.retry_tolerance and m['magnitude'] > self.retry_tolerance:
                        self.motion['retries'] += 1
                        if self.motion['retries'] > self.max_retries:
                            self.gcode.respond_info(
                                f"{main_axis.upper()} Motors adjusted by {m['actual_msteps']}/"
                                f"{self.microsteps} step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                            raise self.gcode.error('Too many retries')
                        self.gcode.respond_info(
                            f"Retries: {self.motion['retries']}/{self.max_retries} Data in loop is incorrect! ")
                        m['move_dir'][1] = 0
                        continue
                    force_exit = True
                delta = round(m['new_magnitude'] - init_min_magnitude, 2)
                if delta < AXES_LEVEL_DELTA or m['new_magnitude'] < init_min_magnitude or force_exit:
                    m['magnitude'] = m['new_magnitude']
                    if self.respond: self.gcode.respond_info(
                        f"Axes are leveled: {main_axis.upper()}: {init_max_magnitude} --> "
                        f"{m['new_magnitude']} {sec_axis.upper()}: {init_min_magnitude}, delta: {delta}")
                    # Measure new shifted magnitude on second axis
                    self.motion[sec_axis]['magnitude'] = self._measure(self.motion[sec_axis]['stepper'], True)
                    if self.respond:
                        self.gcode.respond_info(f"{sec_axis.upper()}-New magnitude: {self.motion[sec_axis]['magnitude']}")
                    return
                m['magnitude'] = m['new_magnitude']
                continue

    def _detect_move_dir(self, axis):
        # Determine movement direction
        self._stepper_move(self.motion[axis]['lookup_stepper'], self.motion[axis]['moving_msteps'] * self.move_len)
        self.motion[axis]['actual_msteps'] += self.motion[axis]['moving_msteps']
        self.motion[axis]['new_magnitude'] = self._measure(self.motion[axis]['stepper'], True)
        if self.respond: self.gcode.respond_info(
            f"{axis.upper()}-New magnitude: {self.motion[axis]['new_magnitude']}"
            f" on {self.motion[axis]['move_dir'][0] * self.motion[axis]['moving_msteps']}/{self.microsteps} step move")
        self.motion[axis]['move_dir'] = [-1, 'Backward'] if (self.motion[axis]['new_magnitude']
                                                             > self.motion[axis]['magnitude']) else [1, 'Forward']
        if self.respond: self.gcode.respond_info(f"{axis.upper()}-Movement direction: {self.motion[axis]['move_dir'][1]}")
        self.motion[axis]['magnitude'] = self.motion[axis]['new_magnitude']

    def _final_sync(self):
        # Axes calibration to zero magnitude
        if self.motion['min_axis'] == self.axes[-1]: self.axes.reverse()
        for axis in itertools.cycle(self.axes):
            m = self.motion[axis]
            if not m['out_msg']:
                if not m['move_dir'][1]: self._detect_move_dir(axis)
                buzz = False if self.fast_threshold and m['magnitude'] > self.fast_threshold else True
                m['moving_msteps'] = max(int(m['magnitude'] / self.steps_coeff), 1)
                self._stepper_move(m['lookup_stepper'], m['moving_msteps'] * self.move_len * m['move_dir'][0])
                m['actual_msteps'] += m['moving_msteps'] * m['move_dir'][0]
                m['new_magnitude'] = self._measure(m['stepper'], buzz)
                if self.respond: self.gcode.respond_info(
                    f"{axis.upper()}-New magnitude: {m['new_magnitude']}"
                    f" on {m['moving_msteps'] * m['move_dir'][0]}/{self.microsteps} step move")
                if m['new_magnitude'] > m['magnitude']:
                    self._stepper_move(m['lookup_stepper'], m['moving_msteps'] * self.move_len * m['move_dir'][0] * -1)
                    m['actual_msteps'] -= m['moving_msteps'] * m['move_dir'][0]
                    if self.retry_tolerance and m['magnitude'] > self.retry_tolerance:
                        self.motion['retries'] += 1
                        if self.motion['retries'] > self.max_retries:
                            self.gcode.respond_info(
                                f"{axis.upper()} Motors adjusted by {m['actual_msteps']}/{self.microsteps}"
                                f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                            raise self.gcode.error('Too many retries')
                        if self.respond: self.gcode.respond_info(
                            f"{axis.upper()} Retries: {self.motion['retries']}/{self.max_retries} Back on last magnitude:"
                            f" {m['magnitude']} on {m['actual_msteps']}/{self.microsteps} step to reach {self.retry_tolerance}")
                        m['move_dir'][1] = 0
                        continue
                    m['out_msg'] = (f"{axis.upper()}-Motors adjusted by {m['actual_msteps']}/"
                                f"{self.microsteps} step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                    continue
                m['magnitude'] = m['new_magnitude']
            else:
                if self.motion[self.axes[0]]['out_msg'] and self.motion[self.axes[1]]['out_msg']: break

    def cmd_RUN_SYNC(self, gcmd):
        self.axes = ['x', 'y']
        self.status.reset()
        # Live variables
        accel_chip = gcmd.get('ACCEL_CHIP', self.accel_chip)
        self.chip_config = self.printer.lookup_object(accel_chip)
        self.steps_coeff = gcmd.get_int('STEPS_COEFF', self.steps_coeff, minval=5000, maxval=999999)
        self.fast_threshold = gcmd.get_int('FAST_THRESHOLD', self.fast_threshold, minval=0, maxval=999999)
        self.retry_tolerance = gcmd.get_int('RETRY_TOLERANCE', self.retry_tolerance, minval=0, maxval=999999)
        self.max_retries = gcmd.get_int('RETRIES', self.max_retries, minval=0, maxval=10)
        # Run
        self._prestart()
        self._static_measure()
        self.motion = {}
        if self.respond: self.gcode.respond_info('Motors synchronization started')
        # Init axes
        for axis in self.axes:
            self.motion[axis] = {}
            self.motion[axis]['stepper'] = 'stepper_' + axis.lower()
            self.motion[axis]['lookup_stepper'] = self.force_move._lookup_stepper({'STEPPER': self.motion[axis]['stepper']})
            self.motion[axis]['init_magnitude'] = self._measure(self.motion[axis]['stepper'], True)
            self.motion[axis]['magnitude'] = self.motion[axis]['init_magnitude']
            self.motion[axis]['new_magnitude'] = 0
            self.motion[axis]['move_dir'] = [1, 'Forward']
            self.motion[axis]['moving_msteps'] = max(int(self.motion[axis]['magnitude'] / self.steps_coeff), 1)
            self.motion[axis]['actual_msteps'] = 0
            self.motion[axis]['out_msg'] = ''
            if self.respond: self.gcode.respond_info(f"{axis.upper()}-Initial magnitude: {self.motion[axis]['init_magnitude']}")
        self.motion['retries'] = 0
        self.motion['max_axis'] = max(self.motion, key=lambda level: self.motion[level]['init_magnitude'] if isinstance(
            self.motion[level], dict) and 'init_magnitude' in self.motion[level] else float('-inf'))
        self.motion['min_axis'] = min(self.motion, key=lambda level: self.motion[level]['init_magnitude'] if isinstance(
            self.motion[level], dict) and 'init_magnitude' in self.motion[level] else float('inf'))
        self._detect_move_dir(self.motion['max_axis'])
        self.motion[self.motion['min_axis']]['move_dir'][1] = 0
        self._axes_level()
        self._final_sync()
        self.status.check_retry_result('done')
        # Info
        for axis in self.axes:
            self.gcode.respond_info(f"{self.motion[axis]['out_msg']}\n")

    def get_status(self, eventtime=None, user=False):
        if not user:
            return self.status.get_status(eventtime)
        else:
            now = self.printer.get_reactor().monotonic()
            return bool(list(self.status.get_status(now).values())[0])

def load_config(config):
    return MotorsSync(config)
