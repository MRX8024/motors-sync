# Motors synchronization script
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, time, itertools
from datetime import datetime
import numpy as np
from . import z_tilt

PLOT_PATH = '~/printer_data/config/adxl_results/motors_sync'
PIN_MIN_TIME = 0.010            # Minimum wait time to enable hardware pin
MOTOR_STALL_TIME = 0.100        # Minimum wait time to enable motor pin
ACCEL_FILTER_THRESHOLD = 3000   # Accelerometer filter disabled at lower sampling rate
AXES_LEVEL_DELTA = 2000         # Magnitude difference between axes in _axes_level()
LEVELING_KINEMATICS = (         # Kinematics with interconnected axes
    ['corexy', 'limited_corexy'])

MATH_MODELS = {
    "polynomial": lambda fx, coeffs:
        max(np.roots([*coeffs[:-1], coeffs[-1] - fx]).real),
    "power": lambda fx, coeffs:
        (fx / coeffs[0]) ** (1 / coeffs[1]),
    "root": lambda fx, coeffs:
        (fx**2 - 2*coeffs[1]*fx + coeffs[1]**2) / coeffs[0]**2,
    "hyperbolic": lambda fx, coeffs:
        coeffs[0] / (fx - coeffs[1]),
    "exponential": lambda fx, coeffs:
        np.log((fx - coeffs[2]) / coeffs[0]) / coeffs[1],
}

class MotionAxis:
    VALID_MSTEPS = [256, 128, 64, 32, 16, 8, 0]
    def __init__(self, name, sync, config):
        self.name = name
        self.sync = sync
        self.config = config
        self.printer = config.get_printer()
        self.move_dir = [1, 'unknown']
        self.move_msteps = 2
        self.actual_msteps = 0
        self.check_msteps = 0
        self.backup_msteps = 0
        self.init_magnitude = 0.
        self.magnitude = 0.
        self.new_magnitude = 0.
        self.curr_retry = 0
        self.is_finished = False
        self.log = []
        self.aclient = None
        stepper = 'stepper_' + name
        st_section = config.getsection(stepper)
        min_pos = st_section.getfloat('position_min', 0)
        max_pos = st_section.getfloat('position_max')
        rd = st_section.getfloat('rotation_distance')
        fspr = st_section.getint('full_steps_per_rotation', 200)
        self.limits = (min_pos + 10, max_pos - 10, (min_pos + max_pos) / 2)
        self.do_buzz = True
        self.rel_buzz_d = rd / fspr * 5
        msteps_dict = {m: m for m in self.VALID_MSTEPS}
        self.microsteps = self.config.getchoice(
            f'microsteps_{name}', msteps_dict, default=0)
        if not self.microsteps:
            self.microsteps = self.config.getchoice(
                'microsteps', msteps_dict, default=16)
        self.move_d = rd / fspr / self.microsteps
        self.chip_name = config.get(f'accel_chip_{name}', '')
        if not self.chip_name:
            self.chip_name = config.get('accel_chip')
        self.init_chip_config(self.chip_name)
        self._init_models()
        self.conf_fan = config.get(f'head_fan_{name}', '')
        if not self.conf_fan:
            self.conf_fan = config.get('head_fan', None)
        mss = self.microsteps / 2
        self.max_step_size = self.config.getint(
            f'max_step_size_{name}', default=0, minval=1, maxval=mss)
        if not self.max_step_size:
            self.max_step_size = self.config.getint(
                'max_step_size', default=3, minval=1, maxval=mss)
        self.axes_steps_diff = self.config.getint(
            f'axes_steps_diff_{name}', default=0, minval=1)
        if not self.axes_steps_diff:
            self.axes_steps_diff = self.config.getint(
                'axes_steps_diff', self.max_step_size + 1, minval=1)
        self.retry_tolerance = self.config.getint(
            f'retry_tolerance_{name}', default=0, minval=0)
        if not self.retry_tolerance:
            self.retry_tolerance = self.config.getint(
                'retry_tolerance', default=0, minval=0)
        self.max_retries = self.config.getint(
            f'retries_{name}', default=0, minval=0, maxval=10)
        if not self.max_retries:
            self.max_retries = self.config.getint(
                'retries', default=0, minval=0, maxval=10)

    def handle_connect(self):
        self._init_steppers()
        self._init_fan()

    def flush_motion_data(self):
        self.move_dir = [1, 'unknown']
        self.move_msteps = 2
        self.actual_msteps = 0
        self.check_msteps = 0
        self.backup_msteps = 0
        self.init_magnitude = 0.
        self.magnitude = 0.
        self.new_magnitude = 0.
        self.curr_retry = 0
        self.is_finished = False
        self.log = []

    def _init_steppers(self):
        steppers = [(s.get_name(), s) for s in self.sync.kin.get_steppers()
                    if self.name in s.get_name()]
        if len(steppers) not in (2,):
            raise self.config.error(f"motors_sync: Not support "
                                    f"'{len(steppers)}' count of motors")
        for _steppers in steppers:
            st_section = self.config.getsection(_steppers[0])
            st_msteps = st_section.getint('microsteps')
            if self.microsteps > st_msteps:
                raise self.config.error(
                    f'motors_sync: Invalid microsteps count, cannot be '
                    f'more than steppers, {self.microsteps} vs {st_msteps}')
        self.steppers = steppers

    def _init_chip_filter(self):
        filters = ['default', 'median', 'kalman']
        filters_d = {m: m for m in filters}
        filter = self.config.getchoice(f'chip_filter_{self.name}',
                                       filters_d, 'default').lower()
        if filter == 'default':
            filter = self.config.getchoice('chip_filter',
                                           filters_d, 'median').lower()
        if filter == 'median':
            window = self.config.getint(f'median_size_{self.name}',
                                        '', minval=3, maxval=9)
            if not window:
                window = self.config.getint('median_size', default=3,
                                            minval=3, maxval=9)
            if window % 2 == 0: raise self.config.error(
                f"motors_sync: parameter 'median_size' cannot be even")
            self.chip_filter = lambda samples, w=window: np.median(
                [samples[i - w:i + w + 1]
                 for i in range(w, len(samples) - w)], axis=1)
        elif filter == 'kalman':
            coeffs = self.config.getfloatlist(f'kalman_coeffs_{self.name}',
                              default=tuple('' for _ in range(6)), count=6)
            if not all(coeffs):
                coeffs = self.config.getfloatlist('kalman_coeffs',
                    default=tuple((1.1, 1., 1e-1, 1e-2, .5, 1.)), count=6)
            self.chip_filter = KalmanLiteFilter(*coeffs).process_samples

    def init_chip_config(self, chip_name):
        self.chip_config = self.printer.lookup_object(chip_name)
        if hasattr(self.chip_config, 'data_rate'):
            if self.chip_config.data_rate > ACCEL_FILTER_THRESHOLD:
                self._init_chip_filter()
            else:
                self.chip_filter = lambda data: data
        elif chip_name == 'beacon':
            # Beacon sampling rate > ACCEL_FILTER_THRESHOLD
            self._init_chip_filter()
        else:
            raise self.config.error(f"motors_sync: Unknown accelerometer"
                                    f" '{chip_name}' sampling rate")

    def _init_models(self):
        models = {
            'linear': {'ct': 2, 'a': None, 'f': MATH_MODELS['polynomial']},
            'quadratic': {'ct': 3, 'a': None, 'f': MATH_MODELS['polynomial']},
            'power': {'ct': 2, 'a': None, 'f': MATH_MODELS['power']},
            'root': {'ct': 2, 'a': 0, 'f': MATH_MODELS['root']},
            'hyperbolic': {'ct': 2, 'a': 0, 'f': MATH_MODELS['hyperbolic']},
            'exponential': {'ct': 3, 'a': 0, 'f': MATH_MODELS['exponential']}
        }
        model_name = self.config.get(f'model_{self.name}', '').lower()
        if not model_name:
            model_name = self.config.get('model', 'linear').lower()
        coeffs_vals = self.config.getfloatlist(f'model_coeffs_{self.name}', '')
        if not coeffs_vals:
            coeffs_vals = self.config.getfloatlist('model_coeffs', [20000, 0])
        coeffs_args = [chr(97 + i) for i in range(len(coeffs_vals) + 1)]
        model_coeffs = {arg: float(val)
                        for arg, val in zip(coeffs_args, coeffs_vals)}
        model_config = models.get(model_name, None)
        if model_config is None:
            raise self.config.error(
                f"motors_sync: Invalid model '{model_name}'")
        if len(model_coeffs) != model_config['ct']:
            raise self.config.error(
                f"motors_sync: Model {model_name} requires "
                f"{model_config['ct']} coefficients")
        if model_coeffs['a'] == model_config['a']:
            raise self.config.error(
                f"motors_sync: Coefficient 'a' cannot be "
                f"{model_coeffs['a']} for a '{model_name}' model")
        self.model_name = model_name
        self.model_coeffs = tuple(model_coeffs.values())
        self.model_solve = lambda fx=None: model_config['f'](
            fx if fx is not None else self.new_magnitude,
            self.model_coeffs)

    def _create_fan_switch(self, method):
        if method == 'heater_fan':
            def fan_switch(on=True):
                if not self.fan:
                    return
                now = self.sync.reactor.monotonic()
                print_time = (self.fan.fan.get_mcu().
                              estimated_print_time(now))
                speed = self.fan.last_speed if on else .0
                self.fan.fan.set_speed(value=speed,
                    print_time=print_time + PIN_MIN_TIME)
        elif method == 'temperature_fan':
            def fan_switch(on=True):
                if not self.fan:
                    return
                if not self.last_fan_target:
                    self.last_fan_target = self.fan.target_temp
                target = self.last_fan_target if on else .0
                self.fan.set_temp(target)
            self.last_fan_target = 0
        else:
            def fan_switch(_):
                return
        self.fan_switch = fan_switch

    def _init_fan(self):
        fan_methods = ['heater_fan', 'temperature_fan']
        if self.conf_fan is None:
            # Create a stub
            self._create_fan_switch(None)
            return
        for method in fan_methods:
            try:
                self.fan = self.printer.lookup_object(
                    f'{method} {self.conf_fan}')
                self._create_fan_switch(method)
                return
            except:
                continue
        raise self.config.error(f"motors_sync: Unknown fan or "
                                f"fan method '{self.conf_fan}'")


class MotorsSync:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.force_move = self.printer.load_object(config, 'force_move')
        self.stepper_en = self.printer.load_object(config, 'stepper_enable')
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.status = z_tilt.ZAdjustStatus(self.printer)
        # Read config
        self._init_axes()
        self._init_sync_method()
        # Register commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_SYNC_MOTORS,
                                    desc=self.cmd_SYNC_MOTORS_help)
        self.gcode.register_command('SYNC_MOTORS_CALIBRATE',
                                    self.cmd_SYNC_MOTORS_CALIBRATE,
                                    desc=self.cmd_SYNC_MOTORS_CALIBRATE_help)
        # Variables
        self._init_stat_manager()

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = min(self.toolhead.max_accel, 5000)
        self.kin = self.toolhead.get_kinematics()
        self.reactor = self.printer.get_reactor()
        for axis in self.motion.values():
            axis.handle_connect()

    def _init_axes(self):
        valid_axes = ['x', 'y']
        printer_section = self.config.getsection('printer')
        self.conf_kin = printer_section.get('kinematics')
        if self.conf_kin in LEVELING_KINEMATICS:
            self.do_level = True
            axes = [a.lower() for a in self.config.getlist(
                'axes', count=2, default=['x', 'y'])]
        elif self.conf_kin == 'cartesian':
            self.do_level = False
            axes = [a.lower() for a in self.config.getlist('axes')]
        else:
            raise self.config.error(f"motors_sync: Not supported "
                                    f"kinematics '{self.conf_kin}'")
        if any(axis not in valid_axes for axis in axes):
            raise self.config.error(f"motors_sync: Invalid axes "
                                    f"parameter '{','.join(axes)}'")
        self.motion = {ax: MotionAxis(ax, self, self.config) for ax in axes}
        if self.conf_kin not in LEVELING_KINEMATICS:
            return
        # Apply restrictions for LEVELING_KINEMATICS kinematics
        common_attr = ['chip_name', 'microsteps', 'model_name',
                       'model_coeffs', 'max_step_size', 'axes_steps_diff']
        for attr in common_attr:
            diff = set([getattr(cls, attr) for cls in self.motion.values()])
            if len(diff) < 2:
                continue
            params_str = ', '.join(f"'{attr}: {v}'" for v in diff)
            raise self.config.error(
                f"motors_sync: Options {params_str} cannot be "
                f"different for a '{self.conf_kin}' kinematics")

    def _init_sync_method(self):
        methods = ['sequential', 'alternately', 'synchronous', 'default']
        self.sync_method = self.config.getchoice(
            'sync_method', {m: m for m in methods}, 'default')
        if self.sync_method == 'default':
            if self.conf_kin in LEVELING_KINEMATICS:
                self.sync_method = methods[1]
            else:
                self.sync_method = methods[0]
        elif (self.sync_method in methods[1:]
              and self.conf_kin not in LEVELING_KINEMATICS):
            raise self.config.error(f"motors_sync: Invalid sync method: "
                                    f"{self.sync_method} for '{self.conf_kin}'")

    def _init_stat_manager(self):
        command = 'SYNC_MOTORS_STATS'
        filename = 'sync_stats.csv'
        format = 'axis,status,magnitudes,steps,msteps,retries,date,'
        def log_parser(log):
            a = {}
            out = []
            for p in log:
                a.setdefault(p[0], {
                    'count': 0,
                    'success': 0,
                    'msteps': 0,
                    'magnitudes': [0., 0., 0., 999999.],
                    'retries': 0,
                })
                a[p[0]]['count'] += 1
                if p[1]:
                    a[p[0]]['success'] += 1
                a[p[0]]['magnitudes'][:2] = (np.add(
                    a[p[0]]['magnitudes'][:2], (p[2][-2], p[2][0])))
                if p[2].max() > a[p[0]]['magnitudes'][2]:
                    a[p[0]]['magnitudes'][2] = p[2].max()
                if p[2].min() < a[p[0]]['magnitudes'][3]:
                    a[p[0]]['magnitudes'][3] = p[2].min()
                a[p[0]]['msteps'] += abs(p[3][-2] / (p[4] / 16))
                a[p[0]]['retries'] += p[5]
            for axis, a in a.items():
                cf_microsteps = self.motion[axis.lower()].microsteps
                st_microsteps = a['msteps'] / a['count'] * (cf_microsteps / 16)
                out.append(f"""
                {axis.upper()} axis statistics:
                Successfully synced:     {a['success'] / a['count'] * 100:.2f}%
                Average start magnitude: {a['magnitudes'][1] / a['count']:.2f}
                Average end magnitude:   {a['magnitudes'][0] / a['count']:.2f}
                Average msteps count:    {st_microsteps:.0f}/{cf_microsteps}
                Average retries count:   {a['retries'] / a['count']:.2f}
                Min detected magnitude:  {a['magnitudes'][3]:.2f}
                Max detected magnitude:  {a['magnitudes'][2]:.2f}
                Synchronization count:   {a['count']}
                """)
                out.append('')
            return out
        manager = StatisticsManager(self.gcode, command,
                                    filename, log_parser, format)
        def write_log(axis=None):
            if manager.error:
                return
            status = axis is None
            for axis in ([axis] if axis else [
                 a for n, a in self.motion.items() if n in self.axes]):
                if not axis.actual_msteps:
                    continue
                name = axis.name
                magnitudes, pos = zip(*axis.log)
                msteps = axis.microsteps
                retries = axis.curr_retry
                date = datetime.now().strftime('%Y-%m-%d')
                manager.write_log([name, status, magnitudes,
                                   pos, msteps, retries, date])
        self.write_log = write_log

    def _send(self, params):
        self.gcode._process_commands([params], False)

    def _stepper_switch(self, stepper, mode, ontime=\
            MOTOR_STALL_TIME, offtime=MOTOR_STALL_TIME):
        self.toolhead.dwell(ontime)
        print_time = self.toolhead.get_last_move_time()
        el = self.stepper_en.enable_lines[stepper]
        if mode:
            el.motor_enable(print_time)
        else:
            el.motor_disable(print_time)
        self.toolhead.dwell(offtime)

    def _stepper_move(self, mcu_stepper, dist):
        self.force_move.manual_move(mcu_stepper, dist,
            self.travel_speed, self.travel_accel)

    def _single_move(self, axis, mcu_stepper=None, dir=1):
        # Move <axis>1 stepper motor by default
        if mcu_stepper is None:
            mcu_stepper = axis.steppers[1][1]
        move_msteps = axis.move_msteps * axis.move_dir[0] * dir
        dist = axis.move_d * move_msteps
        axis.actual_msteps += move_msteps
        axis.check_msteps += move_msteps
        self._stepper_move(mcu_stepper, dist)

    def _buzz(self, axis, rel_moves=25):
        # Fading oscillations by <axis>1 stepper
        last_abs_pos = 0
        self._stepper_switch(axis.steppers[0][0], 0, PIN_MIN_TIME, PIN_MIN_TIME)
        for osc in reversed(range(0, rel_moves)):
            abs_pos = axis.rel_buzz_d * (osc / rel_moves)
            for inv in [1, -1]:
                abs_pos *= inv
                dist = (abs_pos - last_abs_pos)
                last_abs_pos = abs_pos
                self._stepper_move(axis.steppers[1][1], dist)

    def _wait_samples(self, aclient):
        lim = self.reactor.monotonic() + 5.
        while True:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            if aclient.msgs:
                last_mcu_time = aclient.msgs[-1]['data'][-1][0]
                if last_mcu_time > aclient.request_end_time:
                    return True
                elif now > lim:
                    raise self.gcode.error(
                        'motors_sync: No data from accelerometer')

    def _get_accel_samples(self, aclient):
        self._wait_samples(aclient)
        raw_data = np.concatenate([np.array(m['data']) for m in aclient.msgs])
        start_idx = np.searchsorted(raw_data[:, 0],
                    aclient.request_start_time, side='left')
        end_idx = np.searchsorted(raw_data[:, 0],
                    aclient.request_end_time, side='right')
        t_accels = raw_data[start_idx:end_idx]
        return t_accels[:, 1:]

    def _calc_magnitude(self, axis):
        # Calculate impact magnitude
        vects = self._get_accel_samples(axis.aclient)
        vects_len = vects.shape[0]
        # Kalman filter may distort the first values, or in some
        # cases there may be residual values of toolhead inertia.
        # It is better to take a shifted zone from zero.
        static_zone = range(vects_len // 5, vects_len // 3)
        z_cut_zone = vects[static_zone, :]
        z_axis = np.mean(np.abs(z_cut_zone), axis=0).argmax()
        xy_mask = np.arange(vects.shape[1]) != z_axis
        magnitudes = np.linalg.norm(vects[:, xy_mask], axis=1)
        # Add median, Kalman or none filter
        magnitudes = axis.chip_filter(magnitudes)
        # Calculate static noise
        static = np.mean(magnitudes[static_zone])
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitudes)[-5:])
        magnitude = np.around(magnitude - static, 2)
        axis.log.append([int(magnitude), axis.actual_msteps])
        return magnitude

    def _measure(self, axis):
        # Measure the impact
        if axis.do_buzz:
            self._buzz(axis)
        axis.aclient.msgs.clear()
        stepper = axis.steppers[0][0]
        self._stepper_switch(stepper, 1, PIN_MIN_TIME)
        self._stepper_switch(stepper, 0, PIN_MIN_TIME)
        axis.aclient.request_start_time = self.toolhead.get_last_move_time()
        self._stepper_switch(stepper, 1)
        axis.aclient.request_end_time = self.toolhead.get_last_move_time()
        self._stepper_switch(stepper, 0, PIN_MIN_TIME, PIN_MIN_TIME)
        return self._calc_magnitude(axis)

    def _homing(self):
        # Homing and going to center
        now = self.reactor.monotonic()
        axes, confs = zip(*self.motion.items())
        if ''.join(axes) not in self.kin.get_status(now)['homed_axes']:
            self._send(f"G28 {' '.join(axes)}")
        center_pos = ' '.join(f'{a}{c.limits[2]}' for a, c in zip(axes, confs))
        self._send(f"G0 {center_pos} F{self.travel_speed * 60}")
        self.toolhead.dwell(MOTOR_STALL_TIME)

    def _handle_state(self, axis, state=''):
        name = axis.name.upper()
        msg = ''
        if state == 'stepped':
            msteps = axis.move_msteps * axis.move_dir[0]
            msg = (f"{name}-New magnitude: {axis.new_magnitude} "
                   f"on {msteps}/{axis.microsteps} step move")
        elif state == 'static':
            msg = f"{name}-New magnitude: {axis.new_magnitude}"
        elif state == 'direction':
            msg = f"{name}-Movement direction: {axis.move_dir[1]}"
        elif state == 'start':
            axis.flush_motion_data()
            axis.fan_switch(False)
            axis.aclient = axis.chip_config.start_internal_client()
            axis.init_magnitude = axis.magnitude = self._measure(axis)
            msg = (f"{axis.name.upper()}-Initial magnitude: "
                   f"{axis.init_magnitude}")
        elif state == 'done':
            axis.fan_switch(True)
            axis.aclient.finish_measurements()
            self._stepper_switch(axis.steppers[0][0], 1,
                                 PIN_MIN_TIME, PIN_MIN_TIME)
            msg = (f"{name}-Motors adjusted by {axis.actual_msteps}/"
                   f"{axis.microsteps} step, magnitude "
                   f"{axis.init_magnitude} --> {axis.magnitude}")
        elif state == 'retry':
            axis.move_dir[1] = 'unknown'
            msg = (f"{name}-Retries: {axis.curr_retry}/{axis.max_retries} "
                   f"Back on last magnitude: {axis.magnitude} on "
                   f"{axis.actual_msteps}/{axis.microsteps} step "
                   f"to reach {axis.retry_tolerance}")
        elif state == 'error':
            for axis in [c for a, c in self.motion.items() if a in self.axes]:
                axis.fan_switch(True)
                axis.aclient.finish_measurements()
            raise self.gcode.error('Too many retries')
        self.gcode.respond_info(msg, True)

    def _detect_move_dir(self, axis):
        # Determine movement direction
        axis.move_dir = [1, 'unknown']
        self._single_move(axis)
        axis.new_magnitude = self._measure(axis)
        self._handle_state(axis, 'stepped')
        if axis.new_magnitude > axis.magnitude:
            axis.move_dir = [-1, 'Backward']
        else:
            axis.move_dir = [1, 'Forward']
        self._handle_state(axis, 'direction')
        axis.magnitude = axis.new_magnitude

    def _axes_level(self, m, s):
        # Axes leveling by magnitude
        # "m" is a main axis, "s" is a second axis
        delta = m.init_magnitude - s.init_magnitude
        if delta <= AXES_LEVEL_DELTA:
            return
        self.gcode.respond_info(
            f'Start axes level, delta: {delta:.2f}', True)
        force_exit = False
        while True:
            # Note: m.axes_steps_diff == s.axes_steps_diff
            steps_diff = abs(abs(m.check_msteps) - abs(s.check_msteps))
            if steps_diff >= m.axes_steps_diff:
                s.new_magnitude = s.magnitude = self._measure(s)
                self._handle_state(s, 'static')
                m.check_msteps, s.check_msteps = 0, 0
            if m.move_dir[1] == 'unknown':
                self._detect_move_dir(m)
            steps_delta = int(m.model_solve() - m.model_solve(s.magnitude))
            m.move_msteps = min(max(steps_delta, 1), m.max_step_size)
            self._single_move(m)
            m.new_magnitude = self._measure(m)
            self._handle_state(m, 'stepped')
            if m.new_magnitude > m.magnitude:
                self._single_move(m, dir=-1)
                if m.retry_tolerance and m.magnitude > m.retry_tolerance:
                    m.curr_retry += 1
                    if m.curr_retry > m.max_retries:
                        self._handle_state(m, 'done')
                        self.write_log(m)
                        self._handle_state(m, 'error')
                    self._handle_state(m, 'retry')
                    continue
                force_exit = True
            m.magnitude = m.new_magnitude
            delta = m.new_magnitude - s.magnitude
            if (delta < AXES_LEVEL_DELTA
                    or m.new_magnitude < s.magnitude
                    or force_exit):
                self.gcode.respond_info(
                    f"Axes are leveled: {m.name.upper()}: "
                    f"{m.init_magnitude} --> {m.new_magnitude}, "
                    f"{s.name.upper()}: {s.init_magnitude} "
                    f"--> {s.magnitude}, delta: {delta:.2f}", True)
                return
            continue

    def _single_sync(self, m, check_axis=False):
        # "m" is a main axis, just single axis
        if check_axis:
            m.new_magnitude = self._measure(m)
            self._handle_state(m, 'static')
            m.magnitude = m.new_magnitude
            return
        if m.move_dir[1] == 'unknown':
            if not m.actual_msteps or m.curr_retry:
                m.new_magnitude = self._measure(m)
                m.magnitude = m.new_magnitude
                self._handle_state(m, 'static')
            if (not m.actual_msteps
                    and m.retry_tolerance
                    and m.new_magnitude < m.retry_tolerance):
                m.is_finished = True
                return
            self._detect_move_dir(m)
        m.move_msteps = min(max(
            int(m.model_solve()), 1), m.max_step_size)
        self._single_move(m)
        m.new_magnitude = self._measure(m)
        self._handle_state(m, 'stepped')
        if m.new_magnitude > m.magnitude:
            self._single_move(m, dir=-1)
            if m.retry_tolerance and m.magnitude > m.retry_tolerance:
                m.curr_retry += 1
                if m.curr_retry > m.max_retries:
                    # Write error in log
                    self.write_log(m)
                    self._handle_state(m, 'done')
                    self._handle_state(m, 'error')
                self._handle_state(m, 'retry')
                return
            m.is_finished = True
            return
        m.magnitude = m.new_magnitude

    def _run_sync(self):
        # Axes synchronization
        if self.sync_method == 'alternately' and len(self.axes) > 1:
            # Find min and max axes for axes leveling
            min_ax, max_ax = [c for c in sorted(
                self.motion.values(), key=lambda i: i.init_magnitude)]
            self._axes_level(max_ax, min_ax)
            axes = self.axes[::-1] if max_ax.name == self.axes[0] else self.axes
            for axis in itertools.cycle(axes):
                m = self.motion[axis]
                if m.is_finished:
                    if all(self.motion[ax].is_finished for ax in self.axes):
                        break
                    continue
                self._single_sync(m)
        elif self.sync_method == 'synchronous' and len(self.axes) > 1:
            check_axis = False
            cycling = itertools.cycle(self.axes)
            max_ax = [c for c in sorted(
                self.motion.values(), key=lambda i: i.init_magnitude)][-1]
            self._detect_move_dir(max_ax)
            while True:
                axis = next(cycling)
                cycling, cycle = itertools.tee(cycling)
                m = self.motion[axis]
                sec = next(cycle)
                s = self.motion[sec]
                if m.is_finished:
                    if all(self.motion[ax].is_finished for ax in self.axes):
                        break
                    continue
                if m.magnitude < s.magnitude and not s.is_finished:
                    # None: m['axes_steps_diff'] == s['axes_steps_diff']
                    steps_diff = abs(abs(m.check_msteps) - abs(s.check_msteps))
                    if steps_diff >= m.axes_steps_diff:
                        check_axis = True
                        m.check_msteps, s.check_msteps = 0, 0
                    else:
                        continue
                self._single_sync(m, check_axis)
                check_axis = False
        elif self.sync_method == 'sequential' or len(self.axes) == 1:
            for axis in self.axes:
                m = self.motion[axis]
                # To skip _measure() in _single_sync()
                self._detect_move_dir(m)
                while True:
                    if m.is_finished:
                        if all(self.motion[ax].is_finished for ax in self.axes):
                            return
                        break
                    self._single_sync(m)
        else:
            raise self.gcode.error('Error in sync methods!')

    cmd_SYNC_MOTORS_help = 'Start motors synchronization'
    def cmd_SYNC_MOTORS(self, gcmd, force_run=False):
        # Live variables
        axes_from_gcmd = gcmd.get('AXES', '')
        if axes_from_gcmd:
            axes_from_gcmd = axes_from_gcmd.split(',')
            if any([axis not in self.motion.keys()
                    for axis in axes_from_gcmd]):
                raise self.gcode.error(f'Invalid axes parameter')
            self.axes = [axis for axis in axes_from_gcmd]
        else:
            self.axes = list(self.motion.keys())
        chip = gcmd.get(f'ACCEL_CHIP', '')
        for axis in self.axes:
            m = self.motion[axis]
            ax_chip = gcmd.get(f'ACCEL_CHIP_{axis.upper()}', chip).lower()
            if ax_chip and ax_chip != m.chip_name:
                try:
                    self.printer.lookup_object(ax_chip)
                except Exception as e:
                    raise self.gcode.error(e)
                self.motion[axis].init_chip_config(ax_chip)
            retry_tol = gcmd.get_int(f'RETRY_TOLERANCE_{axis.upper()}', 0)
            if not retry_tol:
                retry_tol = gcmd.get_int(f'RETRY_TOLERANCE', 0)
            if retry_tol:
                m.retry_tolerance = retry_tol
            retries = gcmd.get_int(f'RETRIES_{axis.upper()}', 0)
            if not retries:
                retries = gcmd.get_int(f'RETRIES', 0)
            if retries:
                m.max_retries = retries
        # Run
        self.status.reset()
        self._homing()
        self.gcode.respond_info('Motors synchronization started', True)
        # Init axes
        for axis in self.axes:
            self._handle_state(self.motion[axis], 'start')
        # Check if all axes in tolerance
        if not force_run and all(m.init_magnitude < m.retry_tolerance
             for m in (self.motion[ax] for ax in self.axes)):
            retry_tols = ''
            for axis in self.axes:
                m = self.motion[axis]
                m.aclient.finish_measurements()
                m.fan_switch(True)
                retry_tols += f'{m.name.upper()}: {m.retry_tolerance}, '
            self.gcode.respond_info(f"Motors magnitudes are in "
                                    f"tolerance: {retry_tols}", True)
        else:
            self._run_sync()
            # Info
            for axis in self.axes:
                self._handle_state(self.motion[axis], 'done')
        self.status.check_retry_result('done')
        self.write_log()

    cmd_SYNC_MOTORS_CALIBRATE_help = 'Calibrate synchronization process model'
    def cmd_SYNC_MOTORS_CALIBRATE(self, gcmd):
        # Calibrate sync model and model coeffs
        if not hasattr(self, 'cal'):
            cal = MotorsSyncCalibrate(self)
        cal.run_calibrate(gcmd)
        self.status.reset()

    def get_status(self, eventtime=None, user=False):
        if not user:
            return self.status.get_status(eventtime)
        else:
            now = self.reactor.monotonic()
            return bool(list(self.status.get_status(now).values())[0])


class MotorsSyncCalibrate:
    def __init__(self, sync):
        self._load_modules()
        self.sync = sync
        self.gcode = sync.gcode
        self.path = os.path.expanduser(PLOT_PATH)
        self.check_export_path()

    @staticmethod
    def _load_modules():
        globals().update({
            'wrap': __import__('textwrap', fromlist=['wrap']).wrap,
            'multiprocessing': __import__('multiprocessing'),
            'plt': __import__('matplotlib.pyplot', fromlist=['']),
            'ticker': __import__('matplotlib.ticker', fromlist=['']),
            'curve_fit': __import__(
                'scipy.optimize', fromlist=['curve_fit']).curve_fit
        })

    def check_export_path(self):
        if os.path.exists(self.path):
            return
        try:
            os.makedirs(self.path)
        except OSError as e:
            raise self.gcode.error(
                f'Error generate path {self.path}: {e}')

    def linear_model(x, a, b):
        return a*x + b

    def quadratic_model(x, a, b, c):
        return a*x**2 + b*x + c

    def power_model(x, a, b):
        return a * np.power(x, b)

    def root_model(x, a, b):
        return a * np.sqrt(x) + b

    def hyperbolic_model(x, a, b):
        return a / x + b

    def exponential_model(x, a, b, c):
        return a * np.exp(b * x) + c

    models = {
        'Linear': linear_model,
        'Quadratic': quadratic_model,
        'Power': power_model,
        'Root': root_model,
        'Hyperbolic': hyperbolic_model,
        'Exponential': exponential_model
    }

    linestyles = {
        'Linear': '-.',
        'Quadratic': '--',
        'Power': ':',
        'Root': '--',
        'Hyperbolic': '-.',
        'Exponential': ':'
    }

    colors = {
        'Linear': '#DF8816',  # Dark Orange
        'Quadratic': 'green',
        'Power': 'cyan',
        'Root': 'magenta',
        'Hyperbolic': 'purple',
        'Exponential': 'blue'
    }

    def find_best_func(self, x_data, y_data, accel_chip='', msteps=16):
        maxfev = 999999999
        params = {}
        y_pred = {}
        rmse = {}
        for name, model in self.models.items():
            params[name], _ = curve_fit(model, x_data, y_data, maxfev=maxfev)
            y_pred[name] = model(x_data, *params[name])
            rmse[name] = np.sqrt(np.mean((y_data - y_pred[name]) ** 2))
        out = {}
        for name, _ in self.models.items():
            params_str = ','.join([f'{params[name][i]:.10f}'
                                   for i in range(len(params[name]))])
            out[name] = {'val': rmse[name], 'params': params[name],
                         'equation': params_str}
        sorted_out = sorted(out.keys(), key=lambda x: out[x]['val'])
        string_cmd = ['Functions RMSE and coefficients']
        for num, name in enumerate(sorted_out):
            string_cmd.append(f'{name}: RMSE {out[name]["val"]:.0f}'
                              f' coeffs: {out[name]["equation"]}')
        msg = self.plotter(out, sorted_out, x_data, y_data, accel_chip, msteps)
        string_cmd.insert(0, msg)
        return string_cmd

    def plotter(self, out, sorted_out, x_data,
                y_data, accel_chip, msteps, rmse_lim=20000):
        # Plotting
        fig, ax = plt.subplots()
        ax.scatter(x_data, y_data, label='Samples',
                   color='red', zorder=2, s=10)
        x_fit = np.linspace(min(x_data), max(x_data), 200)
        for num, name in enumerate(sorted_out):
            if out[name]['val'] < rmse_lim:
                string_graph = f"{name} RMSE: {out[name]['val']:.0f}"
                linestyle = self.linestyles[name]
                linewidth = 1
                color = self.colors[name]
                ax.plot(x_fit, self.models[name](x_fit, *out[name]['params']),
                        label=string_graph, linestyle=linestyle,
                        linewidth=linewidth, color=color)
        ax.legend(loc='lower right', fontsize=6, framealpha=1, ncol=1)
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        lognames = [now, '_' + accel_chip]
        title = (f"Dependency of desynchronization"
                 f" and functions ({''.join(lognames)})")
        ax.set_title('\n'.join(wrap(title, 66)), fontsize=10)
        ax.set_xlabel(f'Microsteps: 1/{msteps}')
        ax.set_xticks(np.arange(0, max(x_data) + 2.5, 2.5))
        ax.xaxis.set_minor_locator(ticker.MultipleLocator(2.5))
        ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.set_ylabel('Magnitude')
        ax.ticklabel_format(axis='y', style='scientific', scilimits=(0, 0))
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(which='major', color='grey')
        ax.grid(which='minor', color='lightgrey')
        png_path = os.path.join(
            self.path, f'interactive_plot_{accel_chip}_{now}.png')
        plt.savefig(png_path, dpi=1000)
        return f'Access to interactive plot at: {png_path}'

    def run_calibrate(self, gcmd):
        # "m" is a main axis, just single axis
        repeats = gcmd.get_int('REPEATS', 10, minval=2, maxval=100)
        axis = gcmd.get('AXIS', next(iter(self.sync.motion))).lower()
        m = self.sync.motion[axis]
        st_section = self.sync.config.getsection(m.steppers[0][0])
        rd = st_section.getfloat('rotation_distance')
        peak_point = gcmd.get_int('PEAK_POINT', rd * 1250, minval=10000)
        self.gcode.respond_info(
            f'Calibration started on {axis} axis with {repeats} '
            f'repeats, magnitude 0 --> {peak_point}', True)
        self.gcode.respond_info('Synchronizing before calibration...', True)
        self.sync.cmd_SYNC_MOTORS(gcmd, True)
        loop_pos = itertools.cycle(
            [m.limits[2] - rd, m.limits[2], m.limits[2] + rd])
        max_steps = 0
        invs = [1, -1]
        y_samples = np.array([])
        self.sync._handle_state(m, 'start')
        # Set calibrate step
        m.move_msteps = 1
        for i in range(1, repeats + 1):
            # Restore previous true magnitude after invs[-1]
            m.new_magnitude = m.magnitude
            self.gcode.respond_info(
                f"Repeats: {i}/{repeats} Try rise to {peak_point:.2f}"
                f" and lower to ~0 magnitude", True)
            self.sync._send(f'G0 {axis}{next(loop_pos)} '
                            f'F{self.sync.travel_speed * 60}')
            do_init = True
            for inv in invs:
                m.move_dir[0] = inv
                while True:
                    if ((inv == 1 and m.new_magnitude > m.magnitude
                                  and m.new_magnitude < peak_point)
                    or (inv == -1 and (m.new_magnitude < m.magnitude
                                   or m.new_magnitude > peak_point))
                    or do_init):
                        if not (do_init and inv == 1):
                            if m.new_magnitude > (max(y_samples)
                             if y_samples.size > 0 else 0):
                                max_steps += m.move_msteps
                            y_samples = np.append(y_samples, m.new_magnitude)
                        m.magnitude = m.new_magnitude
                        self.sync._single_move(m)
                        m.new_magnitude = self.sync._measure(m)
                        self.sync._handle_state(m, 'stepped')
                        do_init = False
                    else:
                        break
            # Move on previous microstep
            m.move_dir[0] = 1
            self.sync._single_move(m)
        # Move on initial mstep
        m.move_msteps = -m.actual_msteps
        self.sync._single_move(m)
        # Finish actions
        m.fan_switch(True)
        m.aclient.finish_measurements()
        y_samples = np.sort(y_samples)
        x_samples = np.linspace(0.01, max_steps, len(y_samples))
        x_samples_str = ', '.join([str(i) for i in y_samples])
        y_samples_str = ', '.join([f'{i:.2f}' for i in x_samples])
        logging.info(f"motors_sync: y_samples: [{x_samples_str}]")
        logging.info(f"motors_sync: x_samples: [{y_samples_str}]")

        def samples_processing():
            try:
                os.nice(10)
            except:
                pass
            msg = self.find_best_func(x_samples, y_samples,
                                      m.chip_name, m.microsteps)
            for line in msg:
                self.gcode.respond_info(str(line), True)

        # Run plotter
        proces = multiprocessing.Process(target=samples_processing)
        proces.daemon = False
        proces.start()


class KalmanLiteFilter:
    def __init__(self, A, H, Q, R, P0, x0):
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.P = self.st_p = P0
        self.x = self.st_x = x0
        self.I = 1

    def predict(self):
        self.x = self.A * self.x
        self.P = self.A * self.P * self.A + self.Q

    def update(self, z):
        self.predict()
        y = z - (self.H * self.x)
        S = self.H * self.P * self.H + self.R
        K = self.P * self.H * S
        self.x += K * y
        self.P = (self.I - K * self.H) * self.P
        return self.x

    def process_samples(self, samples):
        self.x = self.st_x
        self.P = self.st_p
        return np.array(
            [self.update(z) for z in samples]).reshape(-1)


class StatisticsManager:
    def __init__(self, gcode, cmd_name, log_name, log_parser, format):
        self._load_modules()
        self.gcode = gcode
        self.cmd_name = cmd_name.upper()
        self.log_parser = log_parser
        self.format = format
        # Register commands
        self.gcode.register_command(self.cmd_name, self.cmd_GET_STATS,
                                    desc=self.cmd_GET_STATS_help)
        # Variables
        self.home_dir = os.path.dirname(os.path.realpath(__file__))
        self.log_path = os.path.join(self.home_dir, log_name)
        self.error = ''
        # Checks
        self.check_log()

    @staticmethod
    def _load_modules():
        for module in ['csv', 'ast']:
            globals()[module] = __import__(module)

    def check_log(self):
        if os.path.exists(self.log_path):
            header = ','.join(self.read_log(True))
            if header != self.format:
                self.error = (f'Invalid format, type {self.cmd_name}'
                                f' CLEAR=1 to reset and fix statistics')
        else:
            try:
                self.write_log(self.format.split(','))
            except Exception as e:
                self.error = str(e)

    def read_log(self, only_header=False):
        with open(self.log_path, mode='r', newline='') as f:
            reader = csv.reader(f, delimiter=',')
            header = next(reader)
            if only_header:
                return header
            log = list(reader)
        return np.array(log)

    def write_log(self, line):
        with open(self.log_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(line)

    def clear_log(self):
        os.remove(self.log_path)
        self.check_log()
        self.error = ''

    def parse_raw_log(self, log):
        converted = []
        for line in log:
            converted_line = []
            for element in line:
                element = str(element).strip("'\"")
                try:
                    converted_el = ast.literal_eval(element)
                except (ValueError, SyntaxError):
                    converted_el = element
                if isinstance(converted_el, tuple):
                    converted_el = np.array(converted_el)
                converted_line.append(converted_el)
            converted.append(converted_line)
        return np.array(converted, dtype=object)

    cmd_GET_STATS_help = 'Show statistics'
    def cmd_GET_STATS(self, gcmd):
        do_clear = gcmd.get('CLEAR', '').lower()
        if do_clear in ['true', '1']:
            self.clear_log()
            self.gcode.respond_info('Logs was cleared')
            return
        if self.error:
            self.gcode.respond_info(f'Statistics collection is '
                                    f'disabled due:\n{self.error}')
            return
        raw_log = self.read_log()
        if raw_log.size == 0:
            self.gcode.respond_info('Logs are empty')
            return
        log = self.parse_raw_log(raw_log)
        msg = self.log_parser(log)
        for line in msg:
            self.gcode.respond_info(str(line))


def load_config(config):
    return MotorsSync(config)
