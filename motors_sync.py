# Motors synchronization script
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, time, itertools
from datetime import datetime
import numpy as np
from . import z_tilt

PIN_MIN_TIME = 0.010
DISABLE_STALL_TIME = 0.100
MEASURE_DELAY = 0.05            # Delay between damped oscillations and measurement
ACCEL_FILTER_THRESHOLD = 3000   # Accelerometer filter disabled at lower sampling rate
AXES_LEVEL_DELTA = 2000         # Magnitude difference between axes
LEVELING_KINEMATICS = (         # Kinematics with interconnected axes
    ['corexy', 'limited_corexy'])

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
        self._init_chips()
        self._init_sync_method()
        self._init_models()
        self._init_fan()
        self._init_other_conf_vars()
        self.debug = self.config.getboolean('debug', default=False)
        # Register commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_SYNC_MOTORS, desc=self.cmd_SYNC_MOTORS_help)
        self.gcode.register_command('SYNC_MOTORS_CALIBRATE', self.cmd_SYNC_MOTORS_CALIBRATE,
                                    desc=self.cmd_SYNC_MOTORS_CALIBRATE_help)
        # Variables
        self._init_stat_manager()

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = min(self.toolhead.max_accel, 5000)
        self.kin = self.toolhead.get_kinematics()
        self._init_steppers()
        self._init_fan(True)

    def lookup_config(self, section, section_entries, force_back=None):
        # Loockup in printer config
        out = []
        section = self.config.getsection(section)
        for value in section_entries:
            try:
                out.append(float(section.get(value, force_back)))
            except:
                out.append(section.get(value, force_back))
        if None in out:
            raise
        return out[0] if len(out) == 1 else out

    def _init_axes(self):
        valid_axes = ['X', 'Y']
        self.conf_kin = self.lookup_config('printer', ['kinematics'])
        if self.conf_kin in LEVELING_KINEMATICS:
            self.do_level = True
            axes = [a.upper() for a in self.config.getlist('axes', count=2, default=['x', 'y'])]
        elif self.conf_kin == 'cartesian':
            self.do_level = False
            axes = [a.upper() for a in self.config.getlist('axes')]
        else:
            raise self.config.error(f"motors_sync: Not supported kinematics '{self.conf_kin}'")
        if any(axis not in valid_axes for axis in axes):
            raise self.config.error(f"motors_sync: Invalid axes parameter '{','.join(axes)}'")
        self.motion = {axis: {} for axis in axes}
        for axis in axes:
            min_pos, max_pos = self.lookup_config(
                'stepper_' + axis.lower(), ['position_min', 'position_max'], 0)
            self.motion[axis].update({
                'limits': [min_pos + 10, max_pos - 10, (min_pos + max_pos) / 2]
            })

    def _init_steppers(self):
        for axis in self.motion:
            lo_axis = axis.lower()
            stepper = 'stepper_' + lo_axis
            rd, fspr = self.lookup_config(
                stepper, ['rotation_distance', 'full_steps_per_rotation'], 200)
            move_len = rd / fspr / self.motion[axis]['microsteps']
            steppers, lookuped = zip(*[(l.get_name(), l) for l in
                self.kin.get_steppers() if lo_axis in l.get_name()])
            if len(steppers) not in (2,):
                raise self.config.error(f'motors_sync: Not supported'
                                        f"'{len(steppers)}' count of motors")
            for _axis in self.motion:
                cf_msteps = self.motion[_axis]['microsteps']
                for stepper in steppers:
                    st_msteps = self.lookup_config(stepper, ['microsteps'], None)
                    if cf_msteps > st_msteps:
                        raise self.config.error(
                            f'motors_sync: Invalid microsteps count, cannot be'
                            f' more than steppers, {cf_msteps} vs {st_msteps}')
            self.motion[axis].update({
                'do_buzz': True,
                'steppers': steppers,
                'lookuped_steppers': lookuped,
                'move_len': move_len
            })

    def _init_chip_filter(self, axis):
        filters = ['def', 'median', 'kalman']
        fd = {m: m for m in filters}
        filter = self.config.getchoice(f'chip_filter_{axis.lower()}', fd, 'def').lower()
        if filter == 'def':
            filter = self.config.getchoice('chip_filter', fd, 'median').lower()
        if filter == 'median':
            window = self.config.getint(f'median_size_{axis.lower()}', '', minval=3, maxval=9)
            if not window:
                window = self.config.getint('median_size', default=3, minval=3, maxval=9)
            if window % 2 == 0:
                raise self.config.error(
                    f"motors_sync: parameter 'median_size' cannot be even")
            self.motion[axis]['chip_filter'] = MedianFilter(window).process_samples
        elif filter == 'kalman':
            coeffs = self.config.getfloatlist(
                f'kalman_coeffs_{axis.lower()}', tuple('' for _ in range(6)), count=6)
            if not all(coeffs):
                coeffs = self.config.getfloatlist(
                    'kalman_coeffs', tuple((1.1, 1., 1e-1, 1e-2, .5, 1.)), count=6)
            self.motion[axis]['chip_filter'] = KalmanLiteFilter(*coeffs).process_samples

    def _init_chip_config(self, axis, chip):
        chip_config = self.printer.lookup_object(chip)
        self.motion[axis]['chip'] = chip
        self.motion[axis]['chip_config'] = chip_config
        if hasattr(chip_config, 'data_rate'):
            if chip_config.data_rate > ACCEL_FILTER_THRESHOLD:
                self._init_chip_filter(axis)
            else:
                self.motion[axis]['chip_filter'] = lambda data: data
        elif chip == 'beacon':
            # Beacon sampling rate > ACCEL_FILTER_THRESHOLD
            self._init_chip_filter(axis)
        else:
            raise self.config.error(
                f"motors_sync: Unknown accelerometer '{chip}' sampling rate")

    def _init_chips(self):
        chips = {}
        for axis in self.motion:
            chips[axis] = self.config.get(f'accel_chip_{axis.lower()}', '')
            if not chips[axis]:
                chips[axis] = self.config.get('accel_chip')
        if self.conf_kin in LEVELING_KINEMATICS and len(set(chips.values())) > 1:
            raise self.config.error(f"motors_sync: Accel chips cannot be different"
                                    f" for a '{self.conf_kin}' kinematics")
        for axis, chip in chips.items():
            self._init_chip_config(axis, chip)

    def _init_sync_method(self):
        methods = ['sequential', 'alternately', 'synchronous', 'default']
        self.sync_method = self.config.getchoice(
            'sync_method', {m: m for m in methods}, 'default')
        if self.sync_method == 'default':
            if self.conf_kin in LEVELING_KINEMATICS:
                self.sync_method = methods[1]
            else:
                self.sync_method = methods[0]
        if self.sync_method in methods[1:] and self.conf_kin not in LEVELING_KINEMATICS:
            raise self.config.error(f"motors_sync: Invalid sync method: "
                                    f"{self.sync_method} for '{self.conf_kin}'")

    def _init_models(self):
        final_models = {}
        models = {
            'linear': {'args': {'count': 2, 'a': None}, 'func': self.polynomial_model},
            'quadratic': {'args': {'count': 3, 'a': None}, 'func': self.polynomial_model},
            'power': {'args': {'count': 2, 'a': None}, 'func': self.power_model},
            'root': {'args': {'count': 2, 'a': 0}, 'func': self.root_model},
            'hyperbolic': {'args': {'count': 2, 'a': 0}, 'func': self.hyperbolic_model},
            'exponential': {'args': {'count': 3, 'a': 0}, 'func': self.exponential_model}
        }
        for axis in self.motion:
            model = self.config.get(f'model_{axis.lower()}', '').lower()
            if not model:
                model = self.config.get('model', 'linear').lower()
            coeffs_vals = self.config.getfloatlist(f'model_coeffs_{axis.lower()}', '')
            if not coeffs_vals:
                coeffs_vals = self.config.getfloatlist('model_coeffs', default=[20000, 0])
            coeffs_args = [chr(97 + i) for i in range(len(coeffs_vals) + 1)]
            model_coeffs = {arg: float(val) for arg, val in zip(coeffs_args, coeffs_vals)}
            if model not in models:
                raise self.config.error(f"motors_sync: Invalid model '{model}'")
            if len(model_coeffs) != models[model]['args']['count']:
                raise self.config.error(f"motors_sync: {model.capitalize()} model requires"
                                        f" {models[model]['args']['count']} coefficients")
            if models[model]['args']['a'] == model_coeffs['a']:
                raise self.config.error(f"motors_sync: Coefficient 'a' cannot be "
                                        f"{model_coeffs['a']} for a '{model}' model")
            final_models[axis] = [models[model]['func'], list(model_coeffs.values())]
        if self.conf_kin in LEVELING_KINEMATICS and not all(
                v == final_models[axis] for v in final_models.values()):
            raise self.config.error(f"motors_sync: Models and coefficients cannot be"
                                    f" different for a '{self.conf_kin}' kinematics")
        for axis, (model, coeffs) in final_models.items():
            self.motion[axis].update({
                'solve_model': model,
                'model_coeffs': coeffs
            })

    @staticmethod
    def polynomial_model(coeffs, fx):
        sol = np.roots([*coeffs[:-1], coeffs[-1] - fx])
        return max(sol.real)

    @staticmethod
    def power_model(coeffs, fx):
        a, b = coeffs
        return (fx / a) ** (1 / b)

    @staticmethod
    def root_model(coeffs, fx):
        a, b = coeffs
        return (fx**2 - 2*b*fx + b**2) / a**2

    @staticmethod
    def hyperbolic_model(coeffs, fx):
        a, b = coeffs
        return a / (fx - b)

    @staticmethod
    def exponential_model(coeffs, fx):
        a, b, c = coeffs
        return np.log((fx - c) / a) / b

    def _create_fan_switch(self, method):
        if method == 'heater_fan':
            def fan_switch(on=True):
                if not self.fan:
                    return
                now = self.printer.get_reactor().monotonic()
                print_time = self.fan.fan.get_mcu().estimated_print_time(now)
                speed = self.fan.last_speed if on else .0
                self.fan.fan.set_speed(value=speed, print_time=print_time + PIN_MIN_TIME)
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
        self._fan_switch = fan_switch

    def _init_fan(self, connect=False):
        fan_methods = ['heater_fan', 'temperature_fan']
        self.fan = None
        self.conf_fan = self.config.get('head_fan', default=None)
        # Wait klippy connect
        if not connect:
            return
        # Create a stub
        if not self.conf_fan:
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
        raise self.config.error(
            f"motors_sync: Unknown fan or fan method '{self.conf_fan}'")

    def _init_other_conf_vars(self):
        vb = { # Variables dict
            'microsteps': {'def': 16, 'min': 4, 'max': 256},
            'max_step_size': {'def': 3, 'min': 1, 'max': None},
            'axes_steps_diff': {'def': None, 'min': 1, 'max': 999999},
            'retry_tolerance': {'def': 0, 'min': 0, 'max': 999999},
            'retries': {'def': 0, 'min': 0, 'max': 10},
        }
        gd = {} # Guard dict
        for axis in self.motion:
            gd[axis] = {}
            for var, p in vb.items():
                param = self.config.getint(f'{var}_{axis.lower()}',
                         default=None, minval=p['min'], maxval=p['max'])
                if param is None:
                    param = self.config.getint(f'{var}',
                             default=p['def'], minval=p['min'], maxval=p['max'])
                gd[axis][var] = param
                # Init 'max_step_size': 'max' value = microsteps / 2
                if var == 'microsteps':
                    vb['max_step_size']['max'] = gd[axis]['microsteps'] / 2
                # Init 'axes_steps_diff': 'def' value = max_step_size + 1
                elif var == 'max_step_size':
                    vb['axes_steps_diff']['def'] = gd[axis]['max_step_size'] + 1
        common_items = set.intersection(*(set(d.items()) for d in gd.values()))
        exclude = [dict(set(d.items()) - common_items) for d in gd.values()
                   if set(d.items()) - common_items]
        if self.conf_kin in LEVELING_KINEMATICS and len(exclude) > 1:
            raise self.config.error(f"motors_sync: {', '.join(map(str, exclude))} cannot be "
                                    f"different for a '{self.conf_kin}' kinematics")
        for axis, params in gd.items():
            self.motion[axis].update(params)

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
                cf_microsteps = self.motion[axis.upper()].get('microsteps', 16)
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
            for axis in ([axis] if axis else self.axes):
                if not self.motion[axis]['actual_msteps']:
                    continue
                msteps = self.motion[axis]['microsteps']
                magnitudes, pos = zip(*self.motion[axis]['log'])
                retries = self.motion[axis]['curr_retry']
                date = datetime.now().strftime('%Y-%m-%d')
                manager.write_log([axis, status, magnitudes,
                                   pos, msteps, retries, date])
        self.write_log = write_log

    def _send(self, params):
        self.gcode._process_commands([params], False)

    def _stepper_switch(self, stepper, mode):
        self.toolhead.dwell(DISABLE_STALL_TIME)
        print_time = self.toolhead.get_last_move_time()
        el = self.stepper_en.enable_lines[stepper]
        if mode:
            el.motor_enable(print_time)
        else:
            el.motor_disable(print_time)
        self.toolhead.dwell(DISABLE_STALL_TIME)

    def _stepper_move(self, stepper, dist):
        self.force_move.manual_move(stepper, dist, 100, self.travel_accel)

    def _buzz(self, axis):
        # Fading oscillations
        move_len = self.motion[axis]['move_len']
        lookup_sec_stepper = self.motion[axis]['lookuped_steppers'][1]
        self._stepper_switch(self.motion[axis]['steppers'][0], 0)
        for i in reversed(range(0, int(0.4 / move_len))):
            dist = move_len * 4 * i
            self._stepper_move(lookup_sec_stepper, dist)
            self._stepper_move(lookup_sec_stepper, -dist)

    def _calc_magnitude(self, aclient, axis):
        # Calculate impact magnitude
        if self.debug: start_time = time.perf_counter()
        vect = np.array([[sample.accel_x, sample.accel_y, sample.accel_z]
                         for sample in aclient.get_samples()])
        vect_len = vect.shape[0]
        cut = vect[:vect_len // 10, :]
        z_axis = np.mean(np.abs(cut), axis=0).argmax()
        xy_vect = np.delete(vect, z_axis, axis=1)
        magnitudes = np.linalg.norm(xy_vect, axis=1)
        # Add median, Kalman or none filter
        magnitudes = self.motion[axis]['chip_filter'](magnitudes)
        # Calculate static noise
        static = np.mean(magnitudes[vect_len // 4:vect_len // 2])
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitudes)[-5:])
        if self.debug: self.gcode.respond_info(
            f'Static: {static:.2f}, total time: '
            f'{time.perf_counter() - start_time:.6f}', True)
        magnitude = np.around(magnitude - static, 2)
        self.motion[axis]['log'].append(
            [int(magnitude), self.motion[axis]['actual_msteps']])
        return magnitude

    def _measure(self, axis):
        # Measure the impact
        stepper = self.motion[axis]['steppers'][0]
        if self.motion[axis]['do_buzz']:
            self._buzz(axis)
        self._stepper_switch(stepper, 1)
        self._stepper_switch(stepper, 0)
        aclient = self.motion[axis]['chip_config'].start_internal_client()
        self.toolhead.dwell(MEASURE_DELAY)
        self._stepper_switch(stepper, 1)
        self.toolhead.dwell(MEASURE_DELAY)
        aclient.finish_measurements()
        return self._calc_magnitude(aclient, axis)

    def _homing(self):
        # Homing and going to center
        now = self.printer.get_reactor().monotonic()
        center = {}
        axes = list(self.motion.keys())
        for axis in axes:
            center[axis] = self.motion[axis]['limits'][2]
        if ''.join(axes).lower() not in self.kin.get_status(now)['homed_axes']:
            self._send(f"G28 {' '.join(axes)}")
        self._send(f"G0 {' '.join(f'{axis}{pos}' for axis, pos in center.items())}"
                   f" F{self.travel_speed * 60}")
        self.toolhead.wait_moves()

    def _detect_move_dir(self, axis):
        # Determine movement direction
        m = self.motion[axis]
        self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'])
        m['actual_msteps'] += m['move_msteps']
        m['check_msteps'] += m['move_msteps']
        m['new_magnitude'] = self._measure(axis)
        self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']} "
                                f"on {m['move_msteps']}/{m['microsteps']} step move", True)
        m['move_dir'] = [-1, 'Backward'] if m['new_magnitude'] > m['magnitude'] else [1, 'Forward']
        self.gcode.respond_info(f"{axis}-Movement direction: {m['move_dir'][1]}", True)
        m['magnitude'] = m['new_magnitude']

    def _run_sync(self):
        # Axes synchronization
        def axes_level(min_ax, max_ax):
            # Axes leveling by magnitude
            m = self.motion[max_ax]
            s = self.motion[min_ax]
            delta = m['init_magnitude'] - s['init_magnitude']
            if delta <= AXES_LEVEL_DELTA:
                return
            self.gcode.respond_info(f'Start axes level, delta: {delta:.2f}', True)
            force_exit = False
            while True:
                # m['axes_steps_diff'] == s['axes_steps_diff']
                if abs(abs(m['check_msteps']) - abs(s['check_msteps'])) >= m['axes_steps_diff']:
                    s['new_magnitude'] = self._measure(min_ax)
                    self.gcode.respond_info(f"{min_ax}-New magnitude: {s['new_magnitude']}", True)
                    s['magnitude'] = s['new_magnitude']
                    m['check_msteps'], s['check_msteps'] = 0, 0
                if not m['move_dir'][1]:
                    self._detect_move_dir(max_ax)
                steps_delta = int(m['solve_model'](m['model_coeffs'], m['magnitude']) -
                                  m['solve_model'](m['model_coeffs'], s['magnitude']))
                m['move_msteps'] = min(max(steps_delta, 1), m['max_step_size'])
                self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'] * m['move_dir'][0])
                move_msteps = m['move_msteps'] * m['move_dir'][0]
                m['actual_msteps'] += move_msteps
                m['check_msteps'] += move_msteps
                m['new_magnitude'] = self._measure(max_ax)
                self.gcode.respond_info(f"{max_ax}-New magnitude: {m['new_magnitude']} on "
                                        f"{m['move_msteps'] * m['move_dir'][0]}/{m['microsteps']} step move", True)
                if m['new_magnitude'] > m['magnitude']:
                    self._stepper_move(m['lookuped_steppers'][0], m['move_msteps']
                                       * m['move_len'] * m['move_dir'][0] * -1)
                    m['actual_msteps'] -= m['move_msteps'] * m['move_dir'][0]
                    if m['retry_tolerance'] and m['magnitude'] > m['retry_tolerance']:
                        m['curr_retry'] += 1
                        if m['curr_retry'] > m['retries']:
                            self.gcode.respond_info(
                                f"{max_ax} Motors adjusted by {m['actual_msteps']}/{m['microsteps']}"
                                f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}", True)
                            self._fan_switch(True)
                            self.write_log(axis)
                            raise self.gcode.error('Too many retries')
                        self.gcode.respond_info(f"Retries: {m['curr_retry']}/{m['retries']}"
                                                f" Data in loop is incorrect! ", True)
                        m['move_dir'][1] = ''
                        continue
                    force_exit = True
                delta = m['new_magnitude'] - s['init_magnitude']
                if delta < AXES_LEVEL_DELTA or m['new_magnitude'] < s['init_magnitude'] or force_exit:
                    m['magnitude'] = m['new_magnitude']
                    self.gcode.respond_info(
                        f"Axes are leveled: {max_ax}: {m['init_magnitude']} --> {m['new_magnitude']}"
                        f", {min_ax}: {s['init_magnitude']}, delta: {delta:.2f}", True)
                    return
                m['magnitude'] = m['new_magnitude']
                continue

        def inner_sync(axis, check_axis=False):
            # If you have any ideas on how to simplify this trash, suggest (c)
            m = self.motion[axis]
            if check_axis:
                m['new_magnitude'] = self._measure(axis)
                self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']}", True)
                m['magnitude'] = m['new_magnitude']
                return
            if not m['move_dir'][1]:
                if not m['new_magnitude'] or m['curr_retry']:
                    m['new_magnitude'] = self._measure(axis)
                    m['magnitude'] = m['new_magnitude']
                    self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']}", True)
                if not m['actual_msteps'] and m['retry_tolerance'] and m['new_magnitude'] < m['retry_tolerance']:
                    m['out_msg'] = (f"{axis}-Motors adjusted by {m['actual_msteps']}/{m['microsteps']} "
                                    f"step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                    return
                self._detect_move_dir(axis)
                return
            m['move_msteps'] = min(max(int(m['solve_model'](m['model_coeffs'], m['magnitude'])), 1), m['max_step_size'])
            self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'] * m['move_dir'][0])
            move_msteps = m['move_msteps'] * m['move_dir'][0]
            m['actual_msteps'] += move_msteps
            m['check_msteps'] += move_msteps
            m['new_magnitude'] = self._measure(axis)
            self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']} on "
                                    f"{m['move_msteps'] * m['move_dir'][0]}/{m['microsteps']} step move", True)
            if m['new_magnitude'] > m['magnitude']:
                self._stepper_move(m['lookuped_steppers'][0],
                                   m['move_msteps'] * m['move_len'] * m['move_dir'][0] * -1)
                m['actual_msteps'] -= m['move_msteps'] * m['move_dir'][0]
                if m['retry_tolerance'] and m['magnitude'] > m['retry_tolerance']:
                    m['curr_retry'] += 1
                    if m['curr_retry'] > m['retries']:
                        self.gcode.respond_info(
                            f"{axis} Motors adjusted by {m['actual_msteps']}/{m['microsteps']}"
                            f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}", True)
                        self._fan_switch(True)
                        self.write_log(axis)
                        raise self.gcode.error('Too many retries')
                    self.gcode.respond_info(
                        f"{axis} Retries: {m['curr_retry']}/{m['retries']} Back on last magnitude: {m['magnitude']}"
                        f" on {m['actual_msteps']}/{m['microsteps']} step to reach {m['retry_tolerance']}", True)
                    m['move_dir'][1] = ''
                    return
                m['out_msg'] = (f"{axis}-Motors adjusted by {m['actual_msteps']}/{m['microsteps']}"
                                f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                return
            m['magnitude'] = m['new_magnitude']

        if self.sync_method == 'alternately' and len(self.axes) > 1:
            min_ax, max_ax = sorted(self.motion, key=lambda x: self.motion[x]['init_magnitude'])[:2]
            axes_level(min_ax, max_ax)
            axes = self.axes[::-1] if max_ax == self.axes[0] else self.axes
            check_axis = True
            for axis in itertools.cycle(axes):
                m = self.motion[axis]
                if m['out_msg']:
                    if all(bool(self.motion[axis]['out_msg']) for axis in self.axes):
                        break
                    continue
                inner_sync(axis, check_axis)
                check_axis = False
        elif self.sync_method == 'synchronous' and len(self.axes) > 1:
            check_axis = False
            cycling = itertools.cycle(self.axes)
            self._detect_move_dir(max(self.motion, key=lambda k: self.motion[k]['init_magnitude']))
            while True:
                axis = next(cycling)
                cycling, cycle = itertools.tee(cycling)
                m = self.motion[axis]
                sec = next(cycle)
                s = self.motion[sec]
                if m['out_msg']:
                    if all(bool(self.motion[axis]['out_msg']) for axis in self.axes):
                        break
                    continue
                if m['magnitude'] < s['magnitude'] and not s['out_msg']:
                    # m['axes_steps_diff'] == s['axes_steps_diff']
                    if abs(abs(m['check_msteps']) - abs(s['check_msteps'])) >= m['axes_steps_diff']:
                        check_axis = True
                        m['check_msteps'], s['check_msteps'] = 0, 0
                    else:
                        continue
                inner_sync(axis, check_axis)
                check_axis = False
        elif self.sync_method == 'sequential' or len(self.axes) == 1:
            for axis in self.axes:
                m = self.motion[axis]
                # To skip _measure() in inner_sync()
                self._detect_move_dir(axis)
                while True:
                    if m['out_msg']:
                        if all(bool(self.motion[axis]['out_msg']) for axis in self.axes):
                            return
                        break
                    inner_sync(axis)
        else:
            raise self.gcode.error('Error in sync methods!')

    cmd_SYNC_MOTORS_help = 'Start motors synchronization'
    def cmd_SYNC_MOTORS(self, gcmd, force_run=False):
        # Live variables
        axes_from_gcmd = gcmd.get('AXES', '')
        if axes_from_gcmd:
            axes_from_gcmd = axes_from_gcmd.split(',')
            if any([axis.upper() not in self.motion.keys() for axis in axes_from_gcmd]):
                raise self.gcode.error(f'Invalid axes parameter')
            self.axes = [axis.upper() for axis in axes_from_gcmd]
        else:
            self.axes = list(self.motion.keys())
        chip = gcmd.get(f'ACCEL_CHIP', None)
        for axis in self.axes:
            a_chip = gcmd.get(f'ACCEL_CHIP_{axis}', chip)
            if a_chip and a_chip != self.motion[axis]['chip']:
                try:
                    self.printer.lookup_object(a_chip)
                except Exception as e:
                    raise self.gcode.error(e)
                self._init_chip_config(axis, a_chip)
        for param in ['retry_tolerance', 'retries']:
            exclude = set()
            for axis in self.motion:
                new_param = gcmd.get_int(f'{param.upper()}_{axis}', 0, minval=0, maxval=999999)
                if new_param:
                    self.motion[axis][param] = new_param
                    exclude.add(axis)
            new_param = gcmd.get_int(f'{param.upper()}', 0, minval=0, maxval=999999)
            if new_param:
                for axis in self.motion:
                    if axis not in exclude:
                        self.motion[axis][param] = new_param
        # Run
        self.status.reset()
        self._homing()
        self._fan_switch(False)
        self.gcode.respond_info('Motors synchronization started', True)
        # Init axes
        for axis in self.axes:
            # Need to be init or reset before _measure()
            self.motion[axis].update({
                'actual_msteps': 0,
                'log': [],
                })
            init_magnitude = self._measure(axis)
            self.motion[axis].update({
                'move_dir': [1, ''],
                'move_msteps': 2,
                'check_msteps': 0,
                'init_magnitude': init_magnitude,
                'magnitude': init_magnitude,
                'new_magnitude': 0,
                'curr_retry': 0,
                'out_msg': '',
            })
            self.gcode.respond_info(f"{axis}-Initial magnitude: {init_magnitude}", True)
        if not force_run and all(
                self.motion[axis]['init_magnitude'] < self.motion[axis]['retry_tolerance']
                for axis in self.axes):
            self.gcode.respond_info(
                "Motors magnitudes are in tolerance: " +
                ", ".join(f"{a}: {p['retry_tolerance']}" for a, p in self.motion.items()), True)
        else:
            self._run_sync()
            # Info
            for axis in self.axes:
                self.gcode.respond_info(self.motion[axis]['out_msg'], True)
        self._fan_switch(True)
        self.status.check_retry_result('done')
        self.write_log()

    cmd_SYNC_MOTORS_CALIBRATE_help = 'Calibrate synchronization process'
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
            now = self.printer.get_reactor().monotonic()
            return bool(list(self.status.get_status(now).values())[0])


class MotorsSyncCalibrate:
    def __init__(self, sync):
        self._load_modules()
        self.sync = sync
        self.gcode = sync.gcode
        self.path = os.path.expanduser('~/printer_data/config/adxl_results/motors_sync')
        self.check_export_path()

    @staticmethod
    def _load_modules():
        globals().update({
            'wrap': __import__('textwrap', fromlist=['wrap']).wrap,
            'multiprocessing': __import__('multiprocessing'),
            'plt': __import__('matplotlib.pyplot', fromlist=['']),
            'ticker': __import__('matplotlib.ticker', fromlist=['']),
            'curve_fit': __import__('scipy.optimize', fromlist=['curve_fit']).curve_fit
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
            params_str = ','.join([f'{params[name][i]:.10f}' for i in range(len(params[name]))])
            out[name] = {'val': rmse[name], 'params': params[name], 'equation': params_str}
        sorted_out = sorted(out.keys(), key=lambda x: out[x]['val'])
        string_cmd = ['Functions RMSE and coefficients']
        for num, name in enumerate(sorted_out):
            string_cmd.append(f'{name}: RMSE {out[name]["val"]:.0f} coeffs: {out[name]["equation"]}')
        msg = self.plotter(out, sorted_out, x_data, y_data, accel_chip, msteps)
        string_cmd.insert(0, msg)
        return string_cmd

    def plotter(self, out, sorted_out, x_data, y_data, accel_chip, msteps, rmse_lim=20000):
        # Plotting
        fig, ax = plt.subplots()
        ax.scatter(x_data, y_data, label='Samples', color='red', zorder=2, s=10)
        x_fit = np.linspace(min(x_data), max(x_data), 200)
        for num, name in enumerate(sorted_out):
            if out[name]['val'] < rmse_lim:
                string_graph = f"{name} RMSE: {out[name]['val']:.0f}"
                linestyle = self.linestyles[name]
                linewidth = 1
                color = self.colors[name]
                ax.plot(x_fit, self.models[name](x_fit, *out[name]['params']),
                        label=string_graph, linestyle=linestyle, linewidth=linewidth, color=color)
        ax.legend(loc='lower right', fontsize=6, framealpha=1, ncol=1)
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        lognames = [now, '_' + accel_chip]
        title = f"Dependency of desynchronization and functions ({''.join(lognames)})"
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
        png_path = os.path.join(self.path, f'interactive_plot_{accel_chip}_{now}.png')
        plt.savefig(png_path, dpi=1000)
        return f'Access to interactive plot at: {png_path}'

    def run_calibrate(self, gcmd):
        repeats = gcmd.get_int('REPEATS', 10, minval=2, maxval=100)
        axis = gcmd.get('AXIS', next(iter(self.sync.motion))).upper()
        m = self.sync.motion[axis]
        rd = self.sync.lookup_config(m['steppers'][0], ['rotation_distance'], 0)
        peak_point = gcmd.get_int('PEAK_POINT', rd * 1250, minval=10000, maxval=999999)
        self.gcode.respond_info(f'Calibration started on {axis} axis with '
                                f'{repeats} repeats, magnitude 0 --> {peak_point}', True)
        self.gcode.respond_info('Synchronizing before calibration...', True)
        self.sync.cmd_SYNC_MOTORS(gcmd, True)
        loop_pos = itertools.cycle([m['limits'][2] - rd, m['limits'][2], m['limits'][2] + rd])
        max_steps = 0
        invs = [1, -1]
        y_samples = np.array([])
        m['move_msteps'] = 1
        m['backup_msteps'] = m['actual_msteps']
        for i in range(1, repeats + 1):
            # Restore previous true magnitude after _final_sync() and after invs[-1]
            m['new_magnitude'] = m['magnitude']
            self.gcode.respond_info(
                f"Repeats: {i}/{repeats} Try rise to {peak_point:.2f} and lower to ~0 magnitude", True)
            self.sync._send(f'G0 {axis}{next(loop_pos)} F{self.sync.travel_speed * 60}')
            do_init = True
            for inv in invs:
                while True:
                    if ((inv == 1 and m['new_magnitude'] > m['magnitude'] and m['new_magnitude'] < peak_point) or
                         (inv == -1 and (m['new_magnitude'] < m['magnitude'] or m['new_magnitude'] > peak_point)) or
                          do_init):
                        if not (do_init and inv == 1):
                            if m['new_magnitude'] > (max(y_samples) if y_samples.size > 0 else 0):
                                max_steps += m['move_msteps']
                            y_samples = np.append(y_samples, m['new_magnitude'])
                        m['magnitude'] = m['new_magnitude']
                        self.sync._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'] * inv)
                        m['actual_msteps'] += m['move_msteps'] * inv
                        m['new_magnitude'] = self.sync._measure(axis)
                        self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']} on "
                                                f"{m['move_msteps'] * inv}/{m['microsteps']} step move", True)
                        do_init = False
                    else:
                        break
            # Move on previous microstep
            self.sync._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'])
            m['actual_msteps'] += m['move_msteps']
        # Move on initial mstep
        move_msteps = m['backup_msteps'] - m['actual_msteps']
        self.sync._stepper_move(m['lookuped_steppers'][0], move_msteps * m['move_len'])
        m['actual_msteps'] = m.pop('backup_msteps')
        y_samples = np.sort(y_samples)
        x_samples = np.linspace(0.01, max_steps, len(y_samples))
        if self.sync.debug:
            self.gcode.respond_info(f"Y Samples: {', '.join([str(i) for i in y_samples])}", True)
            self.gcode.respond_info(f"X samples: {', '.join([f'{i:.2f}' for i in x_samples])}", True)

        def samples_processing():
            try:
                os.nice(10)
            except:
                pass
            msg = self.find_best_func(x_samples, y_samples, m['chip'], m['microsteps'])
            for line in msg:
                self.gcode.respond_info(str(line), True)

        # Run plotter
        proces = multiprocessing.Process(target=samples_processing)
        proces.daemon = False
        proces.start()


class MedianFilter:
    def __init__(self, window_size):
        self.w = window_size // 2

    def process_samples(self, samples):
        return np.median(
            [samples[i - self.w:i + self.w + 1]
             for i in range(self.w, len(samples) - self.w)], axis=1)


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
