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
LEVELING_KINEMATICS = (         # Kinematics with interconnected axes
    ['corexy', 'limited_corexy'])

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]

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
    "enc_auto": lambda fx, coeffs: (fx / 1e3 / coeffs[0])
}

class AccelHelper:
    AXES_LEVEL_DELTA = 2000
    ACCEL_FILTER_THRESHOLD = 3000
    def __init__(self, axis, chip_name):
        self.axis = axis
        self.sync = axis.sync
        self.chip_name = chip_name
        self.chip_type = 'accelerometer'
        self.dim_type = 'magnitude'
        self.config = self.sync.config
        self.printer = self.sync.printer
        self.aclient = None
        self.chip_filter = None
        self.init_chip_config(chip_name)
        axis.calc_deviation = self._calc_magnitude
        axis.detect_move_dir = self._detect_move_dir
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.reactor = self.printer.get_reactor()

    def init_chip_config(self, chip_name):
        self.accel_config = self.printer.lookup_object(chip_name)
        if hasattr(self.accel_config, 'data_rate'):
            if self.accel_config.data_rate > self.ACCEL_FILTER_THRESHOLD:
                self.axis._init_chip_filter()
            else:
                self.chip_filter = lambda data: data
        elif chip_name == 'beacon':
            # Beacon sampling rate > ACCEL_FILTER_THRESHOLD
            self.axis._init_chip_filter()
        else:
            raise self.config.error(f"motors_sync: Unknown accelerometer"
                                    f" '{chip_name}' sampling rate")

    def flush_data(self):
        self.aclient.is_finished = False
        self.aclient.msgs.clear()
        self.aclient.request_start_time = None
        self.aclient.request_end_time = None

    def update_start_time(self):
        self.aclient.request_start_time = self.toolhead.get_last_move_time()

    def update_end_time(self):
        self.aclient.request_end_time = self.toolhead.get_last_move_time()

    def start_measurements(self):
        self.aclient = self.accel_config.start_internal_client()

    def finish_measurements(self):
        self.aclient.finish_measurements()

    def _wait_samples(self):
        lim = self.reactor.monotonic() + 5.
        while True:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            if self.aclient.msgs and self.aclient.request_end_time:
                last_mcu_time = self.aclient.msgs[-1]['data'][-1][0]
                if last_mcu_time > self.aclient.request_end_time:
                    return True
                elif now > lim:
                    raise self.gcode.error(
                        'motors_sync: No data from accelerometer')

    def _get_accel_samples(self):
        self._wait_samples()
        raw_data = np.concatenate(
            [np.array(m['data']) for m in self.aclient.msgs])
        start_idx = np.searchsorted(raw_data[:, 0],
                    self.aclient.request_start_time, side='left')
        end_idx = np.searchsorted(raw_data[:, 0],
                    self.aclient.request_end_time, side='right')
        t_accels = raw_data[start_idx:end_idx]
        return t_accels[:, 1:]

    def _calc_magnitude(self):
        # Calculate impact magnitude
        vects = self._get_accel_samples()
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
        magnitudes = self.chip_filter(magnitudes)
        # Calculate static noise
        static = np.mean(magnitudes[static_zone])
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitudes)[-5:])
        magnitude = np.around(magnitude - static, 2)
        self.axis.update_log(int(magnitude))
        return magnitude

    def _detect_move_dir(self):
        # Determine movement direction
        self.axis.move_dir = [1, 'unknown']
        self.sync.single_move(self.axis)
        self.axis.new_magnitude = self.sync.measure(self.axis)
        self.sync.handle_state(self.axis, 'stepped')
        if self.axis.new_magnitude > self.axis.magnitude:
            self.axis.move_dir = [-1, 'Backward']
        else:
            self.axis.move_dir = [1, 'Forward']
        self.sync.handle_state(self.axis, 'direction')
        self.axis.magnitude = self.axis.new_magnitude


class EncoderHelper:
    AXES_LEVEL_DELTA = 5
    MIN_SAMPLE_PERIOD = 0.000400
    def __init__(self, axis, chip_name):
        self.axis = axis
        self.sync = axis.sync
        self.chip_name = 'angle ' + chip_name
        self.chip_type = 'encoder'
        self.dim_type = 'deviation'
        self.config = self.sync.config
        self.printer = self.sync.printer
        self.angle_config = self.printer.lookup_object(self.chip_name)
        self._check_sample_rate()
        self._check_encoder_place()
        self.is_finished = False
        self.samples = []
        self.raw_deviation = 0
        self.request_start_time = None
        self.request_end_time = None
        axis.calc_deviation = self._calc_position
        axis.detect_move_dir = self._detect_move_dir
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.reactor = self.printer.get_reactor()

    def _check_sample_rate(self):
        per = self.angle_config.sample_period
        if per > self.MIN_SAMPLE_PERIOD:
            raise self.config.error(
                f'motors_sync: Encoder sample rate too '
                f'low: {per} < {self.MIN_SAMPLE_PERIOD}')

    def _check_encoder_place(self):
        # Swap duties between motors depending on encoder place
        binded_stepper = self.angle_config.calibration.stepper_name
        axis_steppers = [s.get_name() for s in self.axis.get_steppers()]
        if binded_stepper not in axis_steppers:
            raise self.config.error(
                f"motors_sync: Encoder '{self.chip_name}' stepper: "
                f"'{binded_stepper}' not in '{self.axis.name}' "
                f"axis steppers: '{', '.join(axis_steppers)}'")
        if binded_stepper != axis_steppers[0]:
            self.axis.swap_steppers()

    def handle_batch(self, batch):
        if self.is_finished:
            return False
        samples = batch['data']
        self.samples.extend(samples)
        return True

    def flush_data(self):
        self.is_finished = False
        self.samples.clear()
        self.request_start_time = None
        self.request_end_time = None

    def update_start_time(self):
        self.request_start_time = self.toolhead.get_last_move_time()

    def update_end_time(self):
        self.request_end_time = self.toolhead.get_last_move_time()

    def start_measurements(self):
        self.flush_data()
        self.angle_config.add_client(self.handle_batch)

    def finish_measurements(self):
        self.toolhead.wait_moves()
        self.is_finished = True

    def _wait_samples(self):
        lim = self.reactor.monotonic() + 5.
        while True:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            if self.samples and self.request_end_time:
                last_mcu_time = self.samples[-1][0]
                if last_mcu_time > self.request_end_time:
                    return True
                elif now > lim:
                    raise self.gcode.error(
                        'motors_sync: No data from encoder')

    def _get_encoder_samples(self):
        self._wait_samples()
        raw_data = np.array(self.samples)
        start_idx = np.searchsorted(raw_data[:, 0],
                    self.request_start_time, side='left')
        end_idx = np.searchsorted(raw_data[:, 0],
                    self.request_end_time, side='right')
        t_accels = raw_data[start_idx:end_idx]
        return t_accels[:, 1]

    def normalize_encoder_pos(self, pos):
        angle = (1 << 16) / pos
        length = self.axis.rd / angle
        return angle, length

    def _calc_position(self):
        # Calculate impact position µm on encoder
        positions = self._get_encoder_samples()
        poss_len = positions.shape[0]
        static_zone = range(poss_len // 5, poss_len // 3)
        # Calculate static position
        static = np.mean(positions[static_zone])
        # Return avg of 5 max positions with deduction static
        deviations = positions - static
        top_dev_ids = np.argsort(np.abs(deviations))[-5:]
        deviation = np.mean(deviations[top_dev_ids])
        dev_norm = self.normalize_encoder_pos(deviation)
        deviation = np.around(dev_norm[1] * 1e3, 2)
        abs_deviation = abs(deviation)
        self.axis.update_log(int(abs_deviation))
        self.raw_deviation = deviation
        return abs_deviation

    def _detect_move_dir(self):
        # Determine movement direction
        if self.raw_deviation < 0:
            self.axis.move_dir = [-1, 'Backward']
        else:
            self.axis.move_dir = [1, 'Forward']
        self.sync.handle_state(self.axis, 'direction')
        self.axis.new_magnitude = self.axis.magnitude


class MotionAxis:
    VALID_MSTEPS = [256, 128, 64, 32, 16, 8, 0]
    def __init__(self, sync, name, jx, ph_off):
        self.sync = sync
        self.name = name
        self.joint_axes = jx.get(name, [])
        self.phase_offset = ph_off.get(name, None)
        self.config = sync.config
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
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
        stepper = 'stepper_' + name
        st_section = self.config.getsection(stepper)
        min_pos = st_section.getfloat('position_min', 0)
        max_pos = st_section.getfloat('position_max')
        self.rd = st_section.getfloat('rotation_distance')
        fspr = st_section.getint('full_steps_per_rotation', 200)
        self.limits = (min_pos + 10, max_pos - 10, (min_pos + max_pos) / 2)
        self.do_buzz = True
        self.rel_buzz_d = self.rd / fspr * 5
        msteps_dict = {m: m for m in self.VALID_MSTEPS}
        self.microsteps = self.config.getchoice(
            f'microsteps_{name}', msteps_dict, default=0)
        if not self.microsteps:
            self.microsteps = self.config.getchoice(
                'microsteps', msteps_dict, default=16)
        self.move_d = self.rd / fspr / self.microsteps
        sync.add_connect_task(self._init_steppers)
        sync.add_connect_task(self._init_tmc_drivers)
        self._init_chip_helper()
        sync.add_connect_task(self._init_fan)
        self.conf_fan = self.config.get(f'head_fan_{name}', '')
        if not self.conf_fan:
            self.conf_fan = self.config.get('head_fan', None)
        msmax = self.microsteps / 2
        self.max_step_size = self.config.getint(
            f'max_step_size_{name}', default=0, minval=1, maxval=msmax)
        if not self.max_step_size:
            self.max_step_size = self.config.getint(
                'max_step_size', default=3, minval=1, maxval=msmax)
        self.axes_steps_diff = self.config.getint(
            f'axes_steps_diff_{name}', default=0, minval=1)
        if not self.axes_steps_diff:
            self.axes_steps_diff = self.config.getint(
                'axes_steps_diff', self.max_step_size + 1, minval=1)
        rmin = self.move_d * 1e3
        self.retry_tolerance = self.config.getfloat(
            f'retry_tolerance_{name}', default=0, above=rmin)
        if not self.retry_tolerance:
            self.retry_tolerance = self.config.getfloat(
                'retry_tolerance', default=0, above=rmin)
        self.max_retries = self.config.getint(
            f'retries_{name}', default=0, minval=0, maxval=10)
        if not self.max_retries:
            self.max_retries = self.config.getint(
                'retries', default=0, minval=0, maxval=10)

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

    def swap_steppers(self):
        self.steppers.reverse()
        self.tmcs.reverse()

    def get_steppers(self):
        return self.steppers

    def get_phase(self, tmc):
        field_name = "mscnt"
        if tmc.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "mstep"
        reg = tmc.fields.lookup_register(field_name)
        phase = tmc.mcu_tmc.get_register(reg)
        phase = tmc.fields.get_field(field_name, phase)
        stepper = self.steppers[self.tmcs.index(tmc)]
        if not stepper.get_dir_inverted()[0]:
            return 1023 - phase
        return phase

    def get_phase_offset(self):
        p1, p2 = (self.get_phase(t) for t in self.tmcs)
        return p2 - p1

    def update_phase_offset(self):
        self.phase_offset = self.get_phase_offset()

    def set_phase_offset(self):
        # Set phase offset by <axis>1 stepper
        if self.get_phase_offset() != 0:
            return
        if self.phase_offset is None:
            return
        mode_d = self.phase_offset / 256 * self.move_d * self.microsteps
        if abs(mode_d) < self.move_d * 2:
            return
        self.toggle_main_stepper(0, (PIN_MIN_TIME, PIN_MIN_TIME))
        self.sync.stepper_move(self.steppers[1], mode_d)
        msteps = int(mode_d // self.move_d)
        self.gcode.respond_info(
            f'{self.name.upper()}-Restore previous '
            f'position: {msteps}/{self.microsteps} step')

    def toggle_main_stepper(self, mode, times=None):
        if times is None:
            times = (MOTOR_STALL_TIME,)*2
        elif len(times) < 2:
            times = (*times, MOTOR_STALL_TIME)
        self.sync.stepper_enable(self.steppers[0].get_name(), mode, *times)

    def toggle_steppers(self, mode):
        for st in self.steppers:
            self.sync.stepper_enable(st.get_name(), mode,
                PIN_MIN_TIME, PIN_MIN_TIME)

    def toggle_joint_axes(self, mode):
        for name in self.joint_axes:
            self.sync.motion[name].toggle_steppers(mode)

    def update_log(self, deviation):
        self.log.append([int(deviation), self.actual_msteps])

    def _init_steppers(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        belt_steppers = [s for s in kin.get_steppers()
                         if 'stepper_' + self.name in s.get_name()]
        if len(belt_steppers) not in (2,):
            raise self.config.error(f"motors_sync: Not supported "
                                    f"'{len(belt_steppers)}' count of motors")
        for steppers in belt_steppers:
            st_section = self.config.getsection(steppers.get_name())
            st_msteps = st_section.getint('microsteps')
            if self.microsteps > st_msteps:
                raise self.config.error(
                    f'motors_sync: Invalid microsteps count, cannot be '
                    f'more than steppers, {self.microsteps} vs {st_msteps}')
        self.steppers = belt_steppers

    def _init_tmc_drivers(self):
        self.tmcs = []
        for stepper in self.steppers:
            for driver in TRINAMIC_DRIVERS:
                driver_name = f"{driver} {stepper.get_name()}"
                module = self.printer.lookup_object(driver_name, None)
                if module is not None:
                    self.tmcs.append(module)
                    break
            else:
                raise self.config.error(f"Unable to find TMC driver for "
                                        f"'{stepper.get_name()}' stepper")

    def _init_steps_models(self, def_model):
        # todo: rewrite all func logic
        models = {
            'linear': {'ct': 2, 'a': None, 'f': MATH_MODELS['polynomial']},
            'quadratic': {'ct': 3, 'a': None, 'f': MATH_MODELS['polynomial']},
            'power': {'ct': 2, 'a': None, 'f': MATH_MODELS['power']},
            'root': {'ct': 2, 'a': 0, 'f': MATH_MODELS['root']},
            'hyperbolic': {'ct': 2, 'a': 0, 'f': MATH_MODELS['hyperbolic']},
            'exponential': {'ct': 3, 'a': 0, 'f': MATH_MODELS['exponential']},
            'enc_auto': {'ct': 1, 'a': -1, 'f': MATH_MODELS['enc_auto']},
        }
        model = self.config.getlist(f'steps_model_{self.name}', None)
        if model is None:
            model = self.config.getlist('steps_model', def_model)
        model_name = model[0]
        coeffs_vals = list(map(float, model[1:]))
        coeffs_args = [chr(97 + i) for i in range(len(coeffs_vals) + 1)]
        model_coeffs = {arg: float(val)
                        for arg, val in zip(coeffs_args, coeffs_vals)}
        model_config = models.get(model_name, None)
        if model_config is None:
            raise self.config.error(
                f"motors_sync: Invalid steps model '{model_name}'")
        if len(model_coeffs) != model_config['ct']:
            raise self.config.error(
                f"motors_sync: Steps model '{model_name}' "
                f"requires {model_config['ct']} coefficients")
        if model_coeffs['a'] == model_config['a']:
            raise self.config.error(
                f"motors_sync: Coefficient 'a' cannot be "
                f"{model_coeffs['a']} for a '{model_name}' model")
        self.model_name = model_name
        self.model_coeffs = tuple(model_coeffs.values())
        model_scale = self.microsteps / 16
        if model_name == 'enc_auto':
            model_scale = 1
        def model_solve(fx=None):
            if fx is None:
                fx = self.new_magnitude
            res = model_config['f'](fx, self.model_coeffs)
            if np.isnan(res):
                self.sync.handle_state(self,
                    f"Microsteps calculation returned NaN "
                    f"for {self.name.capitalize()} axis")
            return res * model_scale
        self.model_solve = model_solve

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
            chip_filter = lambda samples, w=window: np.median(
                [samples[i - w:i + w + 1]
                 for i in range(w, len(samples) - w)], axis=1)
        elif filter == 'kalman':
            coeffs = self.config.getfloatlist(
                f'kalman_coeffs_{self.name}',
                default=tuple('' for _ in range(6)), count=6)
            if not all(coeffs):
                coeffs = self.config.getfloatlist('kalman_coeffs',
                    default=tuple((1.1, 1., 1e-1, 1e-2, .5, 1.)), count=6)
            chip_filter = KalmanLiteFilter(*coeffs).process_samples
        # If chip_helper is not init, defer the task to klippy connect state
        if hasattr(self, 'chip_helper'):
            self.chip_helper.chip_filter = chip_filter
        else:
            self.sync.add_connect_task(lambda: setattr(
                self.chip_helper, 'chip_filter', chip_filter))

    def _init_chip_helper(self):
        accel_chip_name = self.config.get(f'accel_chip_{self.name}', '')
        enc_chip_name = self.config.get(f'encoder_chip_{self.name}', '')
        if accel_chip_name and enc_chip_name:
            raise self.config.error(f"motors_sync: Only 1 sensor "
                                    f"type can be selected")
        if not accel_chip_name and not enc_chip_name:
            accel_chip_name = self.config.get('accel_chip', '')
        if not accel_chip_name and not enc_chip_name:
            raise self.config.error(
                f"motors_sync: Sensors type 'accel_chip' or "
                f"'encoder_chip_<axis>' must be provided")
        if accel_chip_name:
            # Init accelerometer config on klippy connect
            self.sync.add_connect_task(lambda: setattr(self,
                'chip_helper', AccelHelper(self, accel_chip_name)))
            self._init_chip_filter()
            def_steps_model = ['linear', 20000, 0]
            self._init_steps_models(def_steps_model)
        elif enc_chip_name:
            # Init encoder config on klippy connect
            self.sync.add_connect_task(lambda: setattr(self,
                'chip_helper', EncoderHelper(self, enc_chip_name)))
            def_steps_model = ['enc_auto', self.move_d]
            self._init_steps_models(def_steps_model)

    def _create_fan_switch(self, method):
        if method == 'heater_fan':
            def fan_switch(on=True):
                if not self.fan:
                    return
                now = self.reactor.monotonic()
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
        self.connect_tasks = []
        # Read config
        self._init_stat_manager()
        self._init_axes()
        self._init_sync_method()
        # Register commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_SYNC_MOTORS,
                                    desc=self.cmd_SYNC_MOTORS_help)
        self.gcode.register_command('SYNC_MOTORS_CALIBRATE',
                                    self.cmd_SYNC_MOTORS_CALIBRATE,
                                    desc=self.cmd_SYNC_MOTORS_CALIBRATE_help)
        # Variables
        self.reactor = self.printer.get_reactor()

    def add_connect_task(self, task):
        self.connect_tasks.append(task)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = min(self.toolhead.max_accel, 5000)
        self.kin = self.toolhead.get_kinematics()
        for task in self.connect_tasks: task()
        self.connect_tasks.clear()

    def _init_axes_phase_offset(self, axes):
        raw_log = self.log_chelper.read_log()
        if raw_log.size == 0:
            return {}
        log = self.log_chelper.parse_raw_log(raw_log)
        a = {}
        for p in log[::-1]:
            if all(ax in a for ax in axes):
                return a
            if p[0] in a:
                continue
            if not p[1]:
                continue
            a[p[0]] = p[6]
        return a

    def _check_common_attr(self):
        # Apply restrictions for LEVELING_KINEMATICS kinematics
        common_attr = ['microsteps', 'model_name', 'model_coeffs',
                       'max_step_size', 'axes_steps_diff']
        for attr in common_attr:
            diff = set([getattr(cls, attr) for cls in self.motion.values()])
            if len(diff) < 2:
                continue
            if (attr == 'chip_name' and list(self.motion.values())[0]
                 .chip_helper.chip_type == 'encoder'):
                continue
            params_str = ', '.join(f"'{attr}: {v}'" for v in diff)
            raise self.config.error(
                f"motors_sync: Options {params_str} cannot be "
                f"different for a '{self.conf_kin}' kinematics")

    def _init_axes(self):
        valid_axes = ['x', 'y']
        printer_section = self.config.getsection('printer')
        self.conf_kin = printer_section.get('kinematics')
        if self.conf_kin in LEVELING_KINEMATICS:
            self.do_level = True
            axes = [a.lower() for a in self.config.getlist(
                'axes', count=2, default=['x', 'y'])]
            joint_ax = {'x': ['y'], 'y': ['x']}
        elif self.conf_kin == 'cartesian':
            self.do_level = False
            axes = [a.lower() for a in self.config.getlist('axes')]
            joint_ax = {}
        else:
            raise self.config.error(f"motors_sync: Not supported "
                                    f"kinematics '{self.conf_kin}'")
        if any(axis not in valid_axes for axis in axes):
            raise self.config.error(f"motors_sync: Invalid axes "
                                    f"parameter '{','.join(axes)}'")
        ph_off = self._init_axes_phase_offset(axes)
        logging.info(f'motors_sync_phase_offset: {ph_off}')
        self.motion = {ax: MotionAxis(self, ax, joint_ax, ph_off)
                       for ax in axes}
        if self.conf_kin in LEVELING_KINEMATICS:
            self._check_common_attr()

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
            raise self.config.error(
                f"motors_sync: Invalid sync method: {self.sync_method} "
                f"for '{self.conf_kin}' type kinematics")

    def _init_stat_manager(self):
        command = 'SYNC_MOTORS_STATS'
        filename = 'sync_stats.csv'
        format = 'axis,status,magnitudes,steps,msteps,retries,offset,date,'
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
                offset = axis.get_phase_offset()
                date = datetime.now().strftime('%Y-%m-%d')
                manager.write_log([name, status, magnitudes, pos,
                                   msteps, retries, offset, date])
        self.log_chelper = manager
        self.write_log = write_log

    def gsend(self, params):
        self.gcode.run_script_from_command(params)

    def stepper_enable(self, stepper, mode, ontime, offtime):
        self.toolhead.dwell(ontime)
        print_time = self.toolhead.get_last_move_time()
        el = self.stepper_en.enable_lines[stepper]
        el.motor_enable(print_time) if mode \
            else el.motor_disable(print_time)
        self.toolhead.dwell(offtime)

    def stepper_move(self, mcu_stepper, dist):
        self.force_move.manual_move(mcu_stepper, dist,
            self.travel_speed, self.travel_accel)

    def single_move(self, axis, mcu_stepper=None, dir=1):
        # Move <axis>1 stepper motor by default
        if mcu_stepper is None:
            mcu_stepper = axis.get_steppers()[1]
        move_msteps = axis.move_msteps * axis.move_dir[0] * dir
        dist = axis.move_d * move_msteps
        axis.actual_msteps += move_msteps
        axis.check_msteps += move_msteps
        self.stepper_move(mcu_stepper, dist)

    def buzz(self, axis, rel_moves=25):
        # Fading oscillations by <axis>1 stepper
        mcu_stepper1 = axis.get_steppers()[1]
        last_abs_pos = 0
        axis.toggle_main_stepper(0, (PIN_MIN_TIME,)*2)
        for osc in reversed(range(0, rel_moves)):
            abs_pos = axis.rel_buzz_d * (osc / rel_moves)
            for inv in [1, -1]:
                abs_pos *= inv
                dist = (abs_pos - last_abs_pos)
                last_abs_pos = abs_pos
                self.stepper_move(mcu_stepper1, dist)

    def measure(self, axis):
        # Measure the impact
        if axis.do_buzz:
            self.buzz(axis)
        axis.chip_helper.flush_data()
        axis.toggle_main_stepper(1, (PIN_MIN_TIME,))
        axis.toggle_main_stepper(0, (PIN_MIN_TIME,))
        axis.chip_helper.update_start_time()
        axis.toggle_main_stepper(1)
        axis.chip_helper.update_end_time()
        if axis.do_buzz:
            self.buzz(axis, 5)
        else:
            axis.toggle_main_stepper(0)
        return axis.calc_deviation()

    def homing(self):
        # Homing and going to center
        now = self.reactor.monotonic()
        axes, confs = zip(*self.motion.items())
        if ''.join(axes) not in self.kin.get_status(now)['homed_axes']:
            self.gsend(f"G28 {' '.join(axes)}")
        center_pos = ' '.join(f'{a}{c.limits[2]}' for a, c in zip(axes, confs))
        self.gsend(f"G0 {center_pos} F{self.travel_speed * 60}")
        self.toolhead.dwell(MOTOR_STALL_TIME)

    def handle_state(self, axis, state=''):
        name = axis.name.upper()
        dim_type = axis.chip_helper.dim_type
        if state == 'stepped':
            msteps = axis.move_msteps * axis.move_dir[0]
            msg = (f"{name}-New {dim_type}: {axis.new_magnitude} "
                   f"on {msteps}/{axis.microsteps} step move")
        elif state == 'static':
            msg = f"{name}-New {dim_type}: {axis.new_magnitude}"
        elif state == 'direction':
            msg = f"{name}-Movement direction: {axis.move_dir[1]}"
        elif state == 'start':
            axis.flush_motion_data()
            axis.fan_switch(False)
            axis.chip_helper.start_measurements()
            axis.init_magnitude = axis.magnitude = self.measure(axis)
            msg = (f"{axis.name.upper()}-Initial {dim_type}: "
                   f"{axis.init_magnitude}")
        elif state == 'done':
            axis.fan_switch(True)
            axis.chip_helper.finish_measurements()
            axis.toggle_main_stepper(1, (PIN_MIN_TIME,)*2)
            axis.update_phase_offset()
            msg = (f"{name}-Motors adjusted by {axis.actual_msteps}/"
                   f"{axis.microsteps} step, {dim_type} "
                   f"{axis.init_magnitude} --> {axis.magnitude}")
        elif state == 'retry':
            axis.move_dir[1] = 'unknown'
            msg = (f"{name}-Retries: {axis.curr_retry}/{axis.max_retries} "
                   f"Back to last {dim_type}: {axis.magnitude} on "
                   f"{axis.actual_msteps}/{axis.microsteps} step "
                   f"to reach {axis.retry_tolerance}")
        else:
            for axis in [c for a, c in self.motion.items() if a in self.axes]:
                axis.fan_switch(True)
                axis.chip_helper.finish_measurements()
            raise self.gcode.error(state)
        self.gcode.respond_info(msg, True)

    def _axes_level(self, m, s):
        # Axes leveling by magnitude
        # "m" is a main axis, "s" is a second axis
        delta = m.init_magnitude - s.init_magnitude
        target_delta = m.chip_helper.AXES_LEVEL_DELTA
        if delta <= target_delta:
            return
        self.gcode.respond_info(
            f'Start axes level, delta: {delta:.2f}', True)
        force_exit = False
        while True:
            self.check_axis_drift(s, m)
            if m.move_dir[1] == 'unknown':
                m.detect_move_dir()
            steps_delta = int(m.model_solve() - m.model_solve(s.magnitude))
            m.move_msteps = min(max(steps_delta, 1), m.max_step_size)
            self.single_move(m)
            m.new_magnitude = self.measure(m)
            self.handle_state(m, 'stepped')
            if m.new_magnitude > m.magnitude:
                self.single_move(m, dir=-1)
                if m.retry_tolerance and m.magnitude > m.retry_tolerance:
                    m.curr_retry += 1
                    if m.curr_retry > m.max_retries:
                        self.handle_state(m, 'done')
                        self.write_log(m)
                        self.handle_state(m, 'Too many retries')
                    self.handle_state(m, 'retry')
                    continue
                force_exit = True
            m.magnitude = m.new_magnitude
            delta = m.new_magnitude - s.magnitude
            if (delta < target_delta
                    or m.new_magnitude < s.magnitude
                    or force_exit):
                self.gcode.respond_info(
                    f"Axes are leveled: {m.name.upper()}: "
                    f"{m.init_magnitude} --> {m.new_magnitude}, "
                    f"{s.name.upper()}: {s.init_magnitude} "
                    f"--> {s.magnitude}, delta: {delta:.2f}", True)
                return
            continue

    def _single_sync(self, m):
        # "m" is a main axis, just single axis
        if m.move_dir[1] == 'unknown':
            if not m.actual_msteps or m.curr_retry:
                m.new_magnitude = self.measure(m)
                m.magnitude = m.new_magnitude
                self.handle_state(m, 'static')
            if (not m.actual_msteps
                    and m.retry_tolerance
                    and m.new_magnitude < m.retry_tolerance):
                m.is_finished = True
                return
            m.detect_move_dir()
        m.move_msteps = min(max(
            int(m.model_solve()), 1), m.max_step_size)
        self.single_move(m)
        m.new_magnitude = self.measure(m)
        self.handle_state(m, 'stepped')
        if m.new_magnitude > m.magnitude:
            self.single_move(m, dir=-1)
            if m.retry_tolerance and m.magnitude > m.retry_tolerance:
                m.curr_retry += 1
                if m.curr_retry > m.max_retries:
                    # Write error in log
                    self.write_log(m)
                    self.handle_state(m, 'done')
                    self.handle_state(m, 'Too many retries')
                self.handle_state(m, 'retry')
                return
            m.is_finished = True
            return
        m.magnitude = m.new_magnitude

    def check_axis_drift(self, m, s):
        # Note: m.axes_steps_diff == s.axes_steps_diff
        steps_diff = abs(abs(m.check_msteps) - abs(s.check_msteps))
        if steps_diff >= m.axes_steps_diff:
            m.new_magnitude = m.magnitude = self.measure(m)
            self.handle_state(m, 'static')
            m.check_msteps, s.check_msteps = 0, 0
            return True

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
            cycling = itertools.cycle(self.axes)
            max_ax = [c for c in sorted(
                self.motion.values(), key=lambda i: i.init_magnitude)][-1]
            max_ax.detect_move_dir()
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
                    self.check_axis_drift(m, s)
                    continue
                self._single_sync(m)
        elif self.sync_method == 'sequential' or len(self.axes) == 1:
            for axis in self.axes:
                m = self.motion[axis]
                # To skip measure() in _single_sync()
                m.detect_move_dir()
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
            if m.chip_helper.chip_type != 'accelerometer':
                continue
            ax_chip = gcmd.get(f'ACCEL_CHIP_{axis.upper()}', chip).lower()
            if ax_chip and ax_chip != m.chip_helper.chip_name:
                try:
                    self.printer.lookup_object(ax_chip)
                except Exception as e:
                    raise self.gcode.error(e)
                self.motion[axis].chip_helper.init_chip_config(ax_chip)
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
        self.homing()
        self.gcode.respond_info('Motors synchronization started', True)
        # Try restore last sync position on cold start
        for axis in self.axes:
            self.motion[axis].set_phase_offset()
        # Init axes magnitudes
        for axis in self.axes:
            self.handle_state(self.motion[axis], 'start')
        # Check if all axes in tolerance
        if not force_run and all(m.init_magnitude < m.retry_tolerance
             for m in (self.motion[ax] for ax in self.axes)):
            retry_tols = ''
            for axis in self.axes:
                m = self.motion[axis]
                m.chip_helper.finish_measurements()
                m.fan_switch(True)
                retry_tols += f'{m.name.upper()}: {m.retry_tolerance}, '
            self.gcode.respond_info(f"Motors magnitudes are in "
                                    f"tolerance: {retry_tols}", True)
        else:
            self._run_sync()
            # Info
            for axis in self.axes:
                self.handle_state(self.motion[axis], 'done')
        self.status.check_retry_result('done')
        self.write_log()

    cmd_SYNC_MOTORS_CALIBRATE_help = 'Calibrate synchronization process model'
    def cmd_SYNC_MOTORS_CALIBRATE(self, gcmd):
        # Calibrate sync model and model coeffs
        if not hasattr(self, 'sync_calibrate_helper'):
            self.sync_calibrate_helper = MotorsSyncCalibrate(self)
        self.sync_calibrate_helper.run_calibrate(gcmd)
        self.status.reset()

    def get_status(self, eventtime):
        return self.status.get_status(eventtime)


class MotorsSyncCalibrate:
    def __init__(self, sync):
        self.sync = sync
        self.gcode = sync.gcode
        try:
            self._load_modules()
        except ImportError as e:
            self.gcode.error(f'Could not import: {e}')
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

    math_models = {
        'linear': (lambda x, a, b: a*x + b, '-.', '#DF8816'),
        'quadratic': (lambda x, a, b, c: a*x**2 + b*x + c, '--', 'green'),
        'power': (lambda x, a, b: a * np.power(x, b), ':', 'cyan'),
        'root': (lambda x, a, b: a * np.sqrt(x) + b, '--', 'magenta'),
        'hyperbolic': (lambda x, a, b: a / x + b, '-.', 'purple'),
        'exponential': (lambda x, a, b, c: a * np.exp(b*x) + c, ':', 'blue')
    }

    def find_best_func(self, x_data, y_data, maxfev=999999999):
        funcs = []
        for name, param in self.math_models.items():
            coeffs, _ = curve_fit(param[0], x_data, y_data, maxfev=maxfev)
            y_pred = param[0](x_data, *coeffs)
            rmse = np.sqrt(np.mean((y_data - y_pred) ** 2))
            funcs.append({'name': name, 'rmse': rmse, 'coeffs': coeffs})
        funcs = sorted(funcs, key=lambda f: f['rmse'])
        info = ['Functions RMSE and coefficients']
        for f in funcs:
            c_str = ','.join([f'{c:.10f}' for c in f['coeffs']])
            info.append(f"{f['name']}: RMSE {f['rmse']:.2f} coeffs: {c_str}")
        return info, (x_data, y_data, funcs)

    def plotter(self, x_data, y_data, funcs, axis, accel_chip,
                peak_msteps, fullstep, rmse_lim=20000):
        # Plotting
        pow_lim = (-2, 2)
        fig, ax = plt.subplots()
        ax.scatter(x_data, y_data, label='Samples',
                   color='red', zorder=2, s=10)
        x_fit = np.linspace(min(x_data), max(x_data), 200)
        for func in funcs:
            rmse = func['rmse']
            if rmse < rmse_lim:
                upname = func['name'].capitalize()
                model = self.math_models[func['name']]
                _func = model[0](x_fit, *func['coeffs'])
                c_str = ','.join([f'{c:.3f}' for c in func['coeffs']])
                func_str = f"{upname} RMSE: {rmse:.2f} coeffs: {c_str}"
                linestyle = model[1]
                color = model[2]
                ax.plot(x_fit, _func, label=func_str,
                        linestyle=linestyle, linewidth=1, color=color)
        ax.legend(loc='lower right', fontsize=6, framealpha=1, ncol=1)
        accel_chip = accel_chip.replace(' ', '-')
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        lognames = (f'calibrate_plot_{axis}_{peak_msteps}-'
                    f'{fullstep}_{accel_chip}_{now}.png')
        title = (f"Dependency of desynchronization "
                 f"and functions ({''.join(lognames)})")
        ax.set_title('\n'.join(wrap(title, 66)), fontsize=10)
        ax.set_xlabel(f'Microsteps: 1/{fullstep}')
        ax.set_xticks(np.arange(0, max(x_data) + 2.5, 2.5))
        ax.xaxis.set_minor_locator(ticker.MultipleLocator(2.5))
        ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.set_ylabel('Magnitude')
        ax.ticklabel_format(axis='y', style='scientific', scilimits=pow_lim)
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(which='major', color='grey')
        ax.grid(which='minor', color='lightgrey')
        png_path = os.path.join(self.path, lognames)
        plt.savefig(png_path, dpi=1000)
        return f'Access to interactive plot at: {png_path}'

    def save_config(self, axis, func):
        configfile = self.sync.printer.lookup_object("configfile")
        axes = [axis] + [a for a in self.sync.motion[axis].joint_axes]
        for ax in axes:
            pname = f'steps_model_{ax}'
            func_name = func['name']
            func_coeffs = list(map(str, func['coeffs']))
            block = func_name + ',\n  ' + ',\n  '.join(func_coeffs)
            configfile.set('motors_sync', pname, block)
            msg = f"{pname}: {func_name}, {','.join(func_coeffs)}"
            self.gcode.respond_info(msg)
        self.gcode.respond_info(
            f"Motors sync model for '{', '.join(axes)}' axis "
             "has been calibrated.\nThe SAVE_CONFIG command "
             "will update the printer config\n file with new "
             "parameters and restart the printer.")

    def run_calibrate(self, gcmd, fullstep=16):
        # "m" is a main axis, just single axis
        repeats = gcmd.get_int('REPEATS', 2, minval=2, maxval=100)
        axis = gcmd.get('AXIS').lower()
        m = self.sync.motion.get(axis, None)
        if m is None:
            self.gcode.error(f'Invalid axis: {axis.upper()}')
        peak_mstep = gcmd.get_int('DISTANCE', fullstep,
                                  minval=2, maxval=fullstep*2)
        need_plot = False
        need_plot_str = gcmd.get('PLOT', 'True').lower()
        if need_plot_str == 'true' or need_plot_str == '1':
            need_plot = True
        self.gcode.respond_info(
            f'Calibration started on {axis.upper()} axis with '
            f'{repeats} repeats, move to +-{peak_mstep}/16 microstep')
        self.gcode.respond_info('Synchronizing before calibration...')
        self.sync.cmd_SYNC_MOTORS(gcmd, force_run=True)
        max_steps = 0
        invs = [1, -1, -1, 1]
        y_samples = [-1,]
        mcu_stepper1 = m.get_steppers()[1]
        # Scale calibration steps
        m.move_msteps = m.microsteps // fullstep
        emul_peak_mstep = peak_mstep * m.move_msteps
        looped_pos = itertools.cycle([m.rd, -m.rd, -m.rd, m.rd])
        self.sync.handle_state(m, 'start')
        for r in range(1, repeats + 1):
            self.gcode.respond_info(
                f'Repeats: {r}/{repeats} Move to +-'
                f'{emul_peak_mstep}/{m.microsteps} microstep')
            self.sync.stepper_move(mcu_stepper1, next(looped_pos))
            for inv in invs:
                m.move_dir[0] = inv
                for _ in range(peak_mstep):
                    self.sync.single_move(m)
                    m.new_magnitude = self.sync.measure(m)
                    self.sync.handle_state(m, 'stepped')
                    if m.new_magnitude > max(y_samples):
                        max_steps += 1
                    y_samples.append(m.new_magnitude)
        m.magnitude = m.new_magnitude
        # Manual handle_state('done')
        m.fan_switch(True)
        m.chip_helper.finish_measurements()
        # To array with removed first sample
        y_samples = np.sort(np.array(y_samples[1:]))
        x_samples = np.linspace(0.01, max_steps, len(y_samples))
        x_samples_str = ', '.join([f'{i:.2f}' for i in x_samples])
        y_samples_str = ', '.join([str(i) for i in y_samples])
        logging.info(f"motors_sync_calibrate: x = [{x_samples_str}]")
        logging.info(f"motors_sync_calibrate: y = [{y_samples_str}]")
        # In general, it should be taken out to samples_processing()
        msg, data = self.find_best_func(x_samples, y_samples)
        self.save_config(axis, data[-1][0])
        if not need_plot:
            return

        def samples_processing():
            try:
                os.nice(10)
            except:
                pass
            self.gcode.respond_info('Generating a plot...', True)
            msg = self.plotter(*data, axis, m.chip_helper.chip_name,
                               peak_mstep, fullstep)
            self.gcode.respond_info(msg, True)

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

    def flush_data(self):
        self.x = self.st_x
        self.P = self.st_p

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
        self.flush_data()
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
                self.clear_log()
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
        self.error = ''
        self.check_log()

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
