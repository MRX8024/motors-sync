# Motors synchronization script
#
# Copyright (C) 2024-2026  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, traceback, itertools, csv, ast, importlib.util
import threading, multiprocessing
from datetime import datetime
import numpy as np
import chelper
from .z_tilt import ZAdjustStatus

PLOT_PATH = '~/printer_data/config/adxl_results/motors_sync'
PIN_MIN_TIME = 0.010            # Minimum wait time to enable hardware pin
MOTOR_STALL_TIME = 0.100        # Minimum wait time to enable motor pin

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]

# Base class for printer kinematics that describes the virtual-axes,
# synchronization logic, calibration logic for a specific kinematics.
class BaseKinematics:
    def __init__(self, config, sync):
        self.stats_helper = sync.stats_helper
        self.msg_helper = sync.msg_helper
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.motion_axes = {}
        self._init_axes(config, sync)
        sync.add_connect_task(self._handle_connect)
        sync.add_connect_task(lambda: self._init_axes_steppers(config))

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.toolhead_kin = self.toolhead.get_kinematics()

    def _init_axes(self, config, sync):
        raise NotImplementedError("Internal error in motors_sync")

    def _init_axes_steppers(self, config):
        raise NotImplementedError("Internal error in motors_sync")

    def get_motion_axes(self):
        return self.motion_axes

    def home_rails(self, axes):
        homed_axes = self.toolhead_kin.get_status(None)['homed_axes']
        axes_to_home = sorted({ax for axis in axes for ax in
                               axis.get_physical_axes()})
        if any(a not in homed_axes for a in axes_to_home):
            command = f"G28 {' '.join(axes_to_home)}"
            self.gcode.run_script_from_command(command)

    def apply_axes_phase_offsets(self, axes):
        # Try restore last sync position on cold start
        self.toolhead.wait_moves()
        applied = False
        for axis in axes:
            applied |= axis.apply_phase_offset()
        self.toolhead.dwell(MOTOR_STALL_TIME)
        return applied

    def is_axes_in_tolerance(self, axes):
        return all(a.init_magnitude < a.retry_tolerance for a in axes)

    def axis_sync_step(self, m):
        # "m" is a main single MotionAxis
        m.move_on_measure_pos()
        if m.is_move_dir_unknown():
            if not m.actual_msteps or m.curr_retry:
                m.new_magnitude = m.measure_deviation()
                m.magnitude = m.new_magnitude
                self.msg_helper.static_msg(m)
            if (not m.actual_msteps
                    and m.retry_tolerance
                    and m.new_magnitude < m.retry_tolerance):
                m.on_finish()
                return
            m.detect_move_dir()
        m.calc_move_msteps()
        m.step_move()
        m.new_magnitude = m.measure_deviation()
        self.msg_helper.stepped_msg(m)
        if m.new_magnitude > m.magnitude:
            m.step_move(dir=-1)
            if m.retry_tolerance and m.magnitude > m.retry_tolerance:
                m.curr_retry += 1
                if m.curr_retry > m.max_retries:
                    raise Exception('Too many retries')
                m.set_move_dir(0)
                self.msg_helper.retry_msg(m)
                return
            m.on_finish()
            return
        m.magnitude = m.new_magnitude

    def axes_sync(self, axes):
        raise self.gcode.error("Not implemented for this kinematics")

    def end_sync(self, axes):
        for axis in axes:
            axis.on_done()
            self.msg_helper.done_msg(axis)

    def on_sync_error(self, axes):
        for axis in axes:
            axis.on_error()
            self.msg_helper.done_msg(axis)

    def run_sync(self, axes, force_run=False):
        self.home_rails(axes)
        self.apply_axes_phase_offsets(axes)
        # Init axes magnitudes
        for ax in axes:
            ax.on_start()
            ax.init_magnitude = ax.magnitude = ax.measure_deviation()
            self.msg_helper.start_msg(ax)
        # Check if all axes in tolerance
        if not force_run and self.is_axes_in_tolerance(axes):
            self.end_sync(axes)
            self.msg_helper.in_tolerance_msg(axes)
            return
        self.axes_sync(axes)
        self.end_sync(axes)

    def start_sync(self, axes=None, force_run=False):
        if axes is None:
            axes = list(self.motion_axes.values())
        try:
            self.run_sync(axes, force_run)
        except Exception as e:
            try:
                self.on_sync_error(axes)
            except:
                raise Exception("Internal error in motors_sync")
            logging.error(f"Internal error in motors_sync:\n "
                          f"{traceback.format_exc()}")
            raise self.gcode.error(str(e))

    def axis_calibrate_cycle(self, m, peak_mstep, repeats):
        # "m" is a main single MotionAxis
        max_steps = 0
        invs = [1, -1, -1, 1]
        samples = [-1,]
        looped_pos = itertools.cycle([m.rd, -m.rd, -m.rd, m.rd])
        m.on_start()
        m.move_on_measure_pos()
        # Scale calibration steps to 1/16
        m.move_msteps = m.microsteps // 16
        m.toggle_main_stepper(0)
        for r in range(1, repeats + 1):
            self.gcode.respond_info(f'Repeats: {r}/{repeats}')
            m.manual_move(next(looped_pos))
            for inv in invs:
                m.set_move_dir(inv)
                for _ in range(peak_mstep):
                    m.step_move()
                    m.new_magnitude = m.measure_deviation()
                    self.msg_helper.stepped_msg(m)
                    if m.new_magnitude > max(samples):
                        max_steps += 1
                    samples.append(m.new_magnitude)
        if repeats % 2 == 1:
            m.manual_move(next(looped_pos))
        m.magnitude = m.new_magnitude
        m.on_done()
        return max_steps, samples[1:]

    def calibrate_axis_steps_model(self, axis, peak_mstep, repeats):
        raise self.gcode.error("Not implemented for this kinematics")

    def get_linked_calibration_axes(self, axis):
        raise self.gcode.error("Not implemented for this kinematics")


# Axially uncoupled cartesian kinematics implementation
class CartesianKinematics(BaseKinematics):
    def __init__(self, config, sync):
        super().__init__(config, sync)

    @staticmethod
    def get_axes_rails_center(config, axes):
        positions = {}
        for axis in axes:
            st_section = config.getsection('stepper_' + axis)
            min_pos = st_section.getfloat('position_min', 0)
            max_pos = st_section.getfloat('position_max')
            positions.update({axis: (min_pos + max_pos) / 2})
        return [positions.get(a, None) for a in ['x', 'y', 'z']]

    def _init_axes(self, config, sync):
        valid_axes = ['x', 'y']
        axes = sorted([a.lower() for a in config.getlist('axes')])
        if any(a not in valid_axes for a in axes):
            raise config.error(f"motors_sync: Invalid axes '{axes}'")
        sync_pos = self.get_axes_rails_center(config, axes)
        ph_offs = self.stats_helper.get_axes_phase_offsets(axes)
        self.motion_axes.update(
            {ax: MotionAxis(config, sync, ax, axes, ph_offs.get(ax),
                'stepper_' + ax, sync_pos, False) for ax in axes})

    def _init_axes_steppers(self, config):
        for axis in self.motion_axes.values():
            belt_steppers = [s for s in self.toolhead_kin.get_steppers()
                             if 'stepper_' + axis.name in s.get_name()]
            if len(belt_steppers) not in (2,):
                raise config.error(
                    f"motors_sync: Not supported "
                    f"'{len(belt_steppers)}' count of motors")
            axis.add_steppers(*belt_steppers, belt_steppers[1], None)

    def axes_sync(self, axes):
        # To skip extra measure_deviation() in axis_sync_step()
        axes[0].detect_move_dir()
        for m in axes:
            while True:
                if m.is_finished:
                    if all(m.is_finished for m in axes):
                        return
                    break
                self.axis_sync_step(m)

    def calibrate_axis_steps_model(self, axis, peak_mstep, repeats):
        return self.axis_calibrate_cycle(axis, peak_mstep, repeats)

    def get_linked_calibration_axes(self, axis):
        return sorted({axis.name for axis in self.motion_axes.values()})


# AWD corexy kinematics implementation
class CoreXYKinematics(BaseKinematics):
    def __init__(self, config, sync):
        super().__init__(config, sync)

    @staticmethod
    def check_common_attr(config, axes, common_attr):
        # Apply restrictions for interconnected axes kinematics
        for attr in common_attr:
            diff = {getattr(cls, attr) for cls in axes}
            if len(diff) != 1:
                params_str = ', '.join(f"'{attr}: {v}'" for v in diff)
                raise config.error(
                    f"motors_sync: Options {params_str} cannot "
                    f"be different for this kinematics")

    def _init_axes(self, config, sync):
        valid_axes = ['x', 'y']
        axes = sorted([a.lower() for a in config.getlist(
            'axes', count=len(valid_axes), default=valid_axes)])
        if any(a not in valid_axes for a in axes):
            raise config.error(f"motors_sync: Invalid axes '{axes}'")
        sync_pos = CartesianKinematics.get_axes_rails_center(config, axes)
        ph_offs = self.stats_helper.get_axes_phase_offsets(axes)
        self.motion_axes.update(
            {ax: MotionAxis(config, sync, ax, axes, ph_offs.get(ax),
                'stepper_' + ax, sync_pos, False) for ax in axes})
        attr = ['microsteps', 'model_name', 'model_coeffs',
                'max_step_size', 'axes_steps_diff']
        self.check_common_attr(config, self.motion_axes.values(), attr)

    def _init_axes_steppers(self, config):
        axes_alloc_steppers = []
        for axis in self.motion_axes.values():
            belt_steppers = [s for s in self.toolhead_kin.get_steppers()
                             if 'stepper_' + axis.name in s.get_name()]
            if len(belt_steppers) not in (2,):
                raise config.error(
                    f"motors_sync: Not supported "
                    f"'{len(belt_steppers)}' count of motors")
            axes_alloc_steppers.append([axis, belt_steppers])
        # Add 1 motor from the opposite axis to conflict motors on each
        # axis. Two enabled motors on the same axis (belt) can twist
        # the beam due to belt tension desync, distorting measurements.
        dx, dy = axes_alloc_steppers
        dx[0].add_steppers(dx[1][0], dx[1][1], dx[1][1], [dy[1][0]])
        dy[0].add_steppers(dy[1][0], dy[1][1], dy[1][1], [dx[1][0]])

    def check_axis_drift(self, m, s):
        steps_diff = abs(abs(m.drift_msteps) - abs(s.drift_msteps))
        if steps_diff >= m.axes_steps_diff:
            m.new_magnitude = m.magnitude = m.measure_deviation()
            self.msg_helper.static_msg(m)
            m.reset_drift_msteps()
            s.reset_drift_msteps()
            return True

    def axes_sync(self, axes):
        cycling = itertools.cycle(axes)
        max_ax = [c for c in sorted(
            axes, key=lambda i: i.init_magnitude)][-1]
        # To skip extra measure_deviation() in axis_sync_step()
        max_ax.detect_move_dir()
        # "m" is a main, "s" is a second MotionAxis
        while True:
            m = next(cycling)
            cycling, _cycle = itertools.tee(cycling)
            s = next(_cycle)
            if m.is_finished:
                if all(m.is_finished for m in axes):
                    break
                continue
            if m.magnitude < s.magnitude and not s.is_finished:
                self.check_axis_drift(m, s)
                continue
            self.axis_sync_step(m)

    def calibrate_axis_steps_model(self, axis, peak_mstep, repeats):
        return self.axis_calibrate_cycle(axis, peak_mstep, repeats)

    def get_linked_calibration_axes(self, axis):
        return sorted({axis.name for axis in self.motion_axes.values()})


# AWD linear delta kinematics implementation
class DeltaKinematics(BaseKinematics):
    def __init__(self, config, sync):
        super().__init__(config, sync)

    @staticmethod
    def get_arms_closest_perpend_pos(config, axes):
        positions = {}
        pt_sec = config.getsection('printer')
        z_min = pt_sec.getfloat('minimum_z_position', 0)
        delta_r = pt_sec.getfloat('delta_radius', above=0.)
        print_r = pt_sec.getfloat('print_radius', delta_r, above=0.)
        def_angles = {'a': 210., 'b': 330., 'c': 90.}
        for axis in axes:
            st_sec = config.getsection('stepper_' + axis)
            angle = st_sec.getfloat('angle', def_angles[axis])
            x_pos, y_pos = (np.cos(np.radians(angle)) * print_r,
                            np.sin(np.radians(angle)) * print_r)
            positions.update({axis: [x_pos, y_pos, z_min + 10]})
        return positions

    def _init_axes(self, config, sync):
        valid_axes = ['a', 'b', 'c']
        axes = sorted([a.lower() for a in config.getlist('axes')])
        if any(a not in valid_axes for a in axes):
            raise config.error(f"motors_sync: Invalid axes '{axes}'")
        sync_pos = self.get_arms_closest_perpend_pos(config, axes)
        ph_offs = self.stats_helper.get_axes_phase_offsets(axes)
        self.motion_axes.update(
            {ax: MotionAxis(config, sync, ax, 'xyz', ph_offs.get(ax),
                'stepper_' + ax, sync_pos[ax], False) for ax in axes})

    def _init_axes_steppers(self, config):
        for axis in self.motion_axes.values():
            belt_steppers = [s for s in self.toolhead_kin.get_steppers()
                             if 'stepper_' + axis.name in s.get_name()]
            if len(belt_steppers) not in (2,):
                raise config.error(
                    f"motors_sync: Not supported "
                    f"'{len(belt_steppers)}' count of motors")
            axis.add_steppers(*belt_steppers, belt_steppers[1], None)

    def axes_sync(self, axes):
        # To skip extra measure_deviation() in axis_sync_step()
        axes[0].detect_move_dir()
        for m in axes:
            while True:
                if m.is_finished:
                    if all(m.is_finished for m in axes):
                        return
                    break
                self.axis_sync_step(m)

    def calibrate_axis_steps_model(self, axis, peak_mstep, repeats):
        return self.axis_calibrate_cycle(axis, peak_mstep, repeats)

    def get_linked_calibration_axes(self, axis):
        return sorted({axis.name for axis in self.motion_axes.values()})


# Parse kinematics from configfile and choosing from supported ones.
class KinematicsParser:
    @staticmethod
    def get_kinematics(config, sync):
        kin_map = {cls.__name__.replace('Kinematics', '').lower(): cls
                   for cls in BaseKinematics.__subclasses__()}
        kin_map.update({'limitedcorexy': CoreXYKinematics})
        printer_section = config.getsection('printer')
        conf_kin = printer_section.get('kinematics')
        kin_helper = kin_map.get("".join(conf_kin.split("_")))
        if kin_helper is None:
            raise config.error(f"motors_sync: Not supported "
                               f"'{conf_kin}' kinematics")
        return kin_helper(config, sync)


# Plug class for the Klipper version before add "motion_queuing"
# module, and probably also for Kalico. May the gods forgive me.
class DummyPrinterMotionQueuing:
    def __init__(self, config):
        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def allocate_trapq(self):
        return self.trapq

    def lookup_trapq_append(self):
        return self.trapq_append

    def note_mcu_movequeue_activity(self, ptime):
        ctime = ptime + 99999.9
        self.trapq_finalize_moves(self.trapq, ctime, ctime)
        self.toolhead.note_mcu_movequeue_activity(ptime)

    def wipe_trapq(self, trapq):
        return


# MCU_stepper manual enables and manual moves outside normal
# kinematics, with extra allocated stepper kinematics and trapq.
class StepperManualMove:
    from . import force_move
    calc_move_time = staticmethod(force_move.calc_move_time)
    del force_move
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.stepper_en = printer.load_object(config, 'stepper_enable')
        self.motion_queuing = \
            printer.load_object(config, 'motion_queuing', None)
        if self.motion_queuing is None:
            self.motion_queuing = DummyPrinterMotionQueuing(config)
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.stepper_kin = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = min(self.toolhead.max_velocity, 100)
        self.travel_accel = min(self.toolhead.max_accel, 5000)

    def steppers_enable(self, mcu_steppers, mode):
        ptime = self.toolhead.get_last_move_time()
        did_change = False
        for mcu_stepper in mcu_steppers:
            el = self.stepper_en.lookup_enable(mcu_stepper.get_name())
            if el.is_motor_enabled() == mode:
                continue
            if mode:
                el.motor_enable(ptime)
            else:
                el.motor_disable(ptime)
            did_change = True
        return did_change

    def manual_move(self, mcu_stepper, moves):
        self.toolhead.flush_step_generation()
        prev_sk = mcu_stepper.set_stepper_kinematics(self.stepper_kin)
        prev_trapq = mcu_stepper.set_trapq(self.trapq)
        mcu_stepper.set_position((0., 0., 0.))
        ptime = start_ptime = self.toolhead.get_last_move_time()
        last_pos = 0.0
        for move in moves:
            if abs(move) < 0.00001:
                continue
            axis_r, accel_t, cruise_t, cruise_v = self.calc_move_time(
                move, self.travel_speed, self.travel_accel)
            self.trapq_append(
                self.trapq, ptime, accel_t, cruise_t, accel_t, last_pos,
                0., 0., axis_r, 0., 0., 0., cruise_v, self.travel_accel)
            ptime = ptime + accel_t + cruise_t + accel_t
            last_pos += move
        if hasattr(mcu_stepper, 'generate_steps'):
            mcu_stepper.generate_steps(ptime)
        self.motion_queuing.note_mcu_movequeue_activity(ptime)
        self.toolhead.dwell(ptime - start_ptime)
        self.toolhead.flush_step_generation()
        mcu_stepper.set_trapq(prev_trapq)
        mcu_stepper.set_stepper_kinematics(prev_sk)
        self.motion_queuing.wipe_trapq(self.trapq)


# Base helper class for bulk sensors readings that measure the degree
# of desynchronization of stepper motors in virtual-axes.
class BaseSensorHelper:
    def __init__(self, axis, chip_name):
        self.axis = axis
        self.chip_name = chip_name
        self.dim_type = ''
        self.msg_helper = axis.msg_helper
        self.printer = axis.printer
        self.stepper_move = axis.stepper_move
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.chip_config = None
        self.is_finished = False
        self.samples = []
        self.request_start_time = None
        self.request_end_time = None
        self._run_on_connect(self._handle_connect)
        self._run_on_connect(self._init_chip_config)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def _init_chip_config(self):
        self.chip_config = self.printer.lookup_object(self.chip_name)

    def _run_on_connect(self, task):
        _, state = self.printer.get_state_message()
        if state == "ready":
            task()
        else:
            self.printer.register_event_handler("klippy:connect", task)

    def get_name(self):
        return self.chip_name

    def flush_data(self):
        self.is_finished = False
        self.samples.clear()
        self.request_start_time = None
        self.request_end_time = None

    def _handle_batch(self, batch):
        if self.is_finished:
            return False
        samples = batch['data']
        self.samples.extend(samples)
        return True

    def update_start_time(self):
        self.request_start_time = self.toolhead.get_last_move_time()

    def update_end_time(self):
        self.request_end_time = self.toolhead.get_last_move_time()

    def start_measurements(self):
        self.flush_data()
        self.chip_config.batch_bulk.add_client(self._handle_batch)

    def finish_measurements(self):
        self.toolhead.wait_moves()
        self.is_finished = True

    def _wait_samples(self):
        now = self.reactor.monotonic()
        est_ptime = self.chip_config.mcu.estimated_print_time(now)
        wait_limit = now + self.request_end_time - est_ptime + 1.
        while True:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            if self.samples:
                last_mcu_time = self.samples[-1][0]
                if last_mcu_time >= self.request_end_time:
                    return True
            if now > wait_limit:
                raise Exception(f"No data from '{self.chip_name}'")

    def _get_samples(self):
        self._wait_samples()
        raw_data = np.array(self.samples)
        start_idx = np.searchsorted(raw_data[:, 0],
                    self.request_start_time, side='left')
        end_idx = np.searchsorted(raw_data[:, 0],
                    self.request_end_time, side='right')
        return raw_data[start_idx:end_idx][:, 1:]

    def measure_deviation(self):
        # Measure the impact
        self.axis.buzz_move(25)
        self.flush_data()
        self.axis.toggle_main_stepper(1, (PIN_MIN_TIME,))
        self.axis.toggle_main_stepper(0, (PIN_MIN_TIME,))
        self.update_start_time()
        self.axis.toggle_main_stepper(1)
        self.update_end_time()
        self.axis.buzz_move(5)
        dev = self.calc_deviation()
        self.axis.update_log(dev)
        return dev

    def calc_deviation(self):
        raise NotImplementedError("Internal error in motors_sync")

    def detect_move_dir(self):
        raise NotImplementedError("Internal error in motors_sync")


# Accelerometer-based sensor helper calculating acceleration values.
class AccelHelper(BaseSensorHelper):
    ACCEL_FILTER_THRESHOLD = 3000
    def __init__(self, axis, chip_name, config=None, cfp=None):
        self.chip_filter_params = cfp
        self.chip_filter = None
        super().__init__(axis, chip_name)
        self.dim_type = 'magnitude'
        if config is None and cfp is None:
            raise Exception("Internal error in motors_sync")
        if config is not None and cfp is None:
            self._read_config_chip_filter(config)
        self._get_axes_mask = None
        self._init_axes_mask()

    def get_chip_filter_params(self):
        return self.chip_filter_params

    def _read_config_chip_filter(self, config):
        filters = {m: m for m in ['default', 'median', 'kalman']}
        filter = config.getchoice(f'chip_filter_{self.axis.name}',
                                  filters, default='default').lower()
        if filter == 'default':
            filter = config.getchoice('chip_filter', filters,
                                      default='median').lower()
        if filter == 'median':
            window = config.getint(f'median_size_{self.axis.name}',
                                   default=None, minval=3, maxval=9)
            if not window:
                window = config.getint('median_size', default=3,
                                       minval=3, maxval=9)
            if window % 2 == 0:
                raise config.error(f"motors_sync: parameter "
                                   f"'median_size' cannot be even")
            self.chip_filter_params = [filter, window]
        elif filter == 'kalman':
            coeffs = config.getfloatlist(
                f'kalman_coeffs_{self.axis.name}',
                default=tuple(None for _ in range(6)), count=6)
            if not all(coeffs):
                default_coeffs = tuple((1.1, 1., 1e-1, 1e-2, .5, 1.))
                coeffs = config.getfloatlist(
                    'kalman_coeffs', default=default_coeffs, count=6)
            self.chip_filter_params = [filter, coeffs]

    def _get_chip_filter(self):
        filter_name, params = self.chip_filter_params
        if filter_name == 'median':
            w = params
            def median_filter(samples):
                return np.median([samples[i - w:i + w + 1]
                    for i in range(w, len(samples) - w)], axis=1)
            return median_filter
        elif filter_name == 'kalman':
            return KalmanLiteFilter(*params).process_samples

    def _init_chip_config(self):
        self.chip_config = self.printer.lookup_object(self.chip_name)
        if self.chip_config.data_rate > self.ACCEL_FILTER_THRESHOLD:
            self.chip_filter = self._get_chip_filter()
        else:
            # Do not use a filter on low samples rate as it will
            # smooth out the already barely noticeable peaks.
            self.chip_filter = lambda data: data

    def _init_axes_mask(self):
        if 'z' in self.axis.get_physical_axes():
            def mask(static_data):
                return np.ones(static_data.shape[1], dtype=bool)
        else:
            def mask(static_data):
                z_axis = np.mean(np.abs(static_data), axis=0).argmax()
                return np.arange(static_data.shape[1]) != z_axis
        self._get_axes_mask = mask

    def calc_deviation(self):
        # Calculate impact magnitude
        vects = self._get_samples()
        vects_len = vects.shape[0]
        # Kalman filter may distort the first values, or in some
        # cases there may be residual values of toolhead inertia.
        # It is better to take a shifted zone from zero.
        static_zone = range(vects_len // 5, vects_len // 3)
        axes_mask = self._get_axes_mask(vects[static_zone, :])
        magnitudes = np.linalg.norm(vects[:, axes_mask], axis=1)
        # Add median, Kalman or none filter
        magnitudes = self.chip_filter(magnitudes)
        # Calculate static noise
        static = np.mean(magnitudes[static_zone])
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitudes)[-5:])
        magnitude = np.around(magnitude - static, 2)
        return magnitude

    def detect_move_dir(self):
        # Determine axis movement direction
        self.axis.set_move_dir(0)
        self.axis.step_move()
        self.axis.new_magnitude = self.measure_deviation()
        self.msg_helper.stepped_msg(self.axis)
        if self.axis.new_magnitude > self.axis.magnitude:
            self.axis.set_move_dir(-1)
        else:
            self.axis.set_move_dir(1)
        self.msg_helper.direction_msg(self.axis)
        self.axis.magnitude = self.axis.new_magnitude


# Beacon accelerometer helper adapter with custom interface.
class BeaconAccelHelper(AccelHelper):
    def __init__(self, axis, chip_name, config=None, cfp=None):
        super().__init__(axis, chip_name, config, cfp)

    def _init_beacon_config(self):
        class BeaconAccelConfigAdapter:
            def __init__(self, beacon):
                self.accel_helper = beacon.accel_helper
                # Beacon use "batch_bulk" named as "_api_dump"
                self.batch_bulk = beacon.accel_helper._api_dump
                self.mcu = beacon._mcu
            def __getattr__(self, name):
                return getattr(self.accel_helper, name)
        beacon = self.printer.lookup_object(self.chip_name)
        if beacon.accel_helper is None:
            msg = "This Beacon has no accelerometer"
            raise self.printer.config_error(msg)
        self.chip_config = BeaconAccelConfigAdapter(beacon)
        # Beacon module doesn't have "data_rate" attribute
        # Beacon adxl345 sampling rate > ACCEL_FILTER_THRESHOLD
        self.chip_filter = self._get_chip_filter()

    def _init_chip_config(self):
        # Move to the end of the klippy:connect queue due beacon
        # create BeaconAccelHelper in the klippy:connect stage
        self._run_on_connect(self._init_beacon_config)

    def _handle_batch(self, batches):
        # Beacon "_api_dump" send data in an additional array
        hb = super()._handle_batch
        for batch in batches:
            res = hb(batch)
        return res


# Encoder-based sensor helper calculating stepper positional values.
class EncoderHelper(BaseSensorHelper):
    UNITS_SCALE = 1e3
    MIN_SAMPLE_PERIOD = 0.000400
    def __init__(self, axis, chip_name):
        super().__init__(axis, chip_name)
        self.dim_type = 'deviation'
        self.last_raw_deviation = 0
        self._check_retry_tolerance()

    def _init_chip_config(self):
        self.chip_config = self.printer.lookup_object(self.chip_name)
        self._check_sample_rate()
        self._check_encoder_place()

    def _check_retry_tolerance(self):
        step_dist = self.axis.move_d
        half_step_dist_um = np.ceil(step_dist / 2 * self.UNITS_SCALE)
        retry_tolerance = self.axis.retry_tolerance
        if retry_tolerance < half_step_dist_um:
            raise self.printer.config_error(
                f"motors_sync: Parameter 'retry_tolerance' cannot "
                f"be less than microsteps half microstep distance, "
                f"{retry_tolerance} < {half_step_dist_um} (µm)")

    def _check_sample_rate(self):
        per = self.chip_config.sample_period
        if per > self.MIN_SAMPLE_PERIOD:
            raise self.printer.config_error(
                f'motors_sync: Encoder sample rate too '
                f'low: {per} < {self.MIN_SAMPLE_PERIOD}')

    def _check_encoder_place(self):
        # Swap duties between motors depending on encoder place
        binded_stepper = self.chip_config.calibration.stepper_name
        sts = self.axis.get_steppers()
        axis_steppers = [s.get_name() for s in sts['self_steppers']]
        if binded_stepper not in axis_steppers:
            raise self.printer.config_error(
                f"motors_sync: Encoder '{self.chip_name}' stepper "
                f"'{binded_stepper}' not in '{self.axis.name}' "
                f"axis steppers: '{', '.join(axis_steppers)}'")
        if binded_stepper != sts['enable_stepper'].get_name():
            if self.axis.swap_steppers(binded_stepper) is False:
                raise self.printer.config_error(
                    f"motors_sync: Encoder '{self.chip_name}' must "
                    f"be on another stepper in axis steppers: "
                    f"'{', '.join(axis_steppers)}")

    def _normalize_encoder_pos(self, pos):
        angle = (1 << 16) / pos
        length = self.axis.rd / angle
        return angle, length

    def calc_deviation(self):
        # Calculate impact position on encoder in µm
        positions = self._get_samples()[:, 0]
        poss_len = positions.shape[0]
        static_zone = range(poss_len // 5, poss_len // 3)
        # Calculate static position
        static = np.mean(positions[static_zone])
        # Return avg of 5 max positions with deduction static
        deviations = positions - static
        top_dev_ids = np.argsort(np.abs(deviations))[-5:]
        deviation = np.mean(deviations[top_dev_ids])
        dev_norm = self._normalize_encoder_pos(deviation)
        deviation = np.around(dev_norm[1] * self.UNITS_SCALE, 2)
        abs_deviation = abs(deviation)
        self.last_raw_deviation = deviation
        return abs_deviation

    def detect_move_dir(self):
        # Determine axis movement direction
        if self.last_raw_deviation < 0:
            self.axis.set_move_dir(-1)
        else:
            self.axis.set_move_dir(1)
        self.msg_helper.direction_msg(self.axis)
        self.axis.new_magnitude = self.axis.magnitude


# Base fan controller class for managing fans during sync process.
# Temporarily disables and restores fan operation, prevents self
# vibrations from affecting sensor measurements.
class FanController:
    def __init__(self, fan, printer):
        self.fan = fan
        self.printer = printer
        self.is_hold = False

    def _hold(self):
        raise Exception("Internal error in motors_sync")

    def _resume(self):
        raise Exception("Internal error in motors_sync")

    def set_state(self, state):
        if not state:
            if self.is_hold:
                return
            self.is_hold = True
            self._hold()
        else:
            if not self.is_hold:
                return
            self.is_hold = False
            self._resume()

    def _set_fan_speed(self, value):
        now = self.printer.get_reactor().monotonic()
        print_time = self.fan.fan.get_mcu().estimated_print_time(now)
        self.fan.fan.set_speed(value=value,
            print_time=print_time + PIN_MIN_TIME)


class ControllerFanController(FanController):
    def __init__(self, fan, printer):
        super().__init__(fan, printer)
        self.fan_speed = None
        self.idle_speed = None
        self.last_speed = None

    def _hold(self):
        self.fan_speed = self.fan.fan_speed
        self.idle_speed = self.fan.idle_speed
        self.last_speed = self.fan.last_speed
        self.fan.fan_speed = 0
        self.fan.idle_speed = 0
        self.fan.last_speed = 0
        self._set_fan_speed(0)

    def _resume(self):
        self.fan.fan_speed = self.fan_speed
        self.fan.idle_speed = self.idle_speed
        self.fan.last_speed = self.last_speed
        self._set_fan_speed(self.last_speed)
        self.fan_speed = None
        self.idle_speed = None
        self.last_speed = None


class HeaterFanController(FanController):
    def __init__(self, fan, printer):
        super().__init__(fan, printer)
        self.fan_speed = fan.fan_speed

    def _hold(self):
        self.fan.fan_speed = self.fan.last_speed = 0.0
        self._set_fan_speed(0.0)

    def _resume(self):
        self.fan.fan_speed = self.fan.last_speed = self.fan_speed
        self._set_fan_speed(self.fan_speed)


class TemperatureFanController(FanController):
    def __init__(self, fan, printer):
        super().__init__(fan, printer)
        self.target_temp = None
        self.min_speed = None
        self.max_speed = None

    def _hold(self):
        _, self.target_temp = self.fan.get_temp(0)
        self.min_speed = self.fan.get_min_speed()
        self.max_speed = self.fan.get_max_speed()
        self.fan.set_temp(0)
        self.fan.set_min_speed(0)
        self.fan.set_max_speed(0)
        self._set_fan_speed(0)

    def _resume(self):
        self.fan.set_temp(self.target_temp)
        self.fan.set_min_speed(self.min_speed)
        self.fan.set_max_speed(self.max_speed)
        self.target_temp = None
        self.min_speed = None
        self.max_speed = None


# Proxy to toggle fans per virtual-axis. Delegates requests to FanManager.
class FanProxy:
    def __init__(self, manager, axis_name, fan_names):
        self._manager = manager
        self._axis_name = axis_name
        self._fan_names = fan_names

    def toggle(self, state):
        for name in self._fan_names:
            self._manager._request_toggle(name, self._axis_name, state)


# Registers and manages all user configured fans. Tracks per-v-axis fans
# disable requests, ensures fans resume only when all v-axes allow it.
class FanManager:
    from . import controller_fan, heater_fan, temperature_fan
    FAN_METHODS = {
        controller_fan.ControllerFan: ControllerFanController,
        heater_fan.PrinterHeaterFan: HeaterFanController,
        temperature_fan.TemperatureFan: TemperatureFanController,
    }
    del controller_fan, heater_fan, temperature_fan
    def __init__(self, config):
        self.printer = config.get_printer()
        self._fans = {}
        self._fan_off_requests = {}

    def register_fans(self, axis_name, fan_names):
        for name in fan_names:
            if name in self._fans:
                continue
            self._register_fan(name)
        return FanProxy(self, axis_name, fan_names)

    def _register_fan(self, fan_name):
        try:
            fan_obj = self.printer.lookup_object(fan_name)
        except:
            if len(fan_name.split()) != 1:
                raise self.printer.config_error(
                    f"motors_sync: Unknown fan '{fan_name}'")
            raise self.printer.config_error(
                f"motors_sync: Provide full fan '{fan_name}' name "
                f"in 'head_fan' option, e.g. 'heater_fan {fan_name}'")
        controller_cls = None
        for fan_type, ctrl_type in self.FAN_METHODS.items():
            if isinstance(fan_obj, fan_type):
                controller_cls = ctrl_type
                break
        if not controller_cls:
            raise self.printer.config_error(
                f"motors_sync: Unknown fan type '{fan_name}' "
                f"--> '{type(fan_obj).__name__}' object")
        self._fans[fan_name] = controller_cls(fan_obj, self.printer)
        self._fan_off_requests[fan_name] = set()

    def _request_toggle(self, fan_name, axis_id, state):
        off_set = self._fan_off_requests[fan_name]
        if state:
            off_set.discard(axis_id)
        else:
            off_set.add(axis_id)
        controller = self._fans[fan_name]
        should_be_on = len(off_set) == 0
        controller.set_state(should_be_on)


# Helper to format MotionAxis state and send to user-facing messages
# of synchronization, calibration process stage.
class MotionAxisMsgHelper:
    def __init__(self, config):
        self.gcode = config.get_printer().lookup_object('gcode')

    def start_msg(self, ax):
        msg = (f"{ax.name_prefixed}-Initial "
               f"{ax.chip_helper.dim_type}: {ax.init_magnitude}")
        self.gcode.respond_info(msg)

    def done_msg(self, ax):
        msg = (f"{ax.name_prefixed}-Motors adjusted by "
               f"{ax.actual_msteps}/{ax.microsteps} step, "
               f"{ax.chip_helper.dim_type} "
               f"{ax.init_magnitude} --> {ax.magnitude}")
        self.gcode.respond_info(msg)

    def in_tolerance_msg(self, axes):
        retry_tols = ", ".join([
            f"{ax.name_prefixed}: {ax.retry_tolerance}"
            for ax in axes])
        msg = f"Motors magnitudes are in tolerance: {retry_tols}"
        self.gcode.respond_info(msg)

    def static_msg(self, ax):
        msg = (f"{ax.name_prefixed}-New {ax.chip_helper.dim_type}: "
               f"{ax.new_magnitude}")
        self.gcode.respond_info(msg)

    def stepped_msg(self, ax):
        msteps = ax.move_msteps * ax.move_dir[0]
        msg = (f"{ax.name_prefixed}-New {ax.chip_helper.dim_type}: "
               f"{ax.new_magnitude} on {msteps}/{ax.microsteps} "
               f"step move")
        self.gcode.respond_info(msg)

    def direction_msg(self, ax):
        msg = f"{ax.name_prefixed}-Movement direction: {ax.move_dir[1]}"
        self.gcode.respond_info(msg)

    def retry_msg(self, ax):
        msg = (f"{ax.name_prefixed}-Retries: {ax.curr_retry}/"
               f"{ax.max_retries} Back to last "
               f"{ax.chip_helper.dim_type}: {ax.magnitude} on "
               f"{ax.actual_msteps}/{ax.microsteps} step "
               f"to reach {ax.retry_tolerance}")
        self.gcode.respond_info(msg)


# Virtual-axis as helper to control and synchronize assigned stepper
# to another stepper on one belt or to the secondary/overall kinematics.
class MotionAxis:
    VALID_MSTEPS = [256, 128, 64, 32, 16, 8, 0]
    def __init__(self, config, sync, name, ph_axes, ph_off,
                 main_stepper, sync_pos, is_multi_axis):
        self.name = name
        self.physical_axes = ph_axes
        self.phase_offset = ph_off
        self.start_sync_pos = sync_pos
        self.is_multi_axis = is_multi_axis
        self.stepper_move = sync.stepper_move
        self.stats_helper = sync.stats_helper
        self.msg_helper = sync.msg_helper
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        sync.add_connect_task(self._handle_connect)
        self.move_dir = []
        self.set_move_dir(0)
        self.move_msteps = 2
        self.actual_msteps = 0
        self.drift_msteps = 0
        self.init_magnitude = 0.
        self.magnitude = 0.
        self.new_magnitude = 0.
        self.curr_retry = 0
        self.is_finished = False
        self.motion_log = []
        self.chip_helper = None
        self.steppers = None
        st_section = config.getsection(main_stepper)
        self.rd = st_section.getfloat('rotation_distance')
        fspr = st_section.getint('full_steps_per_rotation', 200)
        self.buzz_moves = {}
        self.rel_buzz_d = self.rd / fspr * 5
        if self.start_sync_pos[2] is not None:
            self.start_sync_pos[2] += self.rel_buzz_d
        msteps_dict = {m: m for m in self.VALID_MSTEPS}
        self.microsteps = config.getchoice(
            f'microsteps_{name}', msteps_dict, default=0)
        if not self.microsteps:
            self.microsteps = config.getchoice(
                'microsteps', msteps_dict, default=16)
        self.move_d = self.rd / fspr / self.microsteps
        conf_fans = config.getlist(f'head_fan_{name}', '')
        if not conf_fans:
            conf_fans = config.getlist('head_fan', [])
        sync.add_connect_task(lambda: self._init_fan(sync, conf_fans))
        msmax = self.microsteps / 2
        self.max_step_size = config.getint(
            f'max_step_size_{name}', default=0, minval=1, maxval=msmax)
        if not self.max_step_size:
            self.max_step_size = config.getint(
                'max_step_size', default=3, minval=1, maxval=msmax)
        self.axes_steps_diff = config.getint(
            f'axes_steps_diff_{name}', default=0, minval=1)
        if not self.axes_steps_diff:
            self.axes_steps_diff = config.getint(
                'axes_steps_diff', self.max_step_size + 1, minval=1)
        self.retry_tolerance = config.getfloat(
            f'retry_tolerance_{name}', default=0, above=0.)
        if not self.retry_tolerance:
            self.retry_tolerance = config.getfloat(
                'retry_tolerance', default=0, above=0.)
        self.max_retries = config.getint(
            f'retries_{name}', default=0, minval=0, maxval=10)
        if not self.max_retries:
            self.max_retries = config.getint(
                'retries', default=0, minval=0, maxval=10)
        self.name_prefixed = name.upper()
        if name_prefix := config.get(f'axis_prefix_{name}', None):
            self.name_prefixed = f'{name_prefix} {self.name_prefixed}'
        self._init_chip_helper(config)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2

    def _init_steps_model(self, config, def_model):
        def poly(fx, cf):
            return max(np.roots([*cf[:-1], cf[-1] - fx]).real)
        math_models = {
            "linear": {"count": 2, "a_forbidden": None,
                "f": lambda fx, c: poly(fx, c)},
            "quadratic": {"count": 3, "a_forbidden": None,
                "f": lambda fx, c: poly(fx, c)},
            "power": {"count": 2, "a_forbidden": None,
                "f": lambda fx, c: (fx / c[0]) ** (1 / c[1])},
            "root": {"count": 2, "a_forbidden": 0,
                "f": lambda fx, c: (fx**2-2*c[1]*fx+c[1]**2)/c[0]**2},
            "hyperbolic": {"count": 2, "a_forbidden": 0,
                "f": lambda fx, c: c[0] / (fx - c[1])},
            "exponential": {"count": 3, "a_forbidden": 0,
                "f": lambda fx, c: np.log((fx - c[2]) / c[0]) / c[1]},
            "enc_auto": {"count": 1, "a_forbidden": 0, "scale": 1,
                "f": lambda fx, c: fx / EncoderHelper.UNITS_SCALE / c[0]}}
        model = config.getlist(f'steps_model_{self.name}', None)
        if model is None:
            model = config.getlist('steps_model', def_model)
        if len(model) < 2:
            raise config.error(f"motors_sync: Invalid steps "
                               f"model or its coeffs")
        model_name = model[0]
        model_coeffs = tuple(map(float, model[1:]))
        model_config = math_models.get(model_name, None)
        if (model_config is None
                or len(model_coeffs) != model_config['count']
                or model_coeffs[0] == model_config['a_forbidden']):
            raise config.error(f"motors_sync: Invalid steps "
                               f"model or its coeffs")
        # Calibration and description of the steps model is performed
        # with 1/16 step, except encoder "enc_auto" plug model
        model_scale = model_config.get("scale", self.microsteps / 16)
        def model_solve(fx):
            res = model_config['f'](fx, model_coeffs)
            if np.isnan(res):
                raise Exception(
                    f"Microsteps calculation returned NaN "
                    f"for {self.name_prefixed} axis")
            return res * model_scale
        self.steps_model_solve = model_solve
        self.model_name = model_name
        self.model_coeffs = model_coeffs

    def init_accel_chip_helper(self, accel_chip_name, config=None):
        filter_params = None
        if isinstance(self.chip_helper, AccelHelper):
            self.chip_helper.finish_measurements()
            filter_params = self.chip_helper.get_chip_filter_params()
        if accel_chip_name == 'beacon':
            self.chip_helper = BeaconAccelHelper(self,
                accel_chip_name, config=config, cfp=filter_params)
        else:
            self.chip_helper = AccelHelper(self,
                accel_chip_name, config=config, cfp=filter_params)

    def _init_chip_helper(self, config):
        accel_chip_name = config.get(f'accel_chip_{self.name}', None)
        enc_chip_name = config.get(f'encoder_chip_{self.name}', None)
        if accel_chip_name and enc_chip_name:
            raise config.error(f"motors_sync: Only 1 sensor "
                               f"type can be selected")
        if not accel_chip_name and not enc_chip_name:
            accel_chip_name = config.get('accel_chip', None)
        if not accel_chip_name and not enc_chip_name:
            raise config.error(
                f"motors_sync: Sensors type 'accel_chip' or "
                f"'encoder_chip_<axis>' must be provided")
        if accel_chip_name:
            self.init_accel_chip_helper(accel_chip_name, config)
            def_steps_model = ['linear', 10000, 0]
        elif enc_chip_name:
            self.chip_helper = EncoderHelper(self, enc_chip_name)
            def_steps_model = ['enc_auto', self.move_d]
        self._init_steps_model(config, def_steps_model)

    def _init_fan(self, sync, fans):
        self.fan = sync.fan_manager.register_fans(self.name, fans)

    def _get_tmc_drivers(self, mcu_steppers):
        for mcu_stepper in mcu_steppers:
            for driver in TRINAMIC_DRIVERS:
                driver_name = f"{driver} {mcu_stepper.get_name()}"
                module = self.printer.lookup_object(driver_name, None)
                if module is not None:
                    yield module
                    break
            else:
                raise self.printer.config_error(
                    f"motors_sync: Unable to find TMC driver "
                    f"for '{mcu_stepper.get_name()}' stepper")

    def _validate_steppers(self, mcu_steppers, tmcs):
        for stepper, tmc in zip(mcu_steppers, tmcs):
            st_msteps = 256 >> tmc.fields.get_field("mres")
            if self.microsteps > st_msteps:
                raise self.printer.config_error(
                    f'motors_sync: Invalid config microsteps '
                    f'count, cannot be more than in steppers '
                    f'config, {self.microsteps} > {st_msteps}')

    def add_steppers(self, enable, step, buzz, conflict):
        self_steppers = list(dict.fromkeys((enable, step, buzz)))
        if len(self_steppers) != 2:
            raise ValueError("Steppers allocation error")
        self_tmcs = list(self._get_tmc_drivers(self_steppers))
        self._validate_steppers(self_steppers, self_tmcs)
        self.steppers = {
            'self_steppers': self_steppers,
            'self_tmcs': self_tmcs,
            'enable_stepper': enable,
            'step_stepper': step,
            'buzz_stepper': buzz,
            'conflict_steppers': conflict,
        }

    def swap_steppers(self, en_stepper_name):
        if self.is_multi_axis:
            return False
        sts = self.steppers
        new_en_stepper = next(
            (st for st in sts['self_steppers']
             if st.get_name() == en_stepper_name), None)
        if new_en_stepper is None:
            return False
        old_en_stepper = sts['enable_stepper']
        if new_en_stepper == old_en_stepper:
            raise Exception("Swap same stepper")
        sts['enable_stepper'] = new_en_stepper
        for key in ('step_stepper', 'buzz_stepper'):
            curr = sts[key]
            if curr is new_en_stepper:
                sts[key] = old_en_stepper
            elif curr is old_en_stepper:
                sts[key] = new_en_stepper
        sts['self_steppers'].reverse()
        sts['self_tmcs'].reverse()
        return True

    def flush_motion_data(self):
        self.set_move_dir(0)
        self.move_msteps = 2
        self.actual_msteps = 0
        self.drift_msteps = 0
        self.init_magnitude = 0.
        self.magnitude = 0.
        self.new_magnitude = 0.
        self.curr_retry = 0
        self.is_finished = False
        self.motion_log = []

    def get_physical_axes(self):
        return self.physical_axes

    def get_steppers(self):
        return self.steppers

    def set_retry_tolerance(self, retry_tol):
        self.retry_tolerance = retry_tol

    def set_max_retries(self, max_retries):
        self.max_retries = max_retries

    def reset_drift_msteps(self):
        self.drift_msteps = 0

    def set_move_dir(self, dir):
        if dir == 1:
            self.move_dir = [1, 'Forward']
        elif dir == -1:
            self.move_dir = [-1, 'Backward']
        elif dir == 0:
            self.move_dir = [1, 'unknown']

    def is_move_dir_unknown(self):
        return True if self.move_dir[1] == 'unknown' else False

    def update_log(self, deviation):
        self.motion_log.append([int(deviation), self.actual_msteps])

    def _query_phase(self, mcu_stepper, mcu_tmc):
        field_name = "mscnt"
        if mcu_tmc.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "mstep"
        reg = mcu_tmc.fields.lookup_register(field_name)
        phase = mcu_tmc.mcu_tmc.get_register(reg)
        phase = mcu_tmc.fields.get_field(field_name, phase)
        if not mcu_stepper.get_dir_inverted()[0]:
            return (1024 - phase) % 1024
        return phase

    def query_phases_offset(self):
        if self.is_multi_axis:
            return None
        # If the steppers are inverted to each other, the offset
        # of the initial non-inverted phase position must be taken
        # into account to calculate the correct phases offset.
        def query_and_norm_phase(mcu_stepper, tmc):
            phase = self._query_phase(mcu_stepper, tmc)
            if mcu_stepper.get_dir_inverted()[0]:
                return phase
            offset = 1 << tmc.fields.get_field("mres")
            return (phase + offset) % 1024
        p1, p2 = (query_and_norm_phase(s, t)
                  for s, t in zip(self.steppers['self_steppers'],
                                  self.steppers['self_tmcs']))
        return ((p2 - p1 + 512) % 1024) - 512

    def apply_phase_offset(self):
        if self.phase_offset is None or self.is_multi_axis:
            return False
        if self.query_phases_offset() != 0:
            # Phases offset has already been changed by anyone
            return False
        move_msteps = self.phase_offset * self.microsteps // 256
        if abs(move_msteps) < 2:
            return False
        move_msteps1 = move_msteps // 2
        move_msteps2 = move_msteps - move_msteps1
        move_dist1 = move_msteps1 * self.move_d
        move_dist2 = move_msteps2 * self.move_d * -1
        en_mcu_stepper = self.steppers['enable_stepper']
        st_mcu_stepper = self.steppers['step_stepper']
        en1 = self.stepper_move.steppers_enable([en_mcu_stepper], 0)
        en2 = self.stepper_move.steppers_enable([st_mcu_stepper], 1)
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.stepper_move.manual_move(st_mcu_stepper, [move_dist1])
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.stepper_move.steppers_enable([st_mcu_stepper], 0)
        self.stepper_move.steppers_enable([en_mcu_stepper], 1)
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.stepper_move.manual_move(en_mcu_stepper, [move_dist2])
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.gcode.respond_info(
            f'{self.name_prefixed}-Restore previous '
            f'position: {move_msteps}/{self.microsteps} step')
        self.stepper_move.steppers_enable([en_mcu_stepper], en1)
        self.stepper_move.steppers_enable([st_mcu_stepper], not en2)
        self.toolhead.dwell(MOTOR_STALL_TIME)
        return True

    def toggle_main_stepper(self, mode, times=None):
        if times is None:
            times = (MOTOR_STALL_TIME, MOTOR_STALL_TIME)
        elif len(times) < 2:
            times = (times[0], MOTOR_STALL_TIME)
        mcu_stepper = self.steppers['enable_stepper']
        self.toolhead.dwell(times[0])
        self.stepper_move.steppers_enable([mcu_stepper], mode)
        self.toolhead.dwell(times[1])

    def toggle_self_steppers(self, mode):
        mcu_steppers = self.steppers['self_steppers']
        ret = self.stepper_move.steppers_enable(mcu_steppers, mode)
        if ret:
            self.toolhead.dwell(MOTOR_STALL_TIME)

    def toggle_conflict_steppers(self, mode):
        mcu_steppers = self.steppers['conflict_steppers']
        if mcu_steppers is None:
            return
        ret = self.stepper_move.steppers_enable(mcu_steppers, mode)
        if ret:
            self.toolhead.dwell(MOTOR_STALL_TIME)

    def move_on_measure_pos(self):
        curr_pos = self.toolhead.get_position()
        sync_pos = self.start_sync_pos
        poss = zip(curr_pos, sync_pos)
        if all((n is None) or (c == n) for c, n in poss):
            return
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.toolhead.manual_move(sync_pos, self.travel_speed)
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.toolhead.flush_step_generation()

    def manual_move(self, dist):
        self.toggle_conflict_steppers(0)
        mcu_stepper = self.steppers['buzz_stepper']
        self.stepper_move.manual_move(mcu_stepper, [dist])

    def calc_move_msteps(self, dev=None):
        dev = self.new_magnitude if dev is None else dev
        steps_to_zero = max(int(self.steps_model_solve(dev)), 1)
        self.move_msteps = min(steps_to_zero, self.max_step_size)

    def step_move(self, dir=1):
        mcu_stepper = self.steppers['step_stepper']
        move_msteps = self.move_msteps * self.move_dir[0] * dir
        dist = self.move_d * move_msteps
        self.actual_msteps += move_msteps
        self.drift_msteps += move_msteps
        self.stepper_move.manual_move(mcu_stepper, [dist])

    def _gen_buzz_moves(self, rel_moves):
        moves = []
        last_abs_pos = 0
        for osc in reversed(range(0, rel_moves)):
            abs_pos = self.rel_buzz_d * (osc / rel_moves)
            for inv in [1, -1]:
                abs_pos *= inv
                dist = (abs_pos - last_abs_pos)
                moves.append(dist)
                last_abs_pos = abs_pos
        self.buzz_moves[rel_moves] = moves
        return moves

    def buzz_move(self, rel_moves=25):
        # Fading oscillations by <buzz_stepper> stepper
        mcu_stepper = self.steppers['buzz_stepper']
        moves = self.buzz_moves.get(
            rel_moves, self._gen_buzz_moves(rel_moves))
        self.toggle_main_stepper(0, (PIN_MIN_TIME,)*2)
        self.stepper_move.manual_move(mcu_stepper, moves)

    def measure_deviation(self):
        self.move_on_measure_pos()
        self.toggle_conflict_steppers(0)
        return self.chip_helper.measure_deviation()

    def detect_move_dir(self):
        self.move_on_measure_pos()
        self.toggle_conflict_steppers(0)
        return self.chip_helper.detect_move_dir()

    def on_start(self):
        self.flush_motion_data()
        self.fan.toggle(False)
        self.chip_helper.start_measurements()

    def on_finish(self):
        self.is_finished = True
        self.fan.toggle(True)
        self.chip_helper.finish_measurements()

    def on_done(self):
        if not self.is_finished:
            self.on_finish()
        self.toggle_self_steppers(1)
        self.toggle_conflict_steppers(1)
        self.phase_offset = self.query_phases_offset()
        self.stats_helper.write_axis_stats_log(self, True)

    def on_error(self):
        self.fan.toggle(True)
        self.chip_helper.finish_measurements()
        self.toggle_self_steppers(1)
        self.toggle_conflict_steppers(1)
        self.stats_helper.write_axis_stats_log(self, False)


# Main entry point for motors synchronization feature. Initializes
# helpers, registers G-code commands, controls sync state status.
class MotorsSync:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.connect_tasks = []
        self.calibrate_steps_model_helper = None
        printer.register_event_handler("klippy:connect", self._handle_connect)
        # Init motors-sync helpers
        self.status = ZAdjustStatus(self.printer)
        self.stepper_move = StepperManualMove(config)
        self.fan_manager = FanManager(config)
        self.stats_helper = MotionAxesStats(config)
        self.msg_helper = MotionAxisMsgHelper(config)
        self.kin_helper = KinematicsParser.get_kinematics(config, self)
        # Register G-Code commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_SYNC_MOTORS,
                                    desc=self.cmd_SYNC_MOTORS_help)
        self.gcode.register_command('SYNC_MOTORS_CALIBRATE',
                                    self.cmd_SYNC_MOTORS_CALIBRATE,
                                    desc=self.cmd_SYNC_MOTORS_CALIBRATE_help)

    def add_connect_task(self, task):
        self.connect_tasks.append(task)

    def _handle_connect(self):
        for task in self.connect_tasks: task()
        self.connect_tasks.clear()

    cmd_SYNC_MOTORS_help = 'Start motors synchronization'
    def cmd_SYNC_MOTORS(self, gcmd):
        _axes = self.kin_helper.get_motion_axes()
        axes = list(_axes.values())
        if gcmd.get('RESTORE_SYNC', None) is not None:
            if not self.kin_helper.apply_axes_phase_offsets(axes):
                self.gcode.respond_info("Motors sync position already "
                                        "restored or nothing to restore")
            return
        # Override axes to sync from gcmd
        axes_names = gcmd.get('AXES', None)
        if axes_names is not None:
            axes_names = {a.lower() for a in axes_names.split(',')}
            if any(name not in _axes for name in axes_names):
                raise self.gcode.error(f'Invalid axes parameter')
            axes = [_axes[ax] for ax in _axes if ax in axes_names]
        # Check axes sensors, retry tolerance, retries count change
        chip = gcmd.get(f'ACCEL_CHIP', None)
        for axis in axes:
            axis_name = axis.name.upper()
            if isinstance(axis.chip_helper, AccelHelper):
                ax_chip = gcmd.get(f'ACCEL_CHIP_{axis_name}', chip)
                if ax_chip and ax_chip != axis.chip_helper.get_name():
                    try:
                        axis.init_accel_chip_helper(ax_chip)
                    except Exception as e:
                        raise self.gcode.error(str(e))
            retry_tol = gcmd.get_float(f'RETRY_TOLERANCE_{axis_name}',
                                       default=None, above=0.)
            if retry_tol is None:
                retry_tol = gcmd.get_float(f'RETRY_TOLERANCE',
                                           default=None, above=0.)
            if retry_tol is not None:
                axis.set_retry_tolerance(retry_tol)
            retries = gcmd.get_int(f'RETRIES_{axis_name}',
                                   default=None, minval=0)
            if retries is None:
                retries = gcmd.get_int(f'RETRIES',
                                       default=None, minval=0)
            if retries is not None:
                axis.set_max_retries(retries)
        self.status.reset()
        self.gcode.respond_info('Motors synchronization started')
        self.kin_helper.start_sync(axes)
        self.status.check_retry_result('done')

    cmd_SYNC_MOTORS_CALIBRATE_help = 'Calibrate synchronization process model'
    def cmd_SYNC_MOTORS_CALIBRATE(self, gcmd):
        # Calibrate sync model and model coeffs
        if self.calibrate_steps_model_helper is None:
            self.calibrate_steps_model_helper = MotorsSyncCalibrate(self)
        self.calibrate_steps_model_helper.run_calibrate(gcmd)
        self.status.reset()

    def get_status(self, eventtime):
        return self.status.get_status(eventtime)


# Performs virtual-axis step-to-deviation model calibration, which is
# used for a faster and more correct steppers synchronization procedure.
class MotorsSyncCalibrate:
    def __init__(self, sync):
        self.kin_helper = sync.kin_helper
        self.printer = sync.printer
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.path = os.path.expanduser(PLOT_PATH)
        try:
            self._check_modules()
            self._check_export_path()
        except Exception as e:
            raise self.gcode.error(f'motors_sync: {e}')

    def _check_modules(self):
        result = []
        required_mods = ['textwrap', 'matplotlib.pyplot',
                         'matplotlib.ticker', 'scipy.optimize']
        def find_mods():
            nonlocal result
            try:
                missing_mods = [
                    mod for mod in required_mods if
                    importlib.util.find_spec(mod) is None]
                result = (True, ', '.join(missing_mods))
            except BaseException as e:
                result = (False, e)
        thread = threading.Thread(target=find_mods)
        thread.start()
        now = start_t = self.reactor.monotonic()
        while thread.is_alive():
            if now > start_t + 30.0:
                raise Exception("Check packages stuck!")
            now = self.reactor.pause(now + .1)
        thread.join()
        if not result[0]:
            raise Exception(f'Check packages finished '
                            f'with error {result[1]}')
        if result[1]:
            raise Exception(f"Missing packages: {result[1]}")

    def _check_export_path(self):
        try:
            os.makedirs(self.path, exist_ok=True)
        except Exception as e:
            raise Exception(f'Failed to generate export path\n{e}')

    math_models = {
        'linear': (lambda x, a, b: a*x + b, '-.', '#DF8816'),
        'quadratic': (lambda x, a, b, c: a*x**2 + b*x + c, '--', 'green'),
        'power': (lambda x, a, b: a * np.power(x, b), ':', 'cyan'),
        'root': (lambda x, a, b: a * np.sqrt(x) + b, '--', 'magenta'),
        'hyperbolic': (lambda x, a, b: a / x + b, '-.', 'purple'),
        'exponential': (lambda x, a, b, c: a * np.exp(b*x) + c, ':', 'blue')
    }

    def find_best_func(self, x_data, y_data, x_prio_len=0, maxfev=999999999):
        from scipy.optimize import curve_fit
        x_len = len(x_data)
        if x_prio_len == 0 or x_prio_len == x_len:
            x_prio_len = x_len
            sigma = None
        else:
            weights = np.ones(x_len)
            weights[x_prio_len:] = np.linspace(1.0, 0.01, x_len - x_prio_len)
            sigma = 1 / weights
        funcs = []
        with np.errstate(over='raise', divide='raise', invalid='raise'):
            for name, (math_model, *params) in self.math_models.items():
                try:
                    coeffs, _ = curve_fit(math_model, x_data, y_data,
                                          sigma=sigma, maxfev=maxfev)
                    y_pred = math_model(x_data, *coeffs)
                    diff = y_data[:x_prio_len] - y_pred[:x_prio_len]
                    rmse = np.sqrt(np.mean(diff ** 2))
                    funcs.append({'name': name, 'rmse': rmse,
                                  'coeffs': coeffs})
                except BaseException as e:
                    logging.warning(f"motors_sync_calibrate: "
                                    f"error on {name} - {e}")
        funcs = sorted(funcs, key=lambda f: f['rmse'])
        info = ['Functions RMSE and coefficients']
        for f in funcs:
            c_str = ','.join([f'{c:.10f}' for c in f['coeffs']])
            info.append(f"{f['name']}: RMSE {f['rmse']:.2f} coeffs: {c_str}")
        return info, (x_data, y_data, funcs)

    def plotter(self, x_data, y_data, funcs, axis, accel_chip,
                peak_msteps, fullstep, rmse_lim=20000):
        from textwrap import wrap
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
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
        configfile = self.printer.lookup_object("configfile")
        axes_names = self.kin_helper.get_linked_calibration_axes(axis)
        for axis_name in axes_names:
            pname = f'steps_model_{axis_name}'
            func_name = func['name']
            func_coeffs = list(map(str, func['coeffs']))
            block = func_name + ',\n  ' + ',\n  '.join(func_coeffs)
            configfile.set('motors_sync', pname, block)
            msg = f"{pname}: {func_name}, {','.join(func_coeffs)}"
            self.gcode.respond_info(msg)
        self.gcode.respond_info(
            f"Motors sync model for '{', '.join(axes_names)}' "
             "axis has been calibrated.\nThe SAVE_CONFIG command "
             "will update the printer config\n file with new "
             "parameters and restart the printer.")

    def run_calibrate(self, gcmd):
        axis_name = gcmd.get('AXIS').lower()
        axis = self.kin_helper.motion_axes.get(axis_name)
        if axis is None:
            raise self.gcode.error(f'Invalid axis: {axis_name.upper()}')
        peak_mstep = gcmd.get_int('DISTANCE', 16, minval=2, maxval=16 * 2)
        repeats = gcmd.get_int('REPEATS', 2, minval=2, maxval=100)
        need_plot = False
        need_plot_str = gcmd.get('PLOT', 'True').lower()
        if need_plot_str == 'true' or need_plot_str == '1':
            need_plot = True
        self.gcode.respond_info(
            f'Calibration started on {axis.name_prefixed} axis with '
            f'{repeats} repeats, move to +-{peak_mstep}/16 microstep')
        self.gcode.respond_info('Synchronizing before calibration...')
        self.kin_helper.start_sync(force_run=True)
        max_steps, magnitudes = self.kin_helper\
            .calibrate_axis_steps_model(axis, peak_mstep, repeats)
        y_samples = np.sort(np.array(magnitudes))
        x_samples = np.linspace(0.01, max_steps, len(y_samples))
        div = 1. if peak_mstep < 5 else peak_mstep / 5
        x_prio_len = int(x_samples.shape[0] // div)
        x_samples_str = ', '.join([f'{i:.2f}' for i in x_samples])
        y_samples_str = ', '.join([str(i) for i in y_samples])
        logging.info(f"motors_sync_calibrate: x = [{x_samples_str}]")
        logging.info(f"motors_sync_calibrate: y = [{y_samples_str}]")
        self.gcode.respond_info('Data processing...', True)
        def samples_processing():
            try:
                os.nice(10)
            except:
                pass
            try:
                info, data = self.find_best_func(x_samples, y_samples,
                                                 x_prio_len)
                msg = None
                if need_plot:
                    msg = self.plotter(*data, axis.name,
                        axis.chip_helper.chip_name, peak_mstep, 16)
                c_conn.send((False, (msg, data)))
            except Exception as e:
                c_conn.send((True, (e,)))
            finally:
                c_conn.close()
        self.gcode.respond_info('Data processing...')
        p_conn, c_conn = multiprocessing.Pipe()
        proc = multiprocessing.Process(target=samples_processing)
        proc.daemon = True
        proc.start()
        now = start_t = last_report_time = self.reactor.monotonic()
        while proc.is_alive():
            if now > last_report_time + 10.:
                last_report_time = now
                self.gcode.respond_info('Data processing...')
                if now > start_t + 60.:
                    proc.terminate()
                    raise self.gcode.error(f'Data processing stuck!')
            now = self.reactor.pause(now + 1.)
        err, res = p_conn.recv()
        proc.join()
        p_conn.close()
        if err:
            raise self.gcode.error(f'Data processing finished '
                                   f'with error: {res[0]}')
        if res[0]:
            self.gcode.respond_info(str(res[0]))
        # Save best fit function in configfile
        self.save_config(axis, res[1][-1][0])


# Lightweight 1D Kalman filter implementation for noise filtering.
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


# Generic csv log file helper with header and size validation.
class LogFileHelper:
    def __init__(self, file_name, format, max_size=1000):
        self.format = format
        self.max_size = max_size
        # Variables
        self.home_dir = os.path.dirname(os.path.realpath(__file__))
        self.log_path = os.path.join(self.home_dir, file_name)
        self.error = ''
        # Checks
        self.check_log()

    def check_log(self):
        if os.path.exists(self.log_path):
            header, raw_log = self.read_log(True)
            if ','.join(header) != self.format:
                self.clear_log()
            elif raw_log.shape[0] > self.max_size:
                self.trim_log(header, raw_log, self.max_size)
        else:
            try:
                self.write_log(self.format.split(','))
            except Exception as e:
                self.error = str(e)

    def read_log(self, with_header=False):
        with open(self.log_path, mode='r', newline='') as f:
            data = list(csv.reader(f, delimiter=','))
        if with_header:
            return data[0], np.array(data[1:])
        return np.array(data[1:])

    def write_log(self, line):
        with open(self.log_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(line)

    def trim_log(self, header, raw_log, max_size):
        last_rows = raw_log[-max_size:]
        with open(self.log_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(last_rows)

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


# Records sync runs, extract the last successful sync positions, provides
# general synchronizations statistics to user by G-code command.
class MotionAxesStats:
    def __init__(self, config):
        self.gcode = config.get_printer().lookup_object('gcode')
        # Register commands
        self.gcode.register_command('SYNC_MOTORS_STATS', self.cmd_GET_STATS,
                                    desc=self.cmd_GET_STATS_help)
        # Variables
        file_name = 'sync_stats.csv'
        format = 'axis,status,magnitudes,steps,msteps,retries,offset,date,'
        self.log_helper = LogFileHelper(file_name, format)

    def _find_axes_phase_offsets(self, axes):
        raw_log = self.log_helper.read_log()
        if raw_log.size == 0:
            return {}
        log = self.log_helper.parse_raw_log(raw_log)
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

    def get_axes_phase_offsets(self, axes):
        offsets = self._find_axes_phase_offsets(axes)
        logging.info(f'motors_sync_phase_offset: {offsets}')
        return offsets

    def write_axis_stats_log(self, axis, state):
        if self.log_helper.error:
            return
        if not axis.actual_msteps:
            return
        name = axis.name
        magnitudes, pos = zip(*axis.motion_log)
        msteps = axis.microsteps
        retries = axis.curr_retry
        offset = axis.query_phases_offset()
        offset = "None" if offset is None else offset
        date = datetime.now().strftime('%Y-%m-%d')
        self.log_helper.write_log([name, state, magnitudes, pos,
                                   msteps, retries, offset, date])

    def respond_stats(self, log):
        a = {}
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
            cf_microsteps = 16
            st_microsteps = a['msteps'] / a['count'] * (cf_microsteps / 16)
            msg = (f"""
            --------------------------------
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
            self.gcode.respond_info(msg)

    cmd_GET_STATS_help = 'Show statistics'
    def cmd_GET_STATS(self, gcmd):
        do_clear = gcmd.get('CLEAR', '').lower()
        if do_clear in ['true', '1']:
            self.log_helper.clear_log()
            self.gcode.respond_info('Statistics was cleared')
            return
        if self.log_helper.error:
            self.gcode.respond_info(f'Statistics collection is disabled '
                                    f'due:\n{self.log_helper.error}')
            return
        raw_log = self.log_helper.read_log()
        if raw_log.size == 0:
            self.gcode.respond_info('Logs are empty')
            return
        log = self.log_helper.parse_raw_log(raw_log)
        self.respond_stats(log)


def load_config(config):
    return MotorsSync(config)
