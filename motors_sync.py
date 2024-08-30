# Motors synchronization script
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, time, itertools
import numpy as np
from . import z_tilt

MEASURE_DELAY = 0.05        # Delay between damped oscillations and measurement
AXES_STEPS_DELTA = 5        # Steps difference between axes to update axis magnitude
MEDIAN_FILTER_WINDOW = 3    # Number of window lines

class MotorsSync:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.force_move = self.printer.load_object(config, 'force_move')
        self.stepper_en = self.printer.load_object(config, 'stepper_enable')
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.status = z_tilt.ZAdjustStatus(self.printer)
        # Read config
        self.conf_axes, self.do_level = self._init_axes()
        self.accel_chips = self._init_chips()
        self.microsteps = self.config.getint('microsteps', default=16, minval=2, maxval=32)
        self.solve_models = self._init_models()
        self.max_step_size = self.config.getint('max_step_size', default=5, minval=1, maxval=self.microsteps)
        self.retry_tolerance = self.config.getint('retry_tolerance', default=999999, minval=0, maxval=999999)
        self.max_retries = self.config.getint('retries', default=0, minval=0, maxval=10)
        self.debug = self.config.getboolean('debug', default=False)
        # Register commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_RUN_SYNC, desc='Start 4WD synchronization')
        self.gcode.register_command('SYNC_MOTORS_CALIBRATE', self.cmd_CALIBRATE_SYNC, desc='Calibrate synchronization')
        # Variables

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = min(self.toolhead.max_accel, 5000)

    def lookup_config(self, section, section_entries, force_back=None):
        # Loockup in printer config
        out = []
        section = self.config.getsection(section)
        for value in section_entries:
            try:
                out.append(float(section.get(value, force_back)))
            except:
                out.append(section.get(value, force_back))
        if None in out: raise
        return out[0] if len(out) == 1 else out

    def _init_axes(self):
        valid_axes = ['X', 'Y']
        self.kin = self.lookup_config('printer', ['kinematics'])
        if self.kin == 'corexy':
            do_level = True
            axes = [a.upper() for a in self.config.getlist('axes', count=2, default=['x', 'y'])]
        elif self.kin == 'cartesian':
            do_level = False
            axes = [a.upper() for a in self.config.getlist('axes')]
        else:
            raise self.config.error(f"Not supported kinematics '{self.kin}'")
        if any([axis not in valid_axes for axis in axes]):
            raise self.config.error('Invalid axes parameter')
        return axes, do_level

    def _init_chips(self):
        chips = {}
        for axis in self.conf_axes:
            try:
                chips[axis] = self.config.get(f'accel_chip_{axis.lower()}')
            except:
                chips[axis] = self.config.get('accel_chip')
        if self.kin in ['corexy']:
            list_chips = list(chips.values())
            if not all(chip == list_chips[0] for chip in list_chips):
                raise self.config.error(f"Accel chips cannot be different "
                                        f"for a '{self.kin}' kinematics")
        return chips

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
        for axis in self.conf_axes:
            try:
                model = self.config.get(f'model_{axis.lower()}').lower()
            except:
                model = self.config.get('model', 'linear').lower()
            try:
                coeffs_vals = self.config.getlist(f'model_coeffs_{axis.lower()}')
            except:
                coeffs_vals = self.config.getlist('model_coeffs', default=[20000, 0])
            coeffs_args = [chr(97 + i) for i in range(len(coeffs_vals) + 1)]
            model_coeffs = {arg: float(val) for arg, val in zip(coeffs_args, coeffs_vals)}
            if model not in models:
                raise self.config.error(f"Invalid model '{model}'")
            if len(model_coeffs) != models[model]['args']['count']:
                raise self.config.error(f"{model.capitalize()} model requires "
                                        f"{models[model]['args']['count']} coefficients")
            if models[model]['args']['a'] == model_coeffs['a']:
                raise self.config.error(f"Coefficient 'a' cannot be "
                                        f"{model_coeffs['a']} for a '{model}' model")
            final_models[axis] = [models[model]['func'], list(model_coeffs.values())]
        if self.kin in ['corexy']:
            model_params = list(final_models.values())
            if not all(val == model_params[0] for val in model_params):
                raise self.config.error(f"Models and coefficients cannot be "
                                        f"different for a '{self.kin}' kinematics")
        return final_models

    def polynomial_model(self, coeffs, fx):
        sol = np.roots([*coeffs[:-1], coeffs[-1] - fx])
        return max(sol.real)

    def power_model(self, coeffs, fx):
        a, b = coeffs
        return (fx / a) ** (1 / b)

    def root_model(self, coeffs, fx):
        a, b = coeffs
        return (fx**2 - 2*b*fx + b**2) / a**2

    def hyperbolic_model(self, coeffs, fx):
        a, b = coeffs
        return a / (fx - b)

    def exponential_model(self, coeffs, fx):
        a, b, c = coeffs
        return np.log((fx - c) / a) / b

    def _send(self, params):
        self.gcode._process_commands([params], False)

    def _stepper_switch(self, stepper, mode):
        self.stepper_en.motor_debug_enable(stepper, mode)

    def _stepper_move(self, stepper, dist):
        self.force_move.manual_move(stepper, dist, 100, self.travel_accel)

    def _buzz(self, axis):
        # Fading oscillations
        move_len = self.motion[axis]['move_len']
        lookup_sec_stepper = self.motion[axis]['lookuped_steppers'][1]
        self._stepper_switch(self.motion[axis]['stepper'], 0)
        for i in reversed(range(0, int(0.4 / move_len))):
            dist = round((move_len * 4 * i), 4)
            self._stepper_move(lookup_sec_stepper, dist)
            self._stepper_move(lookup_sec_stepper, -dist)

    def _calc_magnitude(self, aclient):
        # Calculate impact magnitude
        if self.debug: start_time = time.perf_counter()
        vect = np.array([[sample.accel_x, sample.accel_y, sample.accel_z]
                         for sample in aclient.get_samples()])
        cut = np.mean(vect[0:100, :], axis=0)
        z_axis = np.abs(cut).argmax()
        static = np.linalg.norm(cut[np.arange(3) != z_axis])
        xy_vect = np.delete(vect, z_axis, axis=1)
        # Add window median filter
        half_window = MEDIAN_FILTER_WINDOW // 2
        magnitudes = np.linalg.norm(np.array(np.median(
            [xy_vect[i - half_window:i + half_window + 1]
             for i in range(half_window, len(xy_vect) - half_window)], axis=1)), axis=1)
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitudes)[-5:])
        if self.debug:
            total_time = time.perf_counter()
            self.gcode.respond_info(f'Static: {static:.2f}, calc time: {total_time - start_time:.6f} sec')
        return np.around(magnitude - static, 2)

    def _measure(self, axis, buzz=True):
        # Measure the impact
        stepper = self.motion[axis]['stepper']
        if buzz: self._buzz(axis)
        self._stepper_switch(stepper, 1)
        self._stepper_switch(stepper, 0)
        aclient = self.motion[axis]['chip_config'].start_internal_client()
        self.toolhead.dwell(MEASURE_DELAY)
        self._stepper_switch(stepper, 1)
        self.toolhead.dwell(MEASURE_DELAY)
        aclient.finish_measurements()
        return self._calc_magnitude(aclient)

    def _homing(self):
        # Homing and going to center
        now = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(now)
        center = {}
        for axis in self.conf_axes:
            stepper = 'stepper_' + axis.lower()
            min_pos, max_pos = self.lookup_config(stepper, ['position_min', 'position_max'], 0)
            center[axis] = min_pos + (max_pos - min_pos) / 2
        if ''.join(self.conf_axes).lower() not in kin_status['homed_axes']:
            self._send(f"G28 {' '.join(self.conf_axes)}")
        self._send(f"G0 {' '.join(f'{axis}{pos}'for axis, pos in center.items())} F{self.travel_speed * 60}")
        self.toolhead.wait_moves()

    def _detect_move_dir(self, axis):
        # Determine movement direction
        m = self.motion[axis]
        self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'])
        m['actual_msteps'] += m['move_msteps']
        m['check_msteps'] += m['move_msteps']
        m['new_magnitude'] = self._measure(axis, True)
        self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']} "
                                f"on {m['move_msteps']}/{self.microsteps} step move")
        m['move_dir'] = [-1, 'Backward'] if m['new_magnitude'] > m['magnitude'] else [1, 'Forward']
        self.gcode.respond_info(f"{axis}-Movement direction: {m['move_dir'][1]}")
        m['magnitude'] = m['new_magnitude']

    def _sync(self):
        def inner_sync(axis, init_axis, force_init):
            m = self.motion[axis]
            if not m['move_dir'][1]:
                if not m['new_magnitude'] and not init_axis or force_init:
                    m['new_magnitude'] = self._measure(axis, True)
                    self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']}")
                    m['magnitude'] = m['new_magnitude']
                    if self.retry_tolerance and m['new_magnitude'] < self.retry_tolerance and not force_init:
                        m['out_msg'] = (f"{axis}-Motors adjusted by {m['actual_msteps']}/{self.microsteps}"
                                        f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                        return
                    elif force_init:
                        return
                self._detect_move_dir(axis)
                return
            m['move_msteps'] = min(max(int(m['solve_model'](m['model_coeffs'], m['magnitude'])), 1), self.max_step_size)
            self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'] * m['move_dir'][0])
            move_msteps = m['move_msteps'] * m['move_dir'][0]
            m['actual_msteps'] += move_msteps
            m['check_msteps'] += move_msteps
            m['new_magnitude'] = self._measure(axis, True)
            self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']} on "
                                    f"{m['move_msteps'] * m['move_dir'][0]}/{self.microsteps} step move")
            if m['new_magnitude'] > m['magnitude']:
                self._stepper_move(m['lookuped_steppers'][0],
                                   m['move_msteps'] * m['move_len'] * m['move_dir'][0] * -1)
                m['actual_msteps'] -= m['move_msteps'] * m['move_dir'][0]
                if self.retry_tolerance and m['magnitude'] > self.retry_tolerance:
                    self.retries += 1
                    if self.retries > self.max_retries:
                        self.gcode.respond_info(
                            f"{axis} Motors adjusted by {m['actual_msteps']}/{self.microsteps}"
                            f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                        raise self.gcode.error('Too many retries')
                    self.gcode.respond_info(
                        f"{axis} Retries: {self.retries}/{self.max_retries} Back on last magnitude: {m['magnitude']}"
                        f" on {m['actual_msteps']}/{self.microsteps} step to reach {self.retry_tolerance}")
                    m['move_dir'][1] = ''
                    return
                m['out_msg'] = (f"{axis}-Motors adjusted by {m['actual_msteps']}/{self.microsteps}"
                                f" step, magnitude {m['init_magnitude']} --> {m['magnitude']}")
                return
            m['magnitude'] = m['new_magnitude']

        init_axis = True
        force_init = False
        if self.do_level and len(self.axes) > 1:
            cycling = itertools.cycle(self.axes)
            while True:
                axis = next(cycling)
                cycling, cycle = itertools.tee(cycling)
                m = self.motion[axis]
                sec = next(cycle)
                s = self.motion[sec]
                if m['out_msg']:
                    if all(bool(self.motion[axis]['out_msg']) for axis in self.motion):
                        return
                    continue
                if m['magnitude'] < s['magnitude'] and not s['out_msg'] and not force_init:
                    continue
                inner_sync(axis, init_axis, force_init)
                init_axis = False
                force_init = False
                if abs(abs(m['check_msteps']) - abs(s['check_msteps'])) > AXES_STEPS_DELTA:
                    force_init = True
                    m['check_msteps'], s['check_msteps'] = 0, 0
        else:
            for axis in self.axes:
                m = self.motion[axis]
                while True:
                    if m['out_msg']:
                        if all(bool(self.motion[axis]['out_msg']) for axis in self.motion):
                            return
                        break
                    inner_sync(axis, init_axis=True, force_init=False)

    def cmd_RUN_SYNC(self, gcmd):
        self.motion = {}
        self.retries = 0
        # Live variables
        axes_from_gcmd = gcmd.get('AXES', '').split(',')
        if axes_from_gcmd:
            self.gcode.respond_info(f'{self.conf_axes}')
            self.gcode.respond_info(f'{axes_from_gcmd}')
            if any([axis.upper() not in self.conf_axes for axis in axes_from_gcmd]):
                raise self.gcode.error(f"Invalid axes parameter")
            self.axes = [axis.upper() for axis in axes_from_gcmd]
        else:
            self.axes = self.conf_axes
        for axis in self.axes:
            self.accel_chips[axis] = gcmd.get(f'ACCEL_CHIP_{axis}', self.accel_chips[axis])
        self.retry_tolerance = gcmd.get_int('RETRY_TOLERANCE', self.retry_tolerance, minval=0, maxval=999999)
        self.max_retries = gcmd.get_int('RETRIES', self.max_retries, minval=0, maxval=10)
        # Run
        self.status.reset()
        self._homing()
        self.gcode.respond_info('Motors synchronization started')
        # Init axes
        for axis in self.axes:
            self.motion[axis] = {}
            self.motion[axis]['chip_config'] = self.printer.lookup_object(self.accel_chips[axis])
            self.motion[axis]['solve_model'], self.motion[axis]['model_coeffs'] = self.solve_models[axis]
            stepper = 'stepper_' + axis.lower()
            self.motion[axis]['stepper'] = stepper
            self.motion[axis]['lookuped_steppers'] = [
                self.force_move.lookup_stepper(stepper + str(n) if n else stepper) for n in range(2)]
            self.motion[axis]['move_dir'] = [1, '']
            rd, fspr = self.lookup_config(stepper, ['rotation_distance', 'full_steps_per_rotation'], 200)
            self.motion[axis]['move_len'] = rd / fspr / self.microsteps
            self.motion[axis]['move_msteps'] = 2
            self.motion[axis]['actual_msteps'] = 0
            self.motion[axis]['check_msteps'] = 0
            self.motion[axis]['init_magnitude'] = self._measure(axis, True)
            self.motion[axis]['magnitude'] = self.motion[axis]['init_magnitude']
            self.motion[axis]['new_magnitude'] = 0
            self.motion[axis]['out_msg'] = ''
            self.gcode.respond_info(f"{axis}-Initial magnitude: {self.motion[axis]['init_magnitude']}")
        if self.retry_tolerance and all(
                params['init_magnitude'] < self.retry_tolerance for params in self.motion.values()):
            self.gcode.respond_info(f'Motors magnitudes are in tolerance ({self.retry_tolerance})')
        else:
            self._sync()
            # Info
            for params in self.motion.values():
                self.gcode.respond_info(params['out_msg'])
        self.status.check_retry_result('done')

    def cmd_CALIBRATE_SYNC(self, gcmd):
        # Calibrate sync model and model coeffs
        from datetime import datetime
        from textwrap import wrap
        import multiprocessing
        import matplotlib.pyplot as plt, matplotlib.ticker as ticker
        from scipy.optimize import curve_fit

        RESULTS_FOLDER = os.path.expanduser('~/printer_data/config/adxl_results/motors_sync')
        RMSE_LIMIT = 20000

        def check_export_path(path):
            if not os.path.exists(path):
                try:
                    os.makedirs(path)
                except OSError as e:
                    raise self.gcode.error(f'Error generate path {path}: {e}')

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

        def find_best_func(x_data, y_data, accel_chip='', msteps=16):
            maxfev = 999999999
            params = {}
            y_pred = {}
            rmse = {}
            for name, model in models.items():
                params[name], _ = curve_fit(model, x_data, y_data, maxfev=maxfev)
                y_pred[name] = model(x_data, *params[name])
                rmse[name] = np.sqrt(np.mean((y_data - y_pred[name]) ** 2))
            out = {}
            for name, _ in models.items():
                params_str = ','.join([f'{params[name][i]:.10f}' for i in range(len(params[name]))])
                out[name] = {'val': rmse[name], 'params': params[name], 'equation': params_str}
            sorted_out = sorted(out.keys(), key=lambda x: out[x]['val'])
            string_cmd = ['Functions RMSE and coefficients']
            for num, name in enumerate(sorted_out):
                string_cmd.append(f'{name}: RMSE {out[name]["val"]:.0f} coeffs: {out[name]["equation"]}')
            msg = plotter(out, sorted_out, x_data, y_data, accel_chip, msteps)
            string_cmd.insert(0, msg)
            return string_cmd

        def plotter(out, sorted_out, x_data, y_data, accel_chip, msteps):
            # Plotting
            fig, ax = plt.subplots()
            ax.scatter(x_data, y_data, label='Samples', color='red', zorder=2, s=10)
            x_fit = np.linspace(min(x_data), max(x_data), 200)
            for num, name in enumerate(sorted_out):
                if out[name]['val'] < RMSE_LIMIT:
                    string_graph = f"{name} RMSE: {out[name]['val']:.0f}"
                    linestyle = linestyles[name]
                    linewidth = 1
                    color = colors[name]
                    ax.plot(x_fit, models[name](x_fit, *out[name]['params']),
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
            check_export_path(RESULTS_FOLDER)
            png_path = os.path.join(RESULTS_FOLDER, f'interactive_plot_{accel_chip}_{now}.png')
            plt.savefig(png_path, dpi=1000)
            return f'Access to interactive plot at: {png_path}'

        repeats = gcmd.get_int('REPEATS', 10, minval=1, maxval=100)
        axis = gcmd.get_int('AXIS', self.axes[0])
        self.gcode.respond_info('Synchronizing before calibration')
        self.cmd_RUN_SYNC(gcmd)
        m = self.motion[axis]
        min_pos, max_pos, rd = self.lookup_config(
            m['stepper'], ['position_min', 'position_max', 'rotation_distance'], 0)
        peak_point = gcmd.get_int('PEAK_POINT', rd * 1250, minval=10000, maxval=999999)
        center = min_pos + ((max_pos - min_pos) / 2)
        pos = itertools.cycle([center - rd, center, center + rd])
        max_steps = 0
        invs = [1, -1]
        y_samples = np.array([])
        m['move_msteps'] = 1
        for i in range(1, repeats + 1):
            # Restore previous true magnitude after _final_sync() and after invs[-1]
            m['new_magnitude'] = m['magnitude']
            self.gcode.respond_info(f"Repeats: {i}/{repeats} Rise to {peak_point:.2f} and lower to ~0 magnitude")
            self._send(f'G0 {axis}{next(pos)} F{self.travel_speed * 60}')
            init = True
            for inv in invs:
                while True:
                    if ((inv == 1 and m['new_magnitude'] > m['magnitude'] and m['new_magnitude'] < peak_point) or
                         (inv == -1 and (m['new_magnitude'] < m['magnitude'] or m['new_magnitude'] > peak_point)) or
                          init):
                        if not (init and inv == 1):
                            if m['new_magnitude'] > (max(y_samples) if y_samples.size > 0 else 0):
                                max_steps += m['move_msteps']
                            y_samples = np.append(y_samples, m['new_magnitude'])
                        m['magnitude'] = m['new_magnitude']
                        self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'] * inv)
                        m['actual_msteps'] += m['move_msteps'] * inv
                        m['new_magnitude'] = self._measure(axis, True)
                        self.gcode.respond_info(f"{axis}-New magnitude: {m['new_magnitude']} on "
                                                f"{m['move_msteps'] * inv}/{self.microsteps} step move")
                        init = False
                    else:
                        break
            # Move on previous microstep
            self._stepper_move(m['lookuped_steppers'][0], m['move_msteps'] * m['move_len'])
            m['actual_msteps'] += m['move_msteps']
        y_samples = np.sort(y_samples)
        x_samples = np.linspace(0.01, max_steps, len(y_samples))
        if self.debug:
            self.gcode.respond_info(f"Y Samples: {', '.join([str(i) for i in y_samples])}")
            self.gcode.respond_info(f"X samples: {', '.join([f'{i:.2f}' for i in x_samples])}")

        def _samples_processing():
            try:
                os.nice(20)
            except:
                pass
            msg = find_best_func(x_samples, y_samples, self.accel_chips[axis], self.microsteps)
            for line in msg:
                self.gcode.respond_info(str(line))

        # Run plotter
        proces = multiprocessing.Process(target=_samples_processing)
        proces.daemon = False
        proces.start()

    def get_status(self, eventtime=None, user=False):
        if not user:
            return self.status.get_status(eventtime)
        else:
            now = self.printer.get_reactor().monotonic()
            return bool(list(self.status.get_status(now).values())[0])

def load_config(config):
    return MotorsSync(config)
