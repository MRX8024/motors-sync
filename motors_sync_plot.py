# Find best function for motors synchronization script
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import numpy as np
import matplotlib.pyplot as plt, matplotlib.ticker as ticker
from textwrap import wrap
from scipy.optimize import curve_fit
from datetime import datetime

RESULTS_FOLDER = os.path.expanduser('~/printer_data/config/adxl_results/motors_sync')
RMSE_LIMIT = 20000

def check_export_path(path):
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except OSError as e:
            print(f'Error generate path {path}: {e}')

def linear_model(x, a, b):
    return a*x + b

def quadratic_model(x, a, b, c):
    return a*x**2 + b*x + c

def cubic_model(x, a, b, c, d):
    return a * x**3 + b*x**2 + c*x + d

def power_model(x, a, b):
    return a * np.power(x, b)

def root_model(x, a, b):
    return a * np.sqrt(x) + b

def hyperbolic_model(x, a, b):
    return a / x + b

def exponential_model(x, a, b, c):
    return a * np.exp(b * x) + c

def calculate_rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2))

models = {
    'Linear': linear_model,
    'Quadratic': quadratic_model,
    'Cubic': cubic_model,
    'Power': power_model,
    'Root': root_model,
    'Hyperbolic': hyperbolic_model,
    'Exponential': exponential_model
}

linestyles = {
    'Linear': '-.',
    'Quadratic': '--',
    'Cubic': '-.',
    'Power': ':',
    'Root': '--',
    'Hyperbolic': '-.',
    'Exponential': ':'
}

colors = {
    'Linear': '#9DB512',       # Green yellow
    'Quadratic': 'green',
    'Cubic': '#DF8816',        # Dark Orange
    'Power': 'cyan',
    'Root': 'magenta',
    'Hyperbolic': 'purple',
    'Exponential': 'blue'
}
def find_func(x_data, y_data, accel_chip='', msteps=16, debug=False):
    maxfev=999999999
    params = {}
    y_pred = {}
    rmse = {}
    for name, model in models.items():
        params[name], _ = curve_fit(model, x_data, y_data, maxfev=maxfev)
        y_pred[name] = model(x_data, *params[name])
        rmse[name] = calculate_rmse(y_data, y_pred[name])

    out = {}
    for name, _ in models.items():
        params_str = ','.join([f'{params[name][i]:.10f}' for i in range(len(params[name]))])
        out[name] = {'val': rmse[name], 'params': params[name], 'equation': params_str}

    sorted_out = sorted(out.keys(), key=lambda x: out[x]['val'])
    string_cmd = ['Functions RMSE and coefficients']
    for num, name in enumerate(sorted_out):
        if debug:
            a = max(len(name) for name in out) - len(name)
            b = max(len(str(round(val))) for val in rmse.values())
            string_cmd.append(f'{name}:{"":<{a}} RMSE {out[name]["val"]:<{b}.0f} coeffs: {out[name]["equation"]}')
        else:
            string_cmd.append(f'{name}: RMSE {out[name]["val"]:.0f} coeffs: {out[name]["equation"]}')
    if debug:
        for line in string_cmd:
            print(line)
    msg = plotter(out, sorted_out, x_data, y_data, accel_chip, msteps, debug)
    string_cmd.insert(0, msg)
    return string_cmd

def plotter(out, sorted_out, x_data, y_data, accel_chip, msteps, debug):
    # Plotting
    fig, ax = plt.subplots()
    ax.scatter(x_data, y_data, label='Samples', color='red', zorder=2, s=10)
    x_fit = np.linspace(min(x_data), max(x_data), 200)
    for num, name in enumerate(sorted_out):
        if out[name]["val"] < RMSE_LIMIT:
            string_graph = f'{name} RMSE: {out[name]["val"]:.0f}'
            linestyle = linestyles[name]
            linewidth = 1
            color = colors[name]
            # if num == 0:
            #     linestyle = '-'
            #     linewidth = 1.5
            #     color = 'black'
            ax.plot(x_fit, models[name](x_fit, *out[name]['params']),
                    label=string_graph, linestyle=linestyle, linewidth=linewidth, color=color)
    ax.legend(loc='lower right', fontsize=6, framealpha=1, ncol=1)
    now = datetime.now().strftime('%Y%m%d_%H%M%S')
    lognames = [now, '_' + accel_chip]
    title = f"Dependency of desynchronization and functions ({''.join(lognames)})"
    ax.set_title("\n".join(wrap(title, 66)), fontsize=10)
    ax.set_xlabel(f'Microsteps: 1/{msteps}')
    ax.set_xticks(np.arange(0, max(x_data)+2.5, 2.5))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(2.5))
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.set_ylabel('Magnitude')
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    # ax.yaxis.set_major_locator(ticker.MultipleLocator(base=1))
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(which='major', color='grey')
    ax.grid(which='minor', color='lightgrey')
    if debug:
        save = input('Enter Y to save: ')
        if save.lower() == 'y':
            plt.savefig(f'{now}.png', dpi=1000)
        plt.show()
    else:
        check_export_path(RESULTS_FOLDER)
        png_path = os.path.join(RESULTS_FOLDER, f'interactive_plot_{accel_chip}_{now}.png')
        plt.savefig(png_path, dpi=1000)
        return f'Access to interactive plot at: {png_path}'

if __name__ == '__main__':
    debug = True
    y_data = input("Enter Y data like *, *, *, ... ")
    x_data = input("Enter X data like *, *, *, ... ")
    y_data = np.sort(np.array([float(i) for i in y_data.split(',')]))
    x_data = np.sort(np.array([float(i) for i in x_data.split(',')]))
    find_func(x_data, y_data, debug=debug)
