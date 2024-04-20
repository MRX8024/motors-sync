import subprocess
import os
import pandas as pd
import numpy as np
import time
from scipy.signal import medfilt


DATA_FOLDER = '/tmp'
# DATA_FOLDER = 'Z:/chopper-resonance-tuner/chopper-resonance-tuner/tmp/'
DELAY = 1.00                # Delay between checks csv in tmp in sec
OPEN_DELAY = 0.25           # Delay between open csv in sec
TIMER = 20.00               # Exit program time in sec
magnitude_threshold = 7500  # Adjust to speed up the process.


def home_printhead():
    subprocess.run(["echo _HOME_XY_AND_MOVE_TO_CENTER > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def resume_cmd():
    subprocess.run(["echo _RESUME_PRINT > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def activate_and_measure_x():
    subprocess.run(["echo _ACTIVATE_AND_MEASURE_X > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def activate_and_measure_y():
    subprocess.run(["echo _ACTIVATE_AND_MEASURE_Y > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def force_move_xoneplus(steps):
    subprocess.run([f'echo M118 "Move X1 motor +{steps}/16 step." > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_XONEPLUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def force_move_xoneminus(steps):
    subprocess.run([f'echo M118 "Move X1 motor -{steps}/16 step." > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_XONEMINUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def force_move_yoneplus(steps):
    subprocess.run([f'echo M118 "Move Y1 motor +{steps}/16 step." > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_YONEPLUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def force_move_yoneminus(steps):
    subprocess.run([f'echo M118 "Move Y1 motor to -{steps}/16 step." > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_YONEMINUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)


def find_z_axis(file_path):
    subprocess.run([f'echo _STATIC_MEASURE > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    wait_csv()
    for f in os.listdir(file_path):
        if f.endswith('stand_still.csv'):
            with open(os.path.join(file_path, f), 'r') as file:
                data = pd.read_csv(file, delimiter=',')
                z_axis = data.iloc[1:].mean().abs().idxmax()
                print(f'Z on "{z_axis}" colum')
                os.remove(os.path.join(file_path, f))
                return z_axis


def wait_csv():
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
                subprocess.run(['RESPOND TYPE=error MSG="No CSV files found in the directory, aborting" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                raise
            else: continue


def process_generated_csv(z_axis, median_filter_window=3, save_filtered_csv=False):
    try:
        wait_csv()
        # Get the list of all CSV files in the directory
        csv_files = [f for f in os.listdir(DATA_FOLDER) if f.endswith('.csv')]
        try:
            if csv_files[1]: raise
        except: pass

        # Pick the first CSV file in the list
        file_name = csv_files[0]
        file_path = os.path.join(DATA_FOLDER, file_name)

        # Read CSV file and
        data = pd.read_csv(file_path)

        # Apply median filter to accelerometer data, calculate magnitude for each row using filtered data
        data['magnitude'] = np.linalg.norm(
                            np.vstack((
                                medfilt(data['accel_x'], kernel_size=median_filter_window)
                                    if z_axis != 'accel_x' else np.zeros_like(data['accel_x']),
                                medfilt(data['accel_y'], kernel_size=median_filter_window)
                                    if z_axis != 'accel_y' else np.zeros_like(data['accel_y']),
                                medfilt(data['accel_z'], kernel_size=median_filter_window)
                                    if z_axis != 'accel_z' else np.zeros_like(data['accel_z']))),
                            axis=0)

        # Find the 5 maximum magnitudes and calculate their average
        average_max_magnitude = data.nlargest(5, 'magnitude')['magnitude'].mean()

        # Print average magnitude value to Web console
        subprocess.run([f'echo M118 "Magnitude: {average_max_magnitude}"'
                        f' > ~/printer_data/comms/klippy.serial'], check=True, shell=True)

        os.remove(file_path)
        return average_max_magnitude

    except Exception as e:
        print(f"Error processing generated CSV: {str(e)}")
        return None


def main():
    print("Homing the printhead...")
    home_printhead()
    os.system('rm -f /tmp/*.csv')
    z_axis = find_z_axis(DATA_FOLDER)
    print("Sending ACTIVATE_AND_MEASURE_X command...")
    subprocess.run([f'echo M118 "X Motors synchronization" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    activate_and_measure_x()
    initial_magnitude = process_generated_csv(z_axis)
    microsteps = 0
    magnitude_before_sync = initial_magnitude
    if initial_magnitude is not None:
        print(f"Initial Magnitude: {initial_magnitude}")
        print("Sending FORCE_MOVE_XONEPLUS command...")
        steps = max(int(initial_magnitude / (magnitude_threshold * 2)), 1)
        force_move_xoneplus(steps)
        microsteps = microsteps + steps
        # Send ACTIVATE_AND_MEASURE_X command after movement
        print("Sending ACTIVATE_AND_MEASURE_X command after movement...")
        activate_and_measure_x()
        new_magnitude = process_generated_csv(z_axis)

        if new_magnitude is not None:
            print(f"New Magnitude: {new_magnitude}")

            # Determine movement direction
            if new_magnitude > initial_magnitude:
                initial_direction = "backward"
            else:
                initial_direction = "forward"

            print(f"Movement Direction: {initial_direction}")
            initial_magnitude = new_magnitude

        if initial_direction == "forward":
            while True:
                print("Sending FORCE_MOVE_XONEPLUS command...")
                steps = max(int(initial_magnitude / magnitude_threshold), 1)
                force_move_xoneplus(steps)
                microsteps = microsteps + steps
                activate_and_measure_x()
                new_magnitude = process_generated_csv(z_axis)

                if new_magnitude > initial_magnitude:
                    force_move_xoneminus(steps)
                    microsteps = microsteps - steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "X Motors synchronization completed. Adjusted by {microsteps}/16 step.'
                                    f' Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}"'
                                    f' > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

        if initial_direction == "backward":
            while True:
                print("Sending FORCE_MOVE_XONEMINUS command...")
                steps = max(int(initial_magnitude / (magnitude_threshold * 2)), 1)
                force_move_xoneminus(steps)
                microsteps = microsteps - steps
                activate_and_measure_x()
                new_magnitude = process_generated_csv(z_axis)

                if new_magnitude > initial_magnitude:
                    force_move_xoneplus(steps)
                    microsteps = microsteps + steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "X Motors synchronization completed. Adjusted by {microsteps}/16 step.'
                                    f' Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}"'
                                    f' > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

    print("Sending ACTIVATE_AND_MEASURE_Y command...")
    subprocess.run([f'echo M118 "Y Motors synchronization" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    activate_and_measure_y()
    initial_magnitude = process_generated_csv(z_axis)
    microsteps = 0
    magnitude_before_sync = initial_magnitude
    if initial_magnitude is not None:
        print(f"Initial Magnitude: {initial_magnitude}")
        print("Sending FORCE_MOVE_YONEPLUS command...")
        steps = max(int(initial_magnitude / magnitude_threshold), 1)
        force_move_yoneplus(steps)
        microsteps = microsteps + steps
        # Send ACTIVATE_AND_MEASURE_Y command after movement
        print("Sending ACTIVATE_AND_MEASURE_Y command after movement...")
        activate_and_measure_y()
        new_magnitude = process_generated_csv(z_axis)

        if new_magnitude is not None:
            print(f"New Magnitude: {new_magnitude}")

            # Determine movement direction
            if new_magnitude > initial_magnitude:
                initial_direction = "backward"
            else:
                initial_direction = "forward"

            print(f"Movement Direction: {initial_direction}")
            initial_magnitude = new_magnitude

        if initial_direction == "forward":
            while True:
                print("Sending FORCE_MOVE_YONEPLUS command...")
                steps = max(int(initial_magnitude / magnitude_threshold), 1)
                force_move_yoneplus(steps)
                microsteps = microsteps + steps
                activate_and_measure_y()
                new_magnitude = process_generated_csv(z_axis)

                if new_magnitude > initial_magnitude:
                    force_move_yoneminus(steps)
                    microsteps = microsteps - steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "Y Motors synchronization completed. Adjusted by {microsteps}/16 step.'
                                    f' Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}"'
                                    f' > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

        if initial_direction == "backward":
            while True:
                print("Sending FORCE_MOVE_YONEMINUS command...")
                steps = max(int(initial_magnitude / magnitude_threshold), 1)
                force_move_yoneminus(steps)
                microsteps = microsteps - steps
                activate_and_measure_y()
                new_magnitude = process_generated_csv(z_axis)

                if new_magnitude > initial_magnitude:
                    force_move_yoneplus(steps)
                    microsteps = microsteps + steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "Y Motors synchronization completed. Adjusted by {microsteps}/16 step.'
                                    f' Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}"'
                                    f' > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

    resume_cmd()

if __name__ == "__main__":
    main()
