import subprocess
import os
import pandas as pd
import numpy as np
import time
from scipy.signal import medfilt

def calculate_magnitude(accel_x, accel_y, accel_z):
    return np.sqrt(accel_x**2 + accel_y**2 + accel_z**2) # Remove the axis that is perpendicular to the ground.

sync_magnitude_threshold = 1500 #Specify the minimum magnitude value for synchronized motors based on your measurements. The value is used to calculate the number of steps in one iteration.

def home_printhead():
    subprocess.run(["echo _HOME_XY_AND_MOVE_TO_CENTER > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def resume_cmd():
    subprocess.run(["echo RESUME > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def activate_and_measure_x():
    subprocess.run(["echo _ACTIVATE_AND_MEASURE_X > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def activate_and_measure_y():
    subprocess.run(["echo _ACTIVATE_AND_MEASURE_Y > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def force_move_xoneplus(steps):
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_XONEPLUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def force_move_xoneminus(steps):
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_XONEMINUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def force_move_yoneplus(steps):
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_YONEPLUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    
def force_move_yoneminus(steps):
    for _ in range(steps):
        subprocess.run(["echo _FORCE_MOVE_YONEMINUS > ~/printer_data/comms/klippy.serial"], check=True, shell=True)
    

def process_generated_csv(directory_path='/tmp', median_filter_window=3):
    try:
        # Get the list of all CSV files in the directory
        csv_files = [f for f in os.listdir(directory_path) if f.endswith('.csv')]

        if not csv_files:
            print("No CSV files found in the directory.")
            return None

        # Pick the first CSV file in the list
        file_name = csv_files[0]
        file_path = os.path.join(directory_path, file_name)

        # Read CSV file
        data = pd.read_csv(file_path)

        # Apply median filter to accelerometer data
        data['filtered_accel_x'] = medfilt(data['accel_x'], kernel_size=median_filter_window)
        data['filtered_accel_y'] = medfilt(data['accel_y'], kernel_size=median_filter_window)
        data['filtered_accel_z'] = medfilt(data['accel_z'], kernel_size=median_filter_window)

        # Calculate magnitude for each row using filtered data
        data['magnitude'] = calculate_magnitude(data['filtered_accel_x'], data['filtered_accel_y'], data['filtered_accel_z'])

        # Find the 5 maximum magnitudes and calculate their average
        top_max_magnitudes = data.nlargest(5, 'magnitude')['magnitude']
        average_max_magnitude = top_max_magnitudes.mean()

        # Print average magnitude value to Klipper console
        subprocess.run([f'echo M118 "Magnitude: {average_max_magnitude}" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)

        # Delete the processed CSV file
        os.remove(file_path)

        return average_max_magnitude

    except Exception as e:
        print(f"Error processing generated CSV: {str(e)}")
        return None


def main():
    print("Homing the printhead...")
    home_printhead()
    
    print("Sending ACTIVATE_AND_MEASURE_X command...")
    subprocess.run([f'echo M118 "X Motors synchronization" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    activate_and_measure_x()
    time.sleep(25)
    initial_magnitude = process_generated_csv()
    microsteps = 0
    magnitude_before_sync = initial_magnitude
    if initial_magnitude is not None:
        print(f"Initial Magnitude: {initial_magnitude}")

        # Send FORCE_MOVE_XONEPLUS command 3 times
        print("Sending FORCE_MOVE_XONEPLUS command...")
        force_move_xoneplus(3)
        microsteps = microsteps + 3    
        # Send ACTIVATE_AND_MEASURE_X command after movement
        print("Sending ACTIVATE_AND_MEASURE_X command after movement...")
        activate_and_measure_x()
        time.sleep(12)  # Adjust the delay time as needed
        new_magnitude = process_generated_csv()

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
                steps = max(int(initial_magnitude / (sync_magnitude_threshold * 3)), 1)
                print("Sending FORCE_MOVE_XONEPLUS command...")
                force_move_xoneplus(steps)
                microsteps = microsteps + steps
                activate_and_measure_x()
                time.sleep(12)  # Adjust the delay time as needed
                new_magnitude = process_generated_csv()

                if new_magnitude > initial_magnitude:
                    force_move_xoneminus(steps)
                    microsteps = microsteps - 1
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "X Motors synchronization completed. Adjusted by {microsteps}/16 steps. Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

        if initial_direction == "backward":
            while True:
                steps = max(int(initial_magnitude / (sync_magnitude_threshold * 3)), 1)
                print("Sending FORCE_MOVE_XONEMINUS command...")
                force_move_xoneminus(steps)
                microsteps = microsteps - steps
                activate_and_measure_x()
                time.sleep(12)  # Adjust the delay time as needed
                new_magnitude = process_generated_csv()

                if new_magnitude > initial_magnitude:
                    force_move_xoneplus(steps)
                    microsteps = microsteps + steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "X Motors synchronization completed. Adjusted by {microsteps}/16 steps. Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

    print("Sending ACTIVATE_AND_MEASURE_Y command...")
    subprocess.run([f'echo M118 "Y Motors synchronization" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
    activate_and_measure_y()
    time.sleep(12)
    initial_magnitude = process_generated_csv()
    microsteps = 0
    magnitude_before_sync = initial_magnitude
    if initial_magnitude is not None:
        print(f"Initial Magnitude: {initial_magnitude}")

        # Send FORCE_MOVE_YONEPLUS command 3 times
        print("Sending FORCE_MOVE_YONEPLUS command...")
        force_move_yoneplus(3)
        microsteps = microsteps + 3
        # Send ACTIVATE_AND_MEASURE_Y command after movement
        print("Sending ACTIVATE_AND_MEASURE_Y command after movement...")
        activate_and_measure_y()
        time.sleep(12)  # Adjust the delay time as needed
        new_magnitude = process_generated_csv()

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
                steps = max(int(initial_magnitude / (sync_magnitude_threshold * 3)), 1)
                print("Sending FORCE_MOVE_YONEPLUS command...")
                force_move_yoneplus(steps)
                microsteps = microsteps + steps
                activate_and_measure_y()
                time.sleep(12)  # Adjust the delay time as needed
                new_magnitude = process_generated_csv()

                if new_magnitude > initial_magnitude:
                    force_move_yoneminus(steps)
                    microsteps = microsteps - steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "Y Motors synchronization completed. Adjusted by {microsteps}/16 steps. Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude

        if initial_direction == "backward":
            while True:
                steps = max(int(initial_magnitude / (sync_magnitude_threshold * 3)), 1)
                print("Sending FORCE_MOVE_YONEMINUS command...")
                force_move_yoneminus(steps)
                microsteps = microsteps - steps
                activate_and_measure_y()
                time.sleep(12)  # Adjust the delay time as needed
                new_magnitude = process_generated_csv()

                if new_magnitude > initial_magnitude:
                    force_move_yoneplus(steps)
                    microsteps = microsteps + steps
                    print("Direction changed. Exiting the loop.")
                    subprocess.run([f'echo M118 "Y Motors synchronization completed. Adjusted by {microsteps}/16 steps. Initial magnitude - {magnitude_before_sync}. Final magnitude - {initial_magnitude}" > ~/printer_data/comms/klippy.serial'], check=True, shell=True)
                    break

                initial_magnitude = new_magnitude
    
    resume_cmd()

if __name__ == "__main__":
    main()
