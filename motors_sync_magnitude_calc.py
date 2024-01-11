import os
import pandas as pd
import numpy as np
import time
import signal
import sys

def calculate_magnitude(accel_x, accel_y, accel_z):
    return np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

def process_csv_file(file_path):
    try:
        # Extract filename without directory
        file_name = os.path.basename(file_path)
        
        # Read CSV file
        data = pd.read_csv(file_path)
        
        # Calculate magnitude for each row
        data['magnitude'] = calculate_magnitude(data['accel_x'], data['accel_y'], data['accel_z'])
        
        # Find the 5 maximum magnitudes and calculate their average
        top_5_max_magnitudes = data.nlargest(5, 'magnitude')['magnitude']
        average_max_magnitude = top_5_max_magnitudes.mean()
        
        # Print filename and average magnitude value
        print(f"{file_name} {average_max_magnitude}")
        
        # Delete the processed CSV file
        os.remove(file_path)
    
    except Exception as e:
        print(f"Error processing file {file_path}: {str(e)}")

def process_new_files(directory_path, processed_files):
    # Get a list of all CSV files in the directory
    csv_files = [f for f in os.listdir(directory_path) if f.endswith('.csv')]

    # Filter out files that have already been processed
    new_files = set(csv_files) - processed_files

    # Process new files
    for file_name in new_files:
        file_path = os.path.join(directory_path, file_name)
        process_csv_file(file_path)

    # Update the set of processed files
    processed_files.update(new_files)

def signal_handler(signal, frame):
    print("\nScript terminated by user.")
    sys.exit(0)

def main():
    # Directory containing CSV files
    directory_path = '/tmp'

    # Set to keep track of processed files
    processed_files = set()

    print("Scanning for new CSV files...")

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    while True:
        # Process new files every 5 seconds
        process_new_files(directory_path, processed_files)
        time.sleep(5)

if __name__ == "__main__":
    main()
