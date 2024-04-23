### 1. Install the calibration script on the printer host. (the klipper will reboot!)
```
 cd ~
 git clone https://github.com/MRX8024/motors-sync
 bash ~/motors-sync/install.sh
```

2. Connect the accelerometer as when measuring resonances for input_shaper.
3. Add a section to the configuration file -
```
[motors_sync]
```
4. Motor synchronization:
    4. Run the `SYNC_MOTORS` command in the terminal, on the main page of the web interface, and wait for the process to complete.
 

5. Notes:

     1. Synchronization occurs with an accuracy of 1/16 steps. We do not recommend using a smaller microstep (1/32), but it is possible to test this.
     2. Do not turn on the hotend heating during synchronization. A running fan may prevent correct and more accurate measurements.
     3. The more the motors are out of sync, the longer the synchronization takes, but the speed is adjusted by the `steps_threshold` parameter, adjust it wisely.
     4. Synchronization can be started at the beginning of printing while the bed is heating up. To do this, you need to add it to the macro \ slicer. For example -
```
M140 S ;set bed temp
SYNC_MOTORS
M190 S; wait for bed temp to stabilize
M104 S ;set extruder temp
BED_MESH_CALIBRATE
...
```

Extra options -

1. `accel_chip:` - accelerometer for collecting vibrations `adxl345 / mpu9250 / lis2dw`, etc.
2. `microsteps: 16` - maximum crushing of the displacement of the rotor shaft of the stepper motor.
3. `steps_threshold: 100000` - coefficient of the number of microsteps of shaft displacement, depending on the magnitude of the deviation (impact). Let's say if our impact is 50,000 units, and the coefficient is 10,000 - the motor will move 5 microsteps.
4. `fast_threshold: 100000` - the threshold up to which the motor will not perform fading oscillations, due to the already high deviations. `(0 - off)`
5. `force_threshold: 1000` - a forced threshold to which the stepper motor will have to lower the deviations, repeating the measurement procedure nth number of times, described by the following parameter.
6. `threshold_retries:` - the maximum number of repetitions to achieve the forced threshold of motor synchronization deviations; exceeding iterations - the program will generate an error.
7. `respond: False` - enable / disable intermediate debugging.

Also, some parameters can be overridden when running the command:
```
SYNC_MOTORS AXIS=xy STEPS_THRESHOLD=100000 FAST_THRESHOLD=0 FORCE_THRESHOLD=0 THRESHOLD_RETRIES=0
```