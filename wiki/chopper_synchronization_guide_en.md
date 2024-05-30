### 1. Install the calibration script on the printer host. (the klipper will reboot!)

```
cd ~
git clone https://github.com/MRX8024/motors-sync
bash ~/motors-sync/install.sh
```

2. Connect the accelerometer as when measuring resonances for input_shaper.
3. Add a section to the configuration file and partially configure it for
   the first measurement -
```
[motors_sync]
#accel_chip:
#    Accelerometer for collecting vibrations adxl345 / mpu9250 / lis2dw, etc.
#microsteps: 16
#    Maximum microstepping of stepper motor rotor displacement, not worth
#    increase the value above 16, do so at your own peril and risk.
#steps_coeff: 999999 (deprecated steps_threshold)
#    Coefficient of number of microsteps of shaft displacement, depending 
#    on the magnitude value (impact), adjusted experimentally. Let's say
#    if our impact was 50,000 units, and the coeff was 10,000 - the motor
#    will move in 5 microsteps at a time to save time.
#fast_threshold: 999999
#    Threshold up to which the motor will not perform decaying oscillations,
#    to save time, due to the already high deviations. Be very be careful
#    when decreasing the value of this parameter.
#retry_tolerance: 999999
#    Forced threshold to which a pair of stepper motors should will omit
#    deviations. After several runs calibration, you will find the limit 
#    to which you can lower this parameter.
#retries: 0
#    Maximum number of repetitions to achieve forced motor synchronization
#    deviation threshold.
#respond: True
#    Enable / disable debugging of intermediate measurements.
```

4. Motor synchronization:

   Enter the command `SYNC_MOTORS` in the terminal, on the main page of the
   web interface, and wait for the process to complete.
   
   Some parameters can be overridden:
   ```
   SYNC_MOTORS [ACCEL_CHIP=<chip_name>] [STEPS_COEFF=<value>]
   [FAST_THRESHOLD=<value>] [RETRY_TOLERANCE=<value>] [RETRIES=<value>]
   ```
   For the convenience of additional parameter settings, you can add macro
   from `motors_sync.cfg` to get physical functions\cells in the interface.


5. Notes:
   1. Do not turn on the hotend heating during synchronization.
      A running fan (basically any fan in the printer) may prevent correct
      and more accurate measurements. But if it has to be turned on, try
      not to let it turn off in the middle of measurements. You can compare
      noises with the command - `MEASURE_AXES_NOISE`


6. Synchronization can be started at the beginning of printing
   while the bed is heating up. To do this, you need to add it 
   to the macro \ slicer. For example -
```
M140 S   ;set bed temp
SYNC_MOTORS
G28 Z
M190 S   ; wait for bed temp to stabilize
M104 S   ;set extruder temp
BED_MESH_CALIBRATE
...
```
7. A variable of status is also introduced, which is reset when the printer
   motors are turned off. You can check its status inside the macro, and do
   not do calibration again, if it has already been done in the current
   session. For example -
```
...
G28 X Y
{% if not printer.motors_sync.applied %}
    SYNC_MOTORS
{% endif %}
G28 Z
...
```
