## Welcome to the stepper motors synchronization project for AWD Systems

## About the Project:

### Currently, the program supports the following kinematics:
`CoreXY / Cartesian` `4 WD`

The synchronization method involves measuring the displacement
(oscillation) of the print head (carriage) when the second stepper motor,
located on the same belt as the first one, is activated. The second motor
moves to the nearest full step, thereby stretching and relaxing the belt
on both sides of the loop due to the first motor holding the belt. The
impacts are measured by an accelerometer permanently located on the
carriage, depending on the kinematic type. Synchronization is 
software-based and is reset when any of the stepper motors on the belt
loop are turned off.

The script determines the direction of movement to reduce the carriage's
oscillations. It adjusts the microsteps until the amount of displacement
decreases, i.e., until the motor starts stretching the belt in the 
opposite direction. In this case, calibration ends at the microstep with
the smallest impact or performs additional iterations to achieve the set
oscillation threshold a specified number of times.

### 1. Installing the calibration script on the printer host -

```
cd ~
git clone https://github.com/MRX8024/motors-sync
bash ~/motors-sync/install.sh
```

2. Connect the accelerometer, for example, as when measuring resonances
for input_shaper.
3. Add a section to the configuration file and partially configure it for
the first measurement -
```
[motors_sync]
axes: x,y
#    Axes on which calibration will be performed.
accel_chip_x:
accel_chip_y:
#    Accelerometers for vibration collection: adxl345 / mpu9250 / lis2dw,
#    etc. Are indicated for each axis on which calibration is performed.
#microsteps: 16
#    Maximum microstepping displacement of the stepper motor rotor. It's
#    not necessary to increase the value above 16 with 20t pulley, these
#    fluctuations are elusive.
#model: linear
#    Model of the dependence of the displacement of microsteps on the
#    shaft of a stepper motor depends on the magnitude of the measured
#    oscillations. Supported models: linear, quadratic, cubic, power, root,
#    hyperbolic, exponential.
#model_coeffs: 20000, 0
#    Coefficients above the described model, for calculating microsteps.
#max_step_size: 5
#    The maximum number of microsteps that the motor can take move at time,
#    to achieve the planned magnitude.
#retry_tolerance: 999999
#    The forced threshold to which a pair of stepper motors on one belt
#    will have to lower the magnitude of the oscillations. It's recommended
#    to configure in order to filter possible inaccuracies. After several
#    iterations of starting synchronization, you will find the edge, to
#    which this parameter should be omitted.
#retries: 0
#    Maximum number of repetitions to achieve a forced threshold of motor
#    synchronization deviations.
```
4. Motor synchronization:
   Enter the `SYNC_MOTORS` command in the terminal on the main web page
   interface and wait for the completion of the process.

   Some parameters can be overridden:
   ```
    SYNC_MOTORS AXES=[<axes>] ACCEL_CHIP_<AXIS>=[<chip_name>]
     [RETRY_TOLERANCE=<value>] [RETRIES=<value>]
   ```
   For the convenience of configuring additional parameters, you can add a
   macro from `motors_sync.cfg` to get the physical buttons\cells in the
   interface.
5. Notes:
    1. Do not turn on the hotend heating during synchronization. Working
       the fan (in general, any fan in the printer) can interfere with
       correct and more accurate measurement. But if he has to be
       when turned on, try not to turn it off in the middle
       measurements. You can measure/compare noises with the standard
       klipper command - [MEASURE_AXES_NOISE
       ](https://www.klipper3d.org/G-Codes.html#measure_axes_noise)
6. Synchronization usually starts at the beginning of printing, during 
   heating the table. To do this, add it to the macro\slicer. For example -
```
M140 S ;set bed temp
SYNC_MOTORS
G28 Z
M190 S   ; wait for bed temp to stabilize
M104 S   ;set extruder temp
...
```
7. A calibration status variable is also entered, which is reset when the
   printer motors are turned off. You can start syncing via
   `motors_sync.cfg`, which already has this check in itself, or check its
   state is inside the macro. In case of a positive status, do not
   calibration once again, if it has already been performed in the current
   session. For example -
```
...
G28 X Y
{% if not printer.motors_sync.applied %}
    SYNC_MOTORS
{% endif%}
G28 Z
...
```
### Calibration of the synchronization model -
After you have made sure that the entire process is working properly, you
do not have accidental erroneous measurements, etc., you can speed up the
execution synchronization by selecting a suitable model for the dependence
of microsteps on the magnitude of fluctuations. To do this, enter the
command `SYNC_MOTORS_CALIBRATE` into the terminal, some parameters can
also be redefined:
```
SYNC_MOTORS_CALIBRATE [PEAK_POINT=<value>] [REPEATS=<value>]
```
By default, the calibration will perform 10 iterations of 
increasing/decreasing magnitude in the range from ~0 to 50K. After it is
completed, you will see in the terminal the path to the graphic image, as
well as the model. Open the file and see something like the following -

![](/wiki/pictures/img_1.png)

Both in the terminal and on the graph, the table shows the name of the
model and its root mean square error (RMSE) from the measured points,
sorted in ascending order. We take the function with the smallest
deviation from the parameters from the terminal, additionally check it
with our eyes on the graph with dots and enter it into the configuration
file. For example -
```
model: exponential
model_coeffs:
    68002.9000704507,
    0.0293145933,
    -70739.3609995888
```
### [Support](https://ko-fi.com/altzbox ) the project
### Contacts - @altzbox @mrx8024 telegram / discord
