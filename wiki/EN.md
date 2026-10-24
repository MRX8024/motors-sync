# Klipper stepper motor synchronization module

This module implements software synchronization of interconnected stepper
motors in various types of kinematics.

---

## Supported kinematics

- **CoreXY**
- **Cartesian** *(bedslinger configurations only)*

---

## Why this is needed
Synchronization is required to eliminate mechanical stresses that may arise
between motors operating in the same belt system.

In AWD systems the belt is physically split by motor pulleys into two loops.
For correct operation both motors must operate consistently — their positions
must correspond to each other in such a way that belt tension remains uniform
in both loops.

However, stepper motors do not power on at an arbitrary position; their
position is determined by discrete steps. The design of the stepper motor
itself allows the rotor position to be uniquely set only within a range of 4
full steps. When the TMC driver is first enabled, voltage is applied to the
motor phases so that its position is near a full step and near the beginning of
the 4 full step range.

A standard stepper motor with a 1.8° step has `360 / 1.8 = 200` full steps per
revolution. Considering that the position can only be determined within a
range of 4 full steps, we get `200 / 4 = 50` ranges in which the rotor
position is controlled.

Assume the motor is in range 25, at the position of the 2nd full step out of 4.
If the motor is disabled, manually rotated to range 20 and then enabled again,
it will remain in that range and will snap to the previous position within the
range — to the position of the 2nd full step out of 4. The maximum position
mismatch is half the range, `4 / 2 = 2` full steps. If the difference is larger,
the motor will snap to the neighboring range when enabled. When using two motors
in one belt loop, the theoretically possible difference doubles and becomes 4
full steps.

For a standard configuration with a GT2 20T pulley the maximum possible position
mismatch between motors will be:

`(20 * 2) / 200 * 4 = 0.8mm`

When desynchronized, one belt loop becomes overtensioned and the other loosens.
The motors begin to «pull» the system in different directions,
which leads to several negative effects:
- increased load on the entire mechanics;
- reduced positioning accuracy and motion stability;
- in CoreXY kinematics — bending of the X beam due to unequal belt tension.

Disadvantages of manual synchronization (pulleys):
- can drift due to thermal expansion;
- lost after re-tensioning belts;
- drifts over time due to mechanical factors.

---

## Principle of operation

### Measurement method
One of the motors is temporarily disabled; at that moment belt tension between
the two loops equalizes due to free rotation of the disabled motor, which no
longer holds its position. After re-enabling, the motor attempts to return to
its previous position, and if the current motor position differs from the
original one, an impulse occurs. The motor again tensions one of the loops while
 loosening the other, attempting to return to its original position.

The strength of the resulting impulse is directly related to the degree of motor
desynchronization in the belt loop. The greater the desynchronization, the
stronger the impulse. With perfect synchronization the impulse is absent,
because re-enabling the motor is not accompanied by movement — it remained in
its original position.

### Measuring motor position deviation
To estimate the motor position deviation one of the following sensors is
used:
- An accelerometer mounted on the carriage in the belt loop, which records
  short-term carriage acceleration when the motor impulse occurs. Measurement
  units — magnitude.
- An encoder mounted on one of the motors in the belt loop, directly recording
  the rotor deviation value. Measurement units — microns.

Further in the article the term «magnitude» refers to the value of motor
position deviation regardless of the measurement units used.

### General synchronization algorithm
The algorithm implements a step-by-step procedure to minimize the motor impulse
magnitude. First the motor movement direction is determined in which the impulse
magnitude is expected to decrease. Then the motor position is adjusted step by
step while recording the change of impulse magnitude at each step.

As long as the impulse magnitude decreases, correction continues. If the impulse
magnitude starts increasing, it means the minimum point has already been reached
— the position of ideal synchronization at the current microstep resolution. In
this case a rollback to the previous step is performed, after which the
synchronization process finishes.

If necessary, repeated iterations are performed to reach a predefined impulse
magnitude threshold. This increases stability by eliminating possible
measurement errors related to mechanical issues and other causes.

---

## Notes
It is recommended to rigidly mount the accelerometer at the closest point where
the belt is attached to the carriage. Mounting in other locations, for example
using a CAN-board accelerometer or a Beacon mounted on a flexible bracket, may
distort measurements.

It is not recommended to use accelerometers other than: `adxl345`,`adxl355`,
`icm20948`, `mpu9250`. Other models have insufficient sampling frequency for
correct impulse measurement. Their operation was optimized by disabling data
filtering, which gives these sensors a chance to record motor impulses. However,
measurement accuracy remains low — correct operation is not guaranteed.

It is not recommended to enable hotend heating during synchronization. A
running fan (generally any fan in the printer) may interfere with correct
impulse measurement. You can measure and compare noise using the standard
command: [MEASURE_AXES_NOISE](
https://www.klipper3d.org/G-Codes.html#measure_axes_noise).

In most cases the configuration parameter `microsteps` (in the `motors_sync`
section) should not be increased above 16 when using a GT2 20T pulley. This
parameter defines the resolution at which the motors will step during the
synchronization process. If set above the recommended value, the difference in
impulse magnitude between adjacent motor positions may be imperceptible to the
accelerometer, which may prematurely terminate the synchronization process due
to an incorrect measurement.

---

## Installing the calibration script on the printer host
Clone the project repository and follow the auto-installer instructions. If
necessary the installation can be performed manually by following the 
[manual instructions](/wiki/manual_install_en.md).
```
cd ~
git clone https://github.com/MRX8024/motors-sync
bash ~/motors-sync/install.sh
```

---

## Configuration
Most parameters support self-assignment to an axis. For example with `axes: x,y`
the parameter `accel_chip` can be defined as `accel_chip_x` and `accel_chip_y`.
However `accel_chip` remains the default parameter if no axis-specific value is
specified. This is relevant for some kinematics where axes may be physically
unrelated, have different implementations, and require individual configuration
parameters.

Parameters starting with `#` are optional. They have default values listed next
to them but can be adjusted for different printer configurations. It is
recommended to review all configuration parameters before using the program.

Add the following section to the printer's configuration file and partially
configure it for the first measurement:

```
[motors_sync]
axes:
#    Axes on which synchronization will be performed, separated by commas.
#    For example, for CoreXY kinematics: x,y.
accel_chip:
#    Accelerometer for vibrations collection: adxl345 / mpu9250 etc.
#encoder_chip_<axis>:
#    Axis name, assigned encoder name, for measuring motor deviations.
#microsteps: 16
#    Maximum microsteps resolution relative to motor full step.
#steps_model: linear, 10000, 0
#    Mathematical model and its coefficients describing the dependence of
#    motor microstep displacement on the magnitude of the measured impulse.
#    The model determines how many microsteps the motor position is corrected
#    per one correction step depending on the measured impulse magnitude.  
#max_step_size: 3
#    Maximum number of microsteps the motor can move in one correction step
#    to achieve the planned impulse magnitude.
#axes_steps_diff:
#    Difference in motor positions in microsteps between two linked axes,
#    to start checking the position offset of the secondary axis. Relevant
#    only for CoreXY type kinematics. Default value - max_step_size + 1.
#retry_tolerance: 0
#    Forced threshold to which the motor impulse magnitude must drop for the
#    synchronization procedure to complete successfully. Recommended to
#    configure in order to filter possible measurement errors. When using an
#    accelerometer, the typical range is [200..2000].
#retries: 0
#    Maximum number of retries to reach retry_tolerance.
#head_fan:
#    Full name of the toolhead fan that will be turned off during sync to
#    eliminate its noise. Can be specified multiple fans by separating
#    their names with commas.
#axis_prefix_<axis>:
#    Allows you to set a custom prefix that will be displayed before the
#    axis name in terminal outputs. You can use special characters or
#    Unicode icons to make the axes easier to distinguish.
```

---

## Motors synchronization
Enter the `SYNC_MOTORS` command in the terminal on the main page of the
web interface. Some parameters can be overridden at launch:
```
SYNC_MOTORS AXES=[<axes>] ACCEL_CHIP=[<chip_name>] [RETRY_TOLERANCE=<value>] [RETRIES=<value>]
```
A parameter can be specified for a specific axis, for example`ACCEL_CHIP` as
`ACCEL_CHIP_X` for axis `X`. A parameter specified for a specific axis has
priority over the general parameter, just as in the case of configuration
parameters.

Motors synchronization can be started immediately before printing. To do this,
add the command to a macro or slicer. For example:
```
...
M140 S   ; set bed temp
SYNC_MOTORS
G28 Z
M190 S   ; wait for bed temp to stabilize
M104 S   ; set extruder temp
...
```

---

## Synchronization status variable
A variable is introduced that reflects the current status of motors
synchronization. The status is reset when any printer motors are turned off.
The status can be checked inside a macro to avoid starting synchronization if
it has already been performed in the current session:
```
...
{% if not printer.motors_sync.applied %}
    SYNC_MOTORS
{% endif %}
...
```

---

## Statistics and synchronization restoration
Each synchronization iteration is written to a log located in the project
directory. The presence of this log allows, at the launch of the
synchronization process, restoring motors positions based on the last
successful synchronization. This often significantly brings the motors closer
to the synchronization point, which reduces the number of measurements required
and speeds up the synchronization process.

Restoration can be performed manually using the command:
```
SYNC_MOTORS RESTORE_SYNC=1
```
This command allows applying position correction without waiting for axis
homing and launching the full synchronization procedure. Thus, correction can
be applied immediately before the kinematics begin to move. This may be useful
in situations where the motors are strongly desynchronized and even enabling
them in such a state is undesirable. Using this argument only corrects the
motors positions; the synchronization process is not started.

The correction procedure can be automated at Klipper startup by adding the
following configuration block:
```
[delayed_gcode restore_motors_sync]
initial_duration: 1.0
gcode:
  SYNC_MOTORS RESTORE_SYNC=1
```
To view synchronization statistics, use the command:
```
SYNC_MOTORS_STATS
```
To clear statistics:
```
SYNC_MOTORS_STATS CLEAR=1
```

---

## Steps model calibration
The configuration parameter `steps_model` represents a model describing the
dependence of motor position on impulse magnitude. By default, a linear model
with coefficient 10_000 is used, which means: for 10_000 impulse magnitude
there is 1 microstep of motor displacement at a time. This is a basic safe
value, however it may differ for each user depending on the printer design,
accelerometer type, and other parameters. Step model calibration is intended to
select an appropriate function and its coefficients describing the dependence
of motor position on impulse magnitude specifically for your printer. This
increases synchronization speed and reduces the risk of incorrect impulse
magnitude measurements during large motor desynchronization, because with a
correctly calibrated model the algorithm can correct the motor position by
several microsteps in a single correction iteration.

By default, 2 iterations of increasing the motor position to +16/16 step are
performed and then decreasing to -16/16 with a step of 1/16. During this process
impulses are measured at specific motor positions. Calibration always occurs
with a step of 1/16 to improve result stability. After that, the model is
automatically adjusted to your `microsteps` value from the configuration file.

```
SYNC_MOTORS_CALIBRATE AXIS=<axis> [DISTANCE=<value>] [REPEATS=<value>] [PLOT=<0/1>]
```

- `AXIS=` — axis for calibration. In the case of `CoreXY` choose any of `x`,`y`;
  calibration for both axes will be shared;
- `DISTANCE=16` — maximum motor displacement distance in one direction, 
  specified in 1/16-step microsteps;
- `REPEATS=2` — number of repetitions;
- `PLOT=1` — whether to generate a plot, enabled by default.

After calibration is completed, a prompt will appear in the terminal to save
the configuration file with the calibrated model parameters, as well as the
path to the graphical image. Opening the file you can see the following:

<img src="/wiki/pictures/model_calibrate.png" width="600">

The graph shows the dependence of motor position on impulse magnitude obtained
during step model calibration. The table lists model names, coefficients, and
RMSE (root-mean-square error) from the measured points, sorted in ascending
order. The lower the RMSE, the better the model fits the measured data.

For `CoreXY` kinematics the model is saved immediately for both axes, since
they must be identical. For other kinematics choose one of the methods:
* Calibrate the model for each axis individually, if that makes sense;
* Manually specify the calibrated model to the desired axes:
  ```
  steps_model_a: ...
  steps_model_b: ...
  steps_model_c: ...
  ```
* Manually specify a common model for all axes:
  ```
  steps_model: ...
  ```
It should be remembered that a parameter specified for a specific axis has
priority over the common parameter for all axes and overrides it.

---

## Encoder-based synchronization  
Encoders offer high precision down to 1/128 of a step, are independent of the
printer’s spatial orientation, and perform the synchronization process faster.
All encoders described in [configuration reference](
https://www.klipper3d.org/Config_Reference.html#angle) are supported. It can be
installed on any motor in the belt loop. After installation, it is recommended
to perform the calibrations described in [G-Codes](
https://www.klipper3d.org/G-Codes.html#angle). Step model calibration is not
required because the encoder works with absolute rotor deviation values rather
than acceleration like an accelerometer.

---

## Contacts
It is preferable to use the GitHub issue system. You can also contact me
personally on Discord — @mrx8024.

---

## A bit of history:
The project idea emerged in the summer of 2023. Its first author, @altzbox,
realized that an accelerometer could measure impact force when activating
motors. The community was unenthusiastic about the idea, and insufficient
programming skills hindered solo implementation. Six months later, tired of
manual motor synchronization, @altzbox wrote code using ChatGPT, leading to the
first working [version](https://github.com/altzbox/motors_sync) in January 2024.
However, the skills were no longer enough for the further development of the
project. In spring 2024, I (@mrx8024) became interested in the idea and decided
to continue development as a hobby.
