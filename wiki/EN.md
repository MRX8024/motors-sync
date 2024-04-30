
## Welcome to the stepper motor synchronization project for 4wd system

## About the project

### Currently the program supports the following kinematics: `CoreXY` `4 WD`

The synchronization method consists of impact measurement turning on the second stepper, located on one belt.
Impacts are measured by accelerometer that remains stable on the print head, depending on the type of kinematics.
The program determines the direction of movement, then performs a competition until the direction changes in the opposite direction.
In this case, it terminates or makes additional iterations to reach force threshold nth number of times.

### Further [instructions](/wiki/chopper_synchronization_guide_en.md), good luck!

### [Support](https://ko-fi.com/altzbox) project

### Contacts - @altzbox @mrx8024 telegram / discord
