# Line follower robot

This repository is dedicated to the line follower project for the Introduction to Robotics course, taken in the 3rd year (2022-2023) at the Faculty of Mathematics and Computer Science, University of Bucharest.

# Components

* Arduino Uno
* Zip-ties
* Power source - 7.4V LiPo battery
* Wheels
* QTR-8A reflectance sensor, along with screws
* Ball caster
* Chassis
* Medium breadboard
* L293D motor driver
* DC motors
* Wires

# Task

The task of the project was to develop a line follower robot. For maximum grade, the calibration of the QTR-8A reflectance sensor had to be done automatically, and the final track had to be completed in less than 20 seconds. The robot developed by me finished the track in 17.046 seconds.

# Details about the implementation

The automatic calibration of the sensors was implemented by moving the robot left and right along the axis according to the values read by the sensor. If the data from the sensors indicated that the robot was in the right side, it would move left, and if the robot was in the left side, it would move right.

The speed of the motors is controlled by the PID control, and the values which worked best for the constants KP, KI and KD were 8, 0.0001 and, respectively, 2. Also, the motors can also move in reverse if necessary, thus being able to take more sharp turns.

## Bonus 

For bonus points, I added some LEDs that can be used as headlights, and I tried to save the calibration to EEPROM so that it is not performed whenever the robot is powered up, but I did not manage to use it correctly.

# [Demo](https://www.youtube.com/watch?v=UGS8K95a6ec)

# [Code](https://github.com/vladfxstoader/ArduinoLineFollower/blob/main/line_follower/line_follower.ino)

# Setup
![Setup](https://github.com/vladfxstoader/ArduinoLineFollowere/blob/main/setup.jpg?raw=true)
