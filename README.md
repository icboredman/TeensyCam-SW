# TeensyCam-SW
**Arduino ([teensyduino](https://www.pjrc.com/teensy/teensyduino.html))** project to create firmware for [**TeensyCam**](https://github.com/icboredman/TeensyCam-HW) - stereo camera module

#### _Status:_
* full speed USB mode only (12 Mbps) resulting in max 1 - 2 FPS :cry:
* abandoned in favor of **MCUXpresso**-based version (see branch [master](https://github.com/icboredman/TeensyCam-SW))

## Features:
* Runs on [Teensy 3.6](https://www.pjrc.com/store/teensy36.html) microcontroller board attached to TeensyCam module
* Configures two connected MT9V034 imaging sensors via I2C interface
* Triggers simultaneous image capture using pre-set exposure duration
* Sends image sensor frames to host processor via USB CDC connection
* Allows host-configurable parameters:
  * exposure
  * analog gain
  * digital gain
  * number of lines to send to host


---
More info in my blog page: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision-part-2
