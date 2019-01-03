# TeensyCam-SW
**uTasker** project to create firmware for [**TeensyCam**](https://github.com/icboredman/TeensyCam-HW) - stereo camera module

#### _Status:_
* although high-speed USB mode does work, overall reliable operation was not achieved :cry:
* abandoned in favor of **MCUXpresso**-based version (see branch [master](https://github.com/icboredman/TeensyCam-SW))

## Features:
* Built using [uTasker](http://www.utasker.com/) V1.4 and [MCUXpresso IDE](http://www.nxp.com/mcuxpresso/ide) 10.1
* Runs on [Teensy 3.6](https://www.pjrc.com/store/teensy36.html) microcontroller board attached to TeensyCam module
* Configures two connected MT9V034 imaging sensors via I2C interface
* Triggers simultaneous image capture using pre-set exposure duration (no automatic exposure control)
* Sends image sensor frames to host processor via USB CDC connection
* Allows host-configurable parameters:
  * exposure
  * analog gain
  * digital gain
  * number of lines to send to host

More info in my blog page: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision-part-2

---
In addition to the referenced LICENSE file, the following notice is applicable to components included as part of NXP SDK code:

>This is a free software and is opened for education, research and commercial developments under license policy of following terms:
>* This is a free software and there is NO WARRANTY.
>* No restriction on use. You can use, modify and redistribute it for personal, non-profit or commercial product UNDER YOUR RESPONSIBILITY.
>* Redistributions of source code must retain the copyright notice in the Processor Expert component code.
>
>IMPORTANT:
>See as well the copyright/licensing notices in each of the source files.