# TeensyCam-SW
**MCUXpresso** project to create firmware for [**TeensyCam**](https://github.com/icboredman/TeensyCam-HW) - stereo camera module
---

#### _Status:_
* in development
* known bug: seldomly occuring line skipping, not yet investigated

## Features:
* Built using [MCUXpresso IDE](http://www.nxp.com/mcuxpresso/ide) 10.2 together with [MCUXpresso SDK](www.nxp.com/mcuxpresso/sdk)
* Runs on [Teensy 3.6](https://www.pjrc.com/store/teensy36.html) microcontroller board attached to TeensyCam module
* Configures two connected MT9V034 imaging sensors via I2C interface
* Triggers simultaneous image capture, hardware synchronized
* Sends image sensor frames to host processor via USB CDC connection
* Exposure control - pre-set or automatic
* Allows host-configurable parameters:
  * exposure
  * analog gain
  * digital gain
  * compounding mode
  * frame rate
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