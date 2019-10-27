# Software Development

This tutorial delves deeper into software architecture on Leo Rover, explains how different elements work together and how can they be developed further.

## 1. Software structure

![](../.gitbook/assets/image%20%2817%29.png)

The base Leo Rover software can be divided into 3 main elements:

* **The firmware** This is the program that runs directly on the processor of CORE2 board. It provides different functionalities to the Raspberry Pi through serial bus. The main features of the default [leo\_firmware](https://github.com/LeoRover/leo_firmware) are:
  * Differential drive controller \([cmd\_vel](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) interface\)
  * wheel states monitoring \([joint\_states](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) interface\)
  * servo control
  * battery voltage monitoring
  * wheel odometry calculation
  * IMU, GPS support
* **ROS nodes** When the Raspberry Pi boots, a set of ROS nodes is started. These nodes allow different features to be accessed via ROS topics and services. They are defined in [leo\_bringup](https://github.com/LeoRover/leo_bringup) package and mainly consist of:
  * [rosserial](http://wiki.ros.org/rosserial) node - communicates with the firmware via serial interface and makes its features available via ROS topics and services
  * [Rosbridge](http://wiki.ros.org/rosbridge_suite) server - creates WebSocket that provides a JSON API to ROS functionality for non-ROS programs.
  * [Raspicam node](https://github.com/UbiquityRobotics/raspicam_node) - publishes images from Raspberry Pi camera module to ROS image transport topic
  * [Web video server](http://wiki.ros.org/web_video_server) - provides a video stream of a ROS image transport topic that can be accessed via HTTP
  * Leo system node - provides system shutdown and reboot via ROS topics.
* **Web User Interface** This is the user interface that can be accessed via a web browser. It communicates with Rosbridge server using [roslibjs](http://wiki.ros.org/roslibjs) to access functionalities that are available in ROS topics. The default [leo\_ui](https://github.com/LeoRover/leo_ui) brings features such as:
  * control of the Rover via a keyboard or a virtual joystick
  * control of servo positions
  * display of a camera stream from Web video server
  * output of current battery voltage measurement
  * reboot and shutdown buttons

## 2. Firmware development

The firmware is written in C++ language using [Husarion hFramework](https://husarion.com/software/hframework/) library.   
The easiest way to build it is to use Visual Studio Code with Husarion extension.

### Preparing the environment

You will need:

* [Visual Studio Code](https://code.visualstudio.com)  A lightweight code editor with rich ecosystem of extensions.
* [Husarion extension](https://marketplace.visualstudio.com/items?itemName=husarion.husarion) An extension to VS Code that will help to build and manage applications that use hFramework library.
* \(Optional\) [Git](https://git-scm.com/downloads) A version control system that will help to track source code changes.

### Building Leo firmware

First, you need to clone the [leo\_firmware](https://github.com/LeoRover/leo_firmware) repository.  
If you use the command line, type:

```bash
git clone https://github.com/LeoRover/leo_firmware.git
```

You can also use one of [Git GUI clients](https://git-scm.com/download/gui/windows) instead of the command line.

{% hint style="info" %}
If you don't want to use Git, you can just download the firmware as a [ZIP file](https://github.com/LeoRover/leo_firmware/archive/master.zip) and extract it somewhere. However, consider that by using Git, it may be easier to merge new changes from repository later on.
{% endhint %}

Now you need to open the project in Visual Studio Code.   
Start VS Code, click on `File -> Open Folder` or type `Ctrl+Shift+P`, type `Open Folder` and click enter. Select the `leo_firmware` folder.

To build it, just type `Ctrl+Shift+B`. 

If the build was successful, `leo_firmware.hex` file should appear. If the build failed, a terminal should appear with the error message.

### Flashing the firmware

To flash the firmware, you can connect USB A &lt;--&gt; microUSB cable between your computer and hSerial port \(microUSB port on CORE2\), then type `Ctrl+Shift+P`, `Flash Project to CORE2`.

You can also upload the `leo_firmware.hex` file to your Rover and follow this tutorial:

{% page-ref page="../software-tutorials/flash-firmware-to-core2-board.md" %}

### Modifying firmware parameters

Leo firmware provides many parameters that can be adjusted for more personalized use. The parameters include:

* Servo configuration
  * PWM period
  * Voltage
  * angle to PWM pulse width mapping
* Controller configuration
  * encoder resolution
  * encoder pullup
  * wheel radius
  * robot width
  * PID configuration
  * power and torque limits
  * input timeout
* Other
  * informative LED pin assignment
  * IMU hSens port assignment

To modify some parameters, open `params.h` file and change selected values. Next, build the firmware again \(`Ctrl+Shift+B`\).

## 3. ROS development

coming soon!

## 4. Web UI development

coming soon!

