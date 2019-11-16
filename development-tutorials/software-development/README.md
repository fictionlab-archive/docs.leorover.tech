# Software Development

This set of tutorials will delve deeper into software architecture on Leo Rover, explain how different elements work together and how can they be developed further.

## Software structure

![](../../.gitbook/assets/image%20%2819%29.png)

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

