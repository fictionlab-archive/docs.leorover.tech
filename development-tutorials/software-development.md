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

### Writing your own custom firmware

To write a custom project, create a new folder and open it in VS Code. Then, type `Ctrl+Shift+P`, `Create Husarion project`. New files should spawn. 

To set project name, open `CMakeLists.txt` and change `myproject` to your custom name.

Now, you can build the project \(`Ctrl+Shift+B`\) and, if the build was successful, a `myproject.hex` file should appear \(or another if you changed the project name\).

Now, you can edit `main.cpp` file to write your custom code.

There are many examples on [Husarion docs page](https://husarion.com/software/hframework/%20) and in [hFramework repository](https://github.com/husarion/hFramework/tree/master/examples/core2) that present basic functionality provided by the library. You can also take a look at [hFramework API reference](https://husarion.com/core2/api_reference/classes.html%20) to learn more about available classes and their application.

To communicate with Raspberry Pi, however, you need to use ROS topics.   
The rosserial client library is provided in hROS module, so make sure this line is present in your `CMakeLists.txt`:

```text
enable_module(hROS)
```

Here's an example code that uses hFramework to interact with CORE2 board hardware and hROS to communicate with rosserial node on Raspberry Pi: 

```cpp
#include "hFramework.h"

// Include ROS client library and the needed message type definitions
#include "ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

using namespace hFramework;

// Declare necessary global variables
ros::NodeHandle nh;

std_msgs::Bool btn_msg;
ros::Publisher *btn_pub;

ros::Subscriber<std_msgs::Empty> *led_sub;

// This function will be called when a new message is received on /toggle_led topic
void toggleLEDCallback(const std_msgs::Empty& msg)
{
    // toggle the led state
    hLED1.toggle();
}

// This function will publish a current button state every 100 ms
void btnLoop()
{
    while (true)
    {
        // Read the button value and replace the message data field
        btn_msg.data = hBtn1.isPressed(); 

        // Publish the message
        btn_pub->publish(&btn_msg);

        //wait 100 ms
        sys.delay(100);
    }
}

void hMain()
{
    // Set baudrate of serial communication to 250kbps
    RPi.setBaudrate(250000);

    // Initialize ROS node with a serial device
    nh.getHardware()->initWithDevice(&RPi);
    nh.initNode();
    
    // Create publisher and subscriber instances
    btn_pub = new ros::Publisher("/btn_state", &btn_msg);
    led_sub = new ros::Subscriber<std_msgs::Empty>("/toggle_led", &toggleLEDCallback);

    // Register new publishers and subscribers
    nh.advertise(*btn_pub);
    nh.subscribe(*led_sub);

    // Create asynchronous task that will publish button state
    sys.taskCreate(&btnLoop);

    while (true)
    {
        // Process all available message callbacks and publications
        nh.spinOnce();

        // wait 10 ms
        sys.delay(10); 
    }
}

```

Replace `main.cpp` with this code, build it and flash to your Board.

Now log into your Rover's console, and type:

```bash
sudo systemctl restart leo
```

This will restart rosserial node \(together with all other ROS nodes\).  
After a while, type:

```bash
rostopic list
```

The topics `/btn_state` and `/toggle_led` should appear on the list.  
The topic `/btn_state` should be published by the firmware every tenth of a second with a Bool message \(True or False\) indicating whether hBtn1 on CORE2 is currently pressed.  
When a message is published to `/toggle_led` topic, hLED1 should change it's state to opposite. 

Try it yourself! Type:

```text
rostopic echo /btn_state
```

and try pressing the button to see how the value changes.

Then, type repeatedly:

```text
rostopic pub /toggle_led std_msgs/Empty
```

and notice how the LED turns on and off.

### Adding features to Leo firmware

Writing your own firmware for the Rover from scratch can be tedious and disappointing. A more straightforward approach would be to add features to stock [leo\_firmware](https://github.com/LeoRover/leo_firmware).

One of the most commonly desired features is to add GPIO support via ROS topics. In this example we will add one publisher that sends voltage level read at pin2 of hExt port \(GPIO input\) and one subscriber that sets voltage level \(high or low\) on pin3 of hExt port \(GPIO output\).

First, we need to include appropriate message type definitions. For GPIO input, we will use [std\_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html) and for the output [std\_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html), so make sure that after `#include "ros.h"`, these messages are included:

```cpp
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
```

Declare these global variables on the top of the file \(where other publishers and subscribers are declared\):

```cpp
std_msgs::Float32 gpio_in_msg;
ros::Publisher *gpio_in_pub;
bool publish_gpio_in = false;

ros::Subscriber<std_msgs::Bool> *gpio_out_sub;
```

Leo firmware publishes all messages in one loop that works with a frequency of 1 kHz. `publish_gpio_in` will tell this loop that a new message is to be published.

Before `initROS()` function \(where all subscriber callbacks are declared\) add this function:

```cpp
void GPIOOutCallback(const std_msgs::Bool& msg)
{
	hExt.pin3.write(msg.data);
}
```

Somewhere inside `initROS()` function, add these lines:

```cpp
gpio_in_pub = new ros::Publisher("/gpio_in", &gpio_in_msg);
gpio_out_sub = new ros::Subscriber<std_msgs::Bool>("/gpio_out", &GPIOOutCallback);

nh.advertise(*gpio_in_pub);
nh.subscribe(*gpio_out_sub);
```

Before `hMain()` function, add:

```cpp
void GPIOInLoop()
{
	uint32_t t = sys.getRefTime();
	long dt = 100;
	while(true)
	{
		if (!publish_gpio_in)
		{
			gpio_in_msg.data = hExt.pin2.analogReadVoltage();
			publish_gpio_in = true;
		}

		sys.delaySync(t, dt);
	}
}
```

This function will check, with frequency of 10Hz, if the previous message was published, and if it was, it will read the voltage value and tell the main loop that the next message is to be published.

Now, in `hMain()` add these lines before `initROS()` function call to setup the pins on hExt port:

```cpp
hExt.pin2.enableADC();
hExt.pin3.setOut();
```

And after `initROS()`, add:

```cpp
sys.taskCreate(&GPIOInLoop);
```

To start the function asynchronously.

The last thing that's left to do is to actually publish the message. To do this, just add these lines to the main loop:

```cpp
if (publish_gpio_in){
	gpio_in_pub->publish(&gpio_in_msg);
	publish_gpio_in = false;
}
```

In case you missed something, here's a working example \(built on top of 0.5.1 version of leo firmware\):  
[https://pastebin.com/7xJRShsS](https://pastebin.com/7xJRShsS)

Build and flash the firmware, log into the console and check with `rostopic list` if the new topics have spawned.

Now, you should be able to check the voltage readings with:

```text
rostopic echo /gpio_in
```

and set the output level with:

```text
rostopic pub /gpio_out std_msgs/Bool -- "data: true"
```

{% hint style="info" %}
`true` will set the output to high \(3.3V\) and `false` will set it to low \(0V\).
{% endhint %}

## 3. ROS development

> The Robot Operating System \(ROS\) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. - [https://www.ros.org/about-ros/](https://www.ros.org/about-ros/)

In the simplest terms, ROS will give us the possibility to write and run different processes \(called [nodes](http://wiki.ros.org/Nodes)\) that communicate with each other by sending and receiving messages on named buses \(called [topics](http://wiki.ros.org/Topics)\) or by calling remote procedures \(called [services](http://wiki.ros.org/Services)\).

This section will describe some basic ROS functionality that can be accomplished with stock Leo Rover.

### Introspecting ROS network with command line tools

ROS comes with some command line tools that can help to introspect the current network of running nodes. Some of the available tools are:

* [rosnode](http://wiki.ros.org/rosnode) - printing information about currently running nodes, killing them, testing connectivity,
* [rostopic](http://wiki.ros.org/rostopic) - listing and printing information about topics currently in use, printing published messages, publishing data to topics, finding a type of published messages
* [rosservice](http://wiki.ros.org/rosservice) - listing and printing information about available services, calling the service with provided arguments,
* [rosmsg](http://wiki.ros.org/rosmsg#rosmsg-1) - displaying the fields of a specified ROS message type

Let's try to run some examples. Before that, connect to the Rover's console:

{% page-ref page="../software-tutorials/connect-to-the-console-ssh.md" %}

Start by reading currently running nodes:

```text
rosnode list
```

You should see most of all the nodes described in the [first section](https://docs.leorover.tech/development-tutorials/software-development#1-software-structure) of this tutorial.  
Among them, the rosserial server node \(called `/serial_node` in this case\), "bridges" communication with the CORE2 board, so any topics it publishes or subscribes are created and used in the firmware.

Let's get more information about this node:

```text
rosnode info /serial_node
```

You should see all the subscribed, published topics and services that the firmware provides. You can learn more about each topic in [leo\_firmware README page.](https://github.com/LeoRover/leo_firmware/blob/master/README.md)

Among published topics, you should see the `/battery` topic. Let's read the published values using `rostopic` tool:

```text
rostopic echo /battery
```

Now, let's look at the `/cmd_vel` topic. This topic is used by the firmware to receive drive commands. We can look at it's type:

```text
rostopic type /cmd_vel
```

You should get `geometry_msgs/Twist`. This is a standard message in ROS for commanding velocity controlled ground robots. We can lookup the message description using `rosmsg` tool:

```text
rosmsg show geometry_msgs/Twist
```

The description should look like this:

```text
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

The `linear` field represents linear velocity \(in meters per second\) along x, y, z axes. `angular` field represents angular velocity \(in radians per second\) along the same axes.

{% hint style="info" %}
You can read more about standard units of measure and coordinate conventions in [REP103](https://www.ros.org/reps/rep-0103.html)
{% endhint %}

For differential drive robots like Leo, only `linear.x` and `angular.z` values are used. 

We can use `rostopic` tool to actually command the Rover to move forward, by sending messages to `/cmd_vel` topic:

```text
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- "linear: {x: 0.2}"
```

The Rover should start moving forward with a velocity of 0.2 m/s.  
To stop message publishing, simply type `Ctrl+C`.

{% hint style="info" %}
The `-r 10` argument tells the `rostopic` tool to publish the message repeatedly 10 times per second instead of publishing only one message. This is necessary because the firmware implements a timeout that will stop the Rover if it doesn't receive the next command after some time \(half a second by default\).
{% endhint %}

### Using ROS client library to publish messages

ROS provides several client libraries that let you write ROS nodes in different languages. The most common ones are [roscpp](http://wiki.ros.org/roscpp) for C++ and [rospy](http://wiki.ros.org/rospy) for Python.

Here is a simple Python node that commands the Rover by publishing to `/cmd_vel` topic:

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# Initialize ROS node
rospy.init_node("test_drive")

# Create ROS publisher
cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# Write a function that drives the Rover with specified
# linear and angular speed for 2 seconds
def drive(linear, angular):
    # Initialize ROS message object
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    
    for _ in range(20): # repeat 20 times
        cmd_pub.publish(twist) # publish message
        rospy.sleep(0.1) # sleep for 100ms

# Now let's actually test driving the Rover
# linear speed is in m/s and angular speed in rad/s 
drive(0.2, 0.0)
drive(0.0, 0.0)
drive(-0.2, 0.0)
drive(0.0, 0.0)
drive(0.0, 1.0)
drive(0.0, 0.0)
drive(0.0, -1.0)
drive(0.0, 0.0)
```

Copy this script to Raspberry Pi filesystem.

{% hint style="info" %}
You can paste this to new file when using a terminal.  
Copy the script to clipboard, then type:

```bash
cat > test_drive.py
```

Type `Ctrl+Shift+V` when using Linux terminal or `Shift+Ins` when using Putty. Then type `Ctrl+d` to end the file.
{% endhint %}

Add execute permission to the file:

```bash
chmod +x test_drive.py
```

And execute it by typing:

```bash
./test_drive.py
```

The Rover should drive forward and backward. then turn in place in left and right direction.

## 4. Web UI development

coming soon!

