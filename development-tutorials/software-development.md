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
`true` will set the output to high \(3.3V\) and `false` will set the output to low \(0V\).
{% endhint %}

## 3. ROS development

coming soon!

## 4. Web UI development

coming soon!

