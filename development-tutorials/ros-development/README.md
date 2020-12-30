# ROS Development

> The Robot Operating System \(ROS\) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. - [https://www.ros.org/about-ros/](https://www.ros.org/about-ros/)

In the simplest terms, ROS will give us the possibility to write and run different processes \(called [nodes](http://wiki.ros.org/Nodes)\) that communicate with each other by sending and receiving messages on named buses \(called [topics](http://wiki.ros.org/Topics)\) or by calling remote procedures \(called [services](http://wiki.ros.org/Services)\). Please read the [ROS/Concepts Wiki page](http://wiki.ros.org/ROS/Concepts) to get a more clear overview of the concepts related to ROS.

This section will describe some basic ROS functionality that can be accomplished with stock Leo Rover.

## Introspecting ROS network with command line tools

ROS comes with some command line tools that can help to introspect the current network of running nodes. Some of the available tools are:

* [rosnode](http://wiki.ros.org/rosnode) - printing information about currently running nodes, killing them, testing connectivity,
* [rostopic](http://wiki.ros.org/rostopic) - listing and printing information about topics currently in use, printing published messages, publishing data to topics, finding a type of published messages
* [rosservice](http://wiki.ros.org/rosservice) - listing and printing information about available services, calling the service with provided arguments,
* [rosmsg](http://wiki.ros.org/rosmsg#rosmsg-1) - displaying the fields of a specified ROS message type

Let's try to run some examples. Before that, connect to the Rover via SSH:

{% page-ref page="../../basic-guides/connect-via-ssh.md" %}

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

## Using ROS client library to publish messages

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

{% hint style="warning" %}
make sure you don't have a Web UI running at the moment as it may cause conflicts on `/cmd_vel` topic
{% endhint %}

## Adding additional functionality to the rover

LeoOS provides an easy mechanism for adding new functionalities without building any of the base packages. The whole process of starting the ROS nodes at boot can be summarized by the following files:

* **/etc/ros/robot.launch** - a `launch file` that starts the robot's functionality. It includes the launch file from the [leo\_bringup](https://github.com/LeoRover/leo_bringup) package which starts the base functionality of the rover, but also allows to add additional nodes to be started or parameters to be set on the Parameter Server.

{% hint style="info" %}
A [launch file](http://wiki.ros.org/roslaunch/XML) is an XML file that describes a set of nodes to be stared with specified parameters. It can be interpreted with [`roslaunch`](http://wiki.ros.org/roslaunch) tool.
{% endhint %}

* **/etc/ros/setup.bash** - The environment setup file that sets all the environment variables necessary for the successful start of the ROS nodes. It sources the environment setup file from the target ROS distribution \(by default, `/opt/ros/melodic/setup.bash`\) and sets additional [environment variables used by ROS](http://wiki.ros.org/ROS/EnvironmentVariables). 
* **/etc/ros/urdf/robot.urdf.xacro** - the URDF description \(in [xacro](http://wiki.ros.org/xacro) format\) that is uploaded to the Parameter Server by the `robot.launch` file. It includes the robot's model from the [leo\_description](https://github.com/LeoRover/leo_description) package, but also allows to add additional links or joints to the model.
* **/usr/bin/leo-start** - a script that starts the robot's functionality. In short, it sources the `/etc/ros/setup.bash` file and launches the `/etc/ros/robot.launch` file.
* **/usr/bin/leo-stop** - a script that stops the currently running `leo-start` process.

On top of that the `leo` systemd service starts the `leo-start` script when the computer boots.

#### starting the functionality manually

To start the nodes manually, you need to stop the currently running ones first. You can do this either by using the `leo-stop` script:

```bash
leo-stop
```

or by stopping the `leo` service:

```bash
sudo systemctl stop leo
```

If you wish to disable the service from starting at boot, you can type:

```bash
sudo systemctl disable leo
```

To turn the service back on, just type:

```bash
sudo systemctl enable leo
```

Now, to start the nodes manually, type:

```bash
leo-start
```

Type `Ctrl+C` to stop the nodes and exit the script.

#### adding additional nodes to the launch file

To add additional nodes to be started, you can modify the `/etc/ros/robot.launch` file. Take a look at the [launch file XML specification](http://wiki.ros.org/roslaunch/XML) \(especially the [node](http://wiki.ros.org/roslaunch/XML/node) and [param](http://wiki.ros.org/roslaunch/XML/param) tags\) for reference. 

Here's an example that uses `node` and `param` tags:

```markup
<param name="name_of_the_global_parameter"
       value="value_of_the_parameter"/>

<node name="name_of_the_node"
      pkg="name_of_the_package"
      type="name_of_the_executable">
      
      <param name="name_of_the_private_parameter"
             value="value_of_the_parameter"/>
</node>
```

Modify it to your needs, add it to the `/etc/ros/robot.launch` file and restart the nodes.

If you want your additional functionality to be easily switchable, you can put these lines, embedded into `<launch>` tag, into a separate file \(e.g. `/etc/ros/function1.launch`\) and add this lines to the `/etc/ros/robot.launch` file:

{% code title="/etc/ros/robot.launch" %}
```markup
<include if="$(optenv USE_FUNCTION1 false)"
         file="/etc/ros/function1.launch"/>
```
{% endcode %}

Then, add this line to the `/etc/ros/setup.bash` file:

{% code title="/etc/ros/setup.bash" %}
```bash
export USE_FUNCTION1=true
```
{% endcode %}

Now you can toggle the functionality simply by changing the `USE_FUNCTION1` environment variable and restarting the nodes.

#### expanding the URDF model

When integrating a sensor or other device to your rover, you might sometimes want to extend the robot's URDF model to:

* visualize the device attached to the rover in RViz
* make the robot aware of device's collision geometry
* provide additional reference frames \(for example for the sensor readings\)

You can create a separate URDF file for your attached device, like this one:

{% code title="/etc/ros/urdf/sensor.urdf.xacro" %}
```markup
<?xml version="1.0"?>
<robot>
  <!-- a link representing visual and collision 
       properties of the sensor -->
  <link name="sensor_base_link">
    <visual>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.7"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- fixed joint that attaches
       the sensor to the rover's body -->
  <joint name="sensor_base_joint" type="fixed">
    <origin xyz="0.08 0 0"/>
    <parent link="base_link"/>
    <child link="sensor_base_link"/>
  </joint>

  <!-- reference frame for sensor readings -->
  <link name="sensor_frame"/>

  <!-- fixed joint that sets the origin 
       of the reference frame -->
  <joint name="sensor_joint" type="fixed">
    <origin xyz="0 0 0.06"/>
    <parent link="sensor_base_link"/>
    <child link="sensor_frame"/>
  </joint>

</robot>
```
{% endcode %}

and include it in the robot's main URDF file, by adding:

{% code title="/etc/ros/urdf/robot.urdf.xacro" %}
```markup
<xacro:include filename="/etc/ros/urdf/sensor.urdf.xacro"/> 
```
{% endcode %}

Now, when you restart the nodes, a new URDF model should be uploaded to the Parameter Server and you should be able to view the new model in RViz.

![](../../.gitbook/assets/image%20%2818%29.png)

You can use `base_link` as a reference frame for other links in the model. The exact position of the `base_link` origin is defined as the center this mounting hole: 

![X - red, Y - green, Z - blue](../../.gitbook/assets/image%20%2848%29.png)

on the upper plane of the mounting plate. The distance can be easily measured in CAD programs or even using physical measuring tools.

For more examples, you can look at these tutorials:

{% page-ref page="../../integrations/lidar-sensor.md" %}

{% page-ref page="../../integrations/imu-module.md" %}

## Building additional ROS packages

ROS uses its own build system for building packages. To learn about it, read the [catkin/conceptual\_overview](http://wiki.ros.org/catkin/conceptual_overview) and [catkin/workspaces](http://wiki.ros.org/catkin/workspaces) ROS wiki pages. Here's a brief summary:

> The packages are the main unit for organizing software in ROS. The current build system that is used to build ROS packages is [catkin](http://wiki.ros.org/catkin). Catkin packages can be built as a standalone project, but catkin also provides the concept of [workspaces](http://wiki.ros.org/catkin/workspaces). 
>
> When building a catkin workspace, the install targets are placed into an [FHS compliant](https://www.ros.org/reps/rep-0122.html) hierarchy inside the [result space](http://wiki.ros.org/catkin/workspaces#Result_space). A set of [environment setup files](http://wiki.ros.org/catkin/workspaces#Environment_Setup_File) allow extending your shell environment, so that you can find and use any resources that have been installed to that location.

{% hint style="info" %}
The prebuilt ROS packages \(installed from the repository\) are placed into `/opt/ros/distribution_name` directory \(`/opt/ros/melodic` in this case\). To use the environment setup file, just type:

```bash
source /opt/ros/melodic/setup.bash
```

If you use LeoOS, this line is already added to `~/.bashrc` file, so it will be automatically executed when you log into the terminal session.
{% endhint %}

> The catkin build system also supports an [overlay](http://wiki.ros.org/catkin/workspaces#Overlays) mechanism, where one workspace can extend another result space. An `environment setup file` from the result space of such workspace will extend your shell environment by packages from both workspaces.
>
> The build system provides a [catkin\_make](http://wiki.ros.org/catkin/commands/catkin_make) command for building workspaces, but we will use `catkin` command line tool from Python package [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) as it delivers more user-friendly and robust environment for building catkin packages.

In this chapter, will will try to:

* create workspace that extends the `melodic` distribution
* add `leo_bringup` to this workspace and build the package
* modify the `/etc/ros/setup.bash` file to use our overlay

Let's start by creating an empty workspace inside home directory on Raspberry Pi:

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin init
```

We want this workspace to extend the prebuilt packages that are already installed on the system. It should be automatically done if you have already sourced `/opt/ros/melodic/setup.bash` file, but we can also explicitly point out the space to extend:

```bash
catkin config --extend /opt/ros/melodic
```

We need to get the sources of the package to build. If the package is available as a git repository \(like in our case\), you can use the `git clone` command: 

```bash
cd src
git clone https://github.com/LeoRover/leo_bringup.git
```

Some of packages will require installing additional dependencies to build and run them. As the `leo_bringup` package is already installed on the system, this step is redundant. For any other package you can use `rosdep` to automatically install any dependencies:

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -iy
```

 build the workspace:

```bash
catkin build
```

If everything works, a [development space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space) should be created inside the `devel` directory. Let's source the environment setup file inside it:

```bash
source ~/ros_ws/devel/setup.bash
```

Now, when you execute `rospack list`, you should see all of the packages installed on your system, but `rospack find leo_bringup` should point you to the directory on your newly created workspace.

The last step is to modify the `/etc/ros/setup.bash` to use our overlay. Simply edit this file \(e.g. with `nano`\) by removing or commenting out the first line and adding:

{% code title="/etc/ros/setup.bash" %}
```bash
# source /opt/ros/melodic/setup.bash
source /home/pi/ros_ws/devel/setup.bash
```
{% endcode %}

When you start the nodes with `leo-start` script, the `/etc/ros/setup.bash` will use your overlay and the `/etc/ros/robot.launch` file should use the version of `leo_bringup` that you have built in your workspace. 

## Connecting other computer to ROS network 

ROS is designed with distributed computing in mind. The nodes make no assumption about where in the network they run. Configuring your computer to be able to communicate with ROS network will let you run nodes that interfere with the Rover's hardware, as well as graphical tools \(like rqt or rviz\) directly on your host machine. 

To install ROS on your computer, you can follow this tutorial:

{% page-ref page="install-ros-on-your-computer.md" %}

In this section we will assume, you run Ubuntu 18.04 with ROS Melodic.

First, connect your computer to the same network your Rover is connected. It can be either the Rover's Access Point \(`LeoRover-XXXX` by default\) or an external router \(if you followed `Connect to the Internet` tutorial\).

To properly communicate over the ROS network, your computer needs to be able to resolve the `master.localnet` hostname. Open a terminal on your computer and type:

```bash
getent hosts master.localnet
```

If you don't see any output, that means you cannot resolve the hostname.

If you are connected to Rover's Access Point, you should be able to resolve it, but if there is and issue with DNS server on the Rover or you are connected through external router, add this line to `/etc/hosts` file on your computer:

```bash
10.0.0.1 master.localnet
```

{% hint style="warning" %}
If you are connected through router, you need to change `10.0.0.1` to IP address of the Rover on your local network.
{% endhint %}

If everything works, you should be able to `ping` the Rover by it's hostname. Type `ping master.localnet` to check.

Now, to be connected in ROS network, you need to set some environment variables. Start by sourcing the result space you are using:

```bash
source /opt/ros/melodic/setup.bash
```

Specify the address of the master node:

```bash
export ROS_MASTER_URI=http://master.localnet:11311
```

And your IP on the network:

```bash
export ROS_IP=X.X.X.X
```

Replace `X.X.X.X` with your IP address.

{% hint style="info" %}
You can check your address by typing `ip address`. Search for your wireless network interface and the `inet` keyword.
{% endhint %}

You will need this lines executed at every terminal session you want to use ROS on. To do this automatically at the start of every session, you can add this lines to the `~/.bashrc` file.

You should now be able to do all the things from `the first section` of this tutorial on your computer.

## Examples of ROS use 

Apart from allowing communication between different processes on Raspberry Pi, ROS will give us the possibility to remotely control the Rover on your computer, as well as run graphical tools to introspect and visualize the current state of the Rover. A lot of these tools are available in distribution packages in the form of [rqt](http://wiki.ros.org/rqt) and [rviz](http://wiki.ros.org/rviz) plugins.

Below are some examples possible to do on the stock Leo Rover.

### Introspecting the ROS computation graph 

A [node graph](http://wiki.ros.org/rqt_graph) is an `rqt` plugin that can visualize [ROS computation graph](http://wiki.ros.org/ROS/Concepts#ROS_Computation_Graph_Level). It is a very handy tool for debugging communication problems.

First, make sure you have the plugin installed:

```bash
sudo apt update
sudo apt install ros-melodic-rqt-graph
```

Start `rqt` by typing:

```bash
rqt
```

Now choose **Plugins -&gt; Introspection -&gt; Node Graph**

If your are connected to your Rover, you should see all the nodes running on Raspberry Pi. You can experiment with Node Graph settings, so it can look like this:

![](../../.gitbook/assets/image%20%2850%29.png)

### Visualizing the model  

To visualize the model, you will need 2 additional packages:

1. [leo\_description](https://github.com/LeoRover/leo_description) − contains the URDF model of Leo Rover with all the required mesh files.
2. [leo\_viz](https://github.com/LeoRover/leo_viz) − contains visualization launch files and RViz configurations for Leo Rover.

You can build them using the instructions from [this chapter](https://docs.leorover.tech/development-tutorials/ros-development#building-additional-ros-packages). You can also download the prebuilt packages from the ROS repository by executing:

```bash
sudo apt install ros-<distribution>-leo-viz
```

{% hint style="info" %}
Replace `<distribution>` with the ROS distribution you have installed on your computer \(either `kinetic` or `melodic`\). 
{% endhint %}

Now, to visualize the model in RViz, just type:

```bash
roslaunch leo_viz rviz.launch
```

![](../../.gitbook/assets/image%20%2822%29.png)

Alternatively, you can open a fresh instance of RViz by typing:

```bash
rviz
```

In the **Fixed Frame** option choose `base_link`.  
In **Displays** panel, click **Add** and choose **RobotModel** plugin.  
Change the **Background Color** to make the model more visible.

You should see the wheels rotating when the Rover is being steered.

#### running the visualization offline

You can run the visualization without being connected to the Rover. For this, you will need to change environment variables to point to your loopback device:

```bash
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
```

Then, use the launch file located in the `leo_viz` package:

```bash
roslaunch leo_viz view_model.launch
```

{% hint style="info" %}
[roslaunch](http://wiki.ros.org/roslaunch) will automatically spawn the Master node \([roscore](http://wiki.ros.org/roscore)\) if it detects that it is not already running.
{% endhint %}

An RViz instance with `RobotModel` plugin should start, as well as GUI for [joint\_state\_publisher](http://wiki.ros.org/joint_state_publisher) that let's you specify simulated wheel positions.

### Steering the Rover with a joystick

In this example, we will create a simple package that will let you control the Rover using a joystick connected to your computer.

We will use two nodes that are available in the ROS distribution:

* `joy_node` \(from [joy](http://wiki.ros.org/joy) package\) - for getting input from the joystick and publishing it on a topic.
* `teleop_node` \(from [teleop\_twist\_joy](http://wiki.ros.org/teleop_twist_joy) package\) - for getting messages from the joystick topic and publishing corresponding steering commands to the Rover.

We assume that you have already created a workspace like in the previous example.

Start by creating an empty package with the specified dependencies:

```bash
cd ~/ros_ws/src
catkin create pkg leo_joy_example --catkin-deps joy teleop_twist_joy
```

You might need to install dependent packages first:

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -i
```

Now, add **launch/** and **config/** directories inside your package:

```bash
cd ~/ros_ws/src/leo_joy_example
mkdir launch config
```

Inside **launch/** directory, add the **joy.launch** file with the following content:

{% code title="leo\_joy\_example/launch/joy.launch" %}
```markup
<launch>
  <arg name="cmd_vel_topic" default="cmd_vel"/>

  <node name="joy_node" pkg="joy" type="joy_node">
    <param name="dev" value="/dev/input/js0"/>
    <param name="coalesce_interval" value="0.02"/>
    <param name="autorepeat_rate" value="30.0"/>
  </node>

  <node name="teleop_node" pkg="teleop_twist_joy" type="teleop_node">
    <rosparam command="load" file="$(find leo_joy_example)/config/joy_mapping.yaml"/>
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
  </node>
</launch>
```
{% endcode %}

Inside **config/** directory, add the **joy\_mapping.yaml** file:

{% code title="leo\_joy\_example/config/joy\_mapping.yaml" %}
```yaml
axis_linear: 1
scale_linear: 0.4
axis_angular: 3
scale_angular: 2.0
enable_button: 5
```
{% endcode %}

Now, build the package:

```bash
cd ~/ros_ws
catkin build
source devel/setup.bash
```

Before you start your `launch` file, you might need to remap axes and buttons to suit the joystick you have. Start `joy_node` by typing:

```bash
rosrun joy joy_node
```

And on another terminal, run:

```bash
rostopic echo /joy
```

Move the axes you want to use for the linear and angular movements of the Rover and check which values are being changed on `axes[]` array \(remember that the values are indexed from 0\).

Choose the button that will be used to enable the command publishing. Check which value is being changed on the `buttons[]` array when you click the button.

Now, change the `axis_linear` , `axis_angular`, `enable_button` parameters in **joy\_mapping.yaml** file.

Close the `joy_node` and start your the `joy.launch` file:

```bash
roslaunch leo_joy_example joy.launch
```

You should now be able to steer the Rover by holding down the enable button and moving the joy axes you set.

### Detecting AR Tags

An **AR-tag** is a fiduciary marker system that can help with robot perception challenges, serving as a point of reference for autonomous tasks.

In this example, we will use [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) package for detecting individual markers.

As sending raw images from the camera via wireless network may be insufficient, we will relay all the processing to the Raspberry Pi. 

Start by logging into your Rover via SSH:

{% page-ref page="../../basic-guides/connect-via-ssh.md" %}

Create a workspace in your home directory if you don't have one yet:

```yaml
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin init
catkin config --extend /opt/ros/melodic
```

Create a new package that depends on `ar_track_alvar`:

```yaml
cd ~/ros_ws/src
catkin create pkg leo_alvar_example --catkin-deps ar_track_alvar
```

Run `rosdep` to install dependent package:

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -i
```

Now, add **launch/** and **config/** directories inside your package:

```bash
cd ~/ros_ws/src/leo_alvar_example
mkdir launch config
```

Inside **launch/** directory add **alvar.launch** with the following content:

{% code title="leo\_alvar\_example/launch/alvar.launch" %}
```markup
<launch>
	<arg name="cam_image_topic" default="camera/image_raw" />
	<arg name="cam_info_topic" default="camera/camera_info" />
    
	<node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen">
		<rosparam command="load" file="$(find leo_alvar_example)/config/alvar.yaml" />

		<remap from="camera_image"  to="$(arg cam_image_topic)" />
		<remap from="camera_info"   to="$(arg cam_info_topic)" />
	</node>
</launch>
```
{% endcode %}

Inside **config/** directory add **alvar.yaml** file:

{% code title="leo\_alvar\_example/config/alvar.yaml" %}
```yaml
marker_size: 10.0
max_new_marker_error: 0.08
max_track_error: 0.2
max_frequency: 8.0
output_frame: base_link
```
{% endcode %}

{% hint style="info" %}
You will most likely need to change `marker_size` parameter depending on the actual size of your printed AR tag. You can read more about the parameters on the [package wiki](http://wiki.ros.org/ar_track_alvar#ar_track_alvar.2BAC8-post-fuerte.Detecting_individual_tags).
{% endhint %}

And build the package:

```bash
cd ~/ros_ws
catkin build
source devel/setup.bash
```

To start the Alvar tracking, type:

```bash
roslaunch leo_alvar_example alvar.launch
```

If you want to start the node when the rover boots, add this line to `robot.launch` file:

{% code title="/etc/ros/robot.launch" %}
```bash
<include file="$(find leo_alvar_example)/launch/alvar.launch"/>
```
{% endcode %}

Now, we need to create some markers, so go back to your computer.

Install the `ar_track_alvar` package: 

```bash
sudo apt install ros-melodic-ar-track-alvar
```

And run the `createMarker` script:

```bash
rosrun ar_track_alvar createMarker 0 -s 10.0
```

This will create **MarkerData\_0.png** file that stores a 10cm x 10cm marker with id 0. Print this file on a sheet of paper.

{% hint style="warning" %}
Due to differences in printer setups, the actual size of the printed marker may be different. Make sure the `marker_size` parameter represents the actual size \(in centimeters\) of the AR tag.
{% endhint %}

Now to visualize detected AR Tags, you just need to:

* open RViz, by typing `rviz` in the terminal
* set **Fixed Frame** to `base_link`
* Click **Add** -&gt; **Marker** and set **Marker Topic** to `visualization_marker`
* \(optionally\) Click **Add** -&gt; **RobotModel** to visualize the Rover
* \(optionally\) Click **Add** -&gt; **Image**, set **Image Topic** to `/camera/image_raw` and **Transport Hint** to `compressed` to open the image stream

If all goes well, you should end up with something like this:

{% embed url="https://www.youtube.com/watch?v=QQpz7LU5eJ4&feature=youtu.be" %}

The detected AR Tags are also published to `/ar_pose_marker` topic, so you could use the output in your custom nodes.

