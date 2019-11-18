# ROS Development

> The Robot Operating System \(ROS\) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. - [https://www.ros.org/about-ros/](https://www.ros.org/about-ros/)

In the simplest terms, ROS will give us the possibility to write and run different processes \(called [nodes](http://wiki.ros.org/Nodes)\) that communicate with each other by sending and receiving messages on named buses \(called [topics](http://wiki.ros.org/Topics)\) or by calling remote procedures \(called [services](http://wiki.ros.org/Services)\).

This section will describe some basic ROS functionality that can be accomplished with stock Leo Rover.

## Introspecting ROS network with command line tools

ROS comes with some command line tools that can help to introspect the current network of running nodes. Some of the available tools are:

* [rosnode](http://wiki.ros.org/rosnode) - printing information about currently running nodes, killing them, testing connectivity,
* [rostopic](http://wiki.ros.org/rostopic) - listing and printing information about topics currently in use, printing published messages, publishing data to topics, finding a type of published messages
* [rosservice](http://wiki.ros.org/rosservice) - listing and printing information about available services, calling the service with provided arguments,
* [rosmsg](http://wiki.ros.org/rosmsg#rosmsg-1) - displaying the fields of a specified ROS message type

Let's try to run some examples. Before that, connect to the Rover's console:

{% page-ref page="../../software-tutorials/connect-to-the-console-ssh.md" %}

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

## Building leo\_bringup and starting the nodes manually 

ROS source code is divided into packages that are build using [catkin](http://wiki.ros.org/catkin) build system. Catkin packages can be built as a standalone project, in the same way that normal CMake projects can be built, but catkin also provides the concept of [workspaces](http://wiki.ros.org/catkin/workspaces). 

When building a catkin workspace, the install targets are placed into [FHS compliant](https://www.ros.org/reps/rep-0122.html) hierarchy inside the [result space](http://wiki.ros.org/catkin/workspaces#Result_space). A set of [environment setup files](http://wiki.ros.org/catkin/workspaces#Environment_Setup_File) allow extending your shell environment, so that you can find and use any resources that have been installed to that location.

{% hint style="info" %}
The prebuilt ROS packages \(installed from repository\) are placed into `/opt/ros/distribution_name` directory \(`/opt/ros/kinetic` in this case\). To use the environment setup file, just type:

```bash
source /opt/ros/kinetic/setup.bash
```

If you use Leo image, this line is already added to `~/.bashrc` file, so it will be automatically executed when you log into the console.
{% endhint %}

The catkin build system also supports an [overlay](http://wiki.ros.org/catkin/workspaces#Overlays) mechanism, where one workspace can extend another result space. An `environment setup file` from the result space of such workspace will extend your shell environment by packages from both workspaces.

The build system provides a [catkin\_make](http://wiki.ros.org/catkin/commands/catkin_make) command for building workspaces, but we will use `catkin` command line tool from Python package [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/) as it delivers more user-friendly and robust environment for building catkin packages.

In this chapter, will will try to:

* create workspace that extends `kinetic` result space
* add `leo_bringup` to this workspace and build the package
* run the startup nodes manually
* modify the nodes that are started at boot

Let's start by creating an empty workspace inside home directory on Raspberry Pi:

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin init
```

We want this workspace to extend the prebuilt packages that are already installed on the system. It should be automatically done if you have already sourced `/opt/ros/kinetic/setup.bash` file, but we can also explicitly point out the directory to extend:

```bash
catkin config --extend /opt/ros/kinetic
```

Now, we need to get the sources of `leo_bringup` package:

```bash
cd src
git clone https://github.com/LeoRover/leo_bringup.git
```

and build the workspace:

```bash
cd ~/ros_ws
catkin build
```

If everything works, a [development space](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space) should be created inside `devel` directory. Let's source the environment setup file inside it:

```bash
source devel/setup.bash
```

Now, when you execute `rospack list`, you should see all of the packages installed on your system, but `rospack find leo_bringup` should point you to the directory on your newly created workspace.

The `leo_bringup` package contains a `launch file` that is started by `leo` service at boot.

{% hint style="info" %}
A [launch file](http://wiki.ros.org/roslaunch/XML) can describe a set of nodes to be stared with specified parameters in XML format which can be interpreted with [`roslaunch`](http://wiki.ros.org/roslaunch) tool.
{% endhint %}

To start the nodes manually, you should stop the service first:

```bash
sudo systemctl stop leo
```

Then, execute the `roslaunch` tool like this:

```bash
roslaunch leo_bringup leo_bringup.launch
```

This should start all the nodes that are normally running on Leo Rover.  
You can tweak the launch file to suit your needs more.

If you want your modified launch file to be started at boot time, you need to disable `leo` service first:

```bash
sudo systemctl disable leo
```

Then, run `install` script from [robot\_upstart](http://wiki.ros.org/robot_upstart) like this:

```bash
rosrun robot_upstart install --job leo-custom --user root --setup /home/husarion/ros_ws/devel/setup.bash --symlink leo_bringup/launch/leo_bringup.launch
```

{% hint style="info" %}
You can change `leo-custom` to your custom name, but don't call it `leo` to avoid conflict with already existing service.
{% endhint %}

The service should start when the Rapsberry Pi boots again. To start it now, you can type:

```bash
sudo systemctl daemon-reload && sudo systemctl start leo-custom
```

## Connecting other computer to ROS network 

ROS is designed with distributed computing in mind. The nodes make no assumption about where in the network they run. Configuring your computer to be able to communicate with ROS network will let you run nodes that interfere with the Rover's hardware, as well as graphical tools \(like rqt or rviz\) directly on your host machine. 

To install ROS on your computer, you can follow this tutorial:

{% page-ref page="../install-ros-on-your-computer.md" %}

In this section we will assume, you run Ubuntu 18.04 with ROS Melodic.

First, connect your computer to the same network your Rover is connected. It can be either the Rover's Access Point \(`LeoRover-XXYYY` by default\) or an external router \(if you followed `Connect to the Internet` tutorial\).

To properly communicate over the ROS network, you need to be able to resolve husarion hostname. Type:

```bash
getent hosts husarion
```

If you don't see any output, that means you cannot resolve the hostname.

If you are connected to Rover's Access Point, you should be able to resolve it, but if there is and issue with DNS server on the Rover or you are connected through external router, add this line to `/etc/hosts` file on you computer:

```bash
10.0.0.1 husarion
```

{% hint style="warning" %}
If you are connected through router, you need to change `10.0.0.1` to IP address of the Rover on your local network.
{% endhint %}

If everything works, you should be able to `ping` the Rover by it's hostname. Type `ping husarion` to check.

Now, to be connected in ROS network, you need to set some environment variables. Start by `sourcing` the result space you are using:

```bash
source /opt/ros/melodic/setup.bash
```

Specify the address of the master node:

```bash
export ROS_MASTER_URI=http://husarion:11311
```

And your IP on the network:

```bash
export ROS_IP=X.X.X.X
```

Replace `X.X.X.X` with your IP address.

{% hint style="info" %}
You can check your address by typing `ip address`. Search for your wireless network interface and the `inet` keyword.
{% endhint %}

You will need this lines executed at every terminal session you want to use ROS on. To do this automatically at the start of every session, you can add this lines to `~/.bashrc` file.

You should now be able to do all the things from `the first section` of this tutorial on your computer.

## Examples of ROS use 

Apart from communicating different processes on Raspberry Pi, ROS will give us the possibility to remotely control the Rover on your computer, as well as run graphical tools to introspect and visualize what is currently happening. A lot of these tools are available in distribution packages in the form of [rqt](http://wiki.ros.org/rqt) and [rviz](http://wiki.ros.org/rviz) plugins.

Below are some examples possible to do on stock Leo Rover.

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

![a placeholder screenshot \[TODO\]](../../.gitbook/assets/image%20%286%29.png)

### Visualizing the model  

To visualize the model, you need to build [leo\_description](https://github.com/LeoRover/leo_description) package first.  
Start by creating a new local workspace if you don't have one yet:

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws
catkin init
catkin config --extend /opt/ros/melodic
```

{% hint style="info" %}
If `catkin` command is not found on your system, you might need to install `catkin-tools` package:

```bash
sudo apt install python-catkin-tools
```
{% endhint %}

Add the package to source space:

```bash
cd ~/ros_ws/src
git clone https://github.com/LeoRover/leo_description.git
```

Then build the workspace and `source` the result space:

```bash
cd ~/ros_ws
catkin build
source devel/setup.bash
```

If the package was built successfully, the command:

```bash
rospack find leo_description
```

should return the correct path to the package.

Now, open RViz by typing:

```bash
rviz
```

In the **Fixed Frame** option choose `base_link`.  
In **Displays** panel, click **Add** and choose **RobotModel** plugin.

![](../../.gitbook/assets/image%20%2815%29.png)

You should see the wheels rotating when steering the Rover.

#### running the visualization offline

You can run the visualization without being connected to the Rover. For this, you will need to change environment variables to point to your loopback device:

```bash
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
```

Then, use the launch file located in the package:

```bash
roslaunch leo_description display.launch gui:=true
```

{% hint style="info" %}
[roslaunch](http://wiki.ros.org/roslaunch) will automatically spawn [roscore](http://wiki.ros.org/roscore) if it detects that it is not already running.
{% endhint %}

{% hint style="warning" %}
If you get error messages about missing packages, you might need to run [rosdep](http://wiki.ros.org/rosdep) to install them.

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -i
```
{% endhint %}

An RViz instance with `RobotModel` plugin should start, as well as GUI for [joint\_state\_publisher](http://wiki.ros.org/joint_state_publisher) that let's you specify simulated wheel rotation.

### Steering the Rover with a joystick

In this example, we will create a simple package that will let you control the Rover using joystick connected to your computer.

We will use two nodes available in distribution:

* `joy_node` \(from [joy](http://wiki.ros.org/joy) package\) - getting input from the joystick and publishing it to a topic.
* `teleop_node` \(from [teleop\_twist\_joy](http://wiki.ros.org/teleop_twist_joy) package\) - getting messages from joystick topic and publishing appropriate steering commands to the Rover.

We assume you have already created a workspace like in a previous example.  
Create an empty package with specified dependencies:

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

Inside **launch/** directory add **joy.launch** with the following content:

```markup
<launch>
  <node name="joy_node" pkg="joy" type="joy_node">
    <param name="dev" value="/dev/input/js0"/>
    <param name="coalesce_interval" value="0.02"/>
    <param name="autorepeat_rate" value="30.0"/>
  </node>

  <node name="tr_teleop_joy" pkg="tr_teleop" type="tr_teleop_joy">
    <rosparam command="load" file="$(find leo_joy_example)/config/joy_mapping.yaml"/>
  </node>
</launch>

```

Inside **config/** directory add **joy\_mapping.yaml** file:

```yaml
axis_linear: 1
scale_linear: 0.4
axis_angular: 3
scale_angular: 1.0
```

And build the package:

```bash
cd ~/ros_ws
catkin build
source devel/setup.bash
```

Before you start your `launch` file, you might need to remap axes to suit the joystick you have. Start `joy_node` by typing:

```bash
rosrun joy joy_node
```

And on another terminal, run:

```bash
rostopic echo /joy
```

Move the axes you want to map and check which values are being changed on `axes[]` array \(remember that the values are indexed from 0\).

Now, change `axis_linear` and `axis_angular` parameters in **joy\_mapping.yaml** file.

Close the `joy_node` and start your the `joy.launch` file:

```bash
roslaunch leo_joy_example joy.launch
```

You should now be able to steer the Rover with the joy axes you set.

### Detecting AR Tags

An **ARTag** is a fiduciary marker system that can help with robot perception challenges, serving as a point of reference for autonomous tasks.

In this example, we will use [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) package for detecting individual markers.

As sending raw images from the camera via wireless network may be insufficient, we will relay all the processing to the Raspberry Pi.   
Start by logging into your Rover's console:

{% page-ref page="../../software-tutorials/connect-to-the-console-ssh.md" %}

Create a workspace in your home directory if you don't have one yet:

```yaml
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin init
catkin config --extend /opt/ros/kinetic
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

Inside **config/** directory add **alvar.yaml** file:

```yaml
marker_size: 10.0
max_new_marker_error: 0.08
max_track_error: 0.2
max_frequency: 8.0
output_frame: base_link
```

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

* open RViz, by typing `rviz` in console
* set **Fixed Frame** to `base_link`
* Click **Add** -&gt; **Marker** and set **Marker Topic** to `visualization_marker`
* \(optionally\) Click **Add** -&gt; **RobotModel** to visualize the Rover
* \(optionally\) Click **Add** -&gt; **Image**, set **Image Topic** to `/camera/image_raw` and **Transport Hint** to `compressed` to open the image stream

If all goes well, you should end up with something like this:

{% embed url="https://www.youtube.com/watch?v=QQpz7LU5eJ4&feature=youtu.be" %}

The detected AR Tags are also published to `/ar_pose_marker` topic, so you could use the output in your custom nodes.

