# PhantomX Pincher Arm

In this tutorial, we will show you how to configure and remotely control PhantomX Pincher Robot connected to the Rover.

![](../.gitbook/assets/71099672_434779074055562_670313626036338688_n%20%281%29.jpg)



![source: trossenrobotics.com](../.gitbook/assets/image%20%287%29.png)

## Prerequisites

Before you begin, make sure you have internet connection on your Rover.

{% page-ref page="../software-tutorials/connect-to-the-internet.md" %}

Also, you need to have ROS installed on your computer.

{% page-ref page="../development-tutorials/install-ros-on-your-computer.md" %}

And, of course, assembled Pincher Arm together with ArbotiX-M Robocontroller connected to your Rover. 

## 1. On your computer

### 1.1 Flash ArbotiX-M Robocontroller board

To program Arbotix board, you can follow the [Getting Started Guide](https://learn.trossenrobotics.com/arbotix/7-arbotix-quick-start-guide) from Trossen Robotics.

If everything works, open and upload the following sketch

```text
File -> Sketchbook -> ArbotiX Sketches -> ros
```

### 1.2 Install Arbotix ROS drivers

You can install arbotix ROS packages from repository \(`ros-kinetic-arbotix` package\), but for the pincher gripper to fully work, you need to build an unrealeased [0.11.0 version](https://github.com/corb555/arbotix_ros) which adds support for prismatic joints.

To build it, you can follow these steps:

```bash
source /opt/ros/kinetic/setup.bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/corb555/arbotix_ros
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -iry
catkin build
```

### 1.3 Set Dynamixel IDs

Connect Arbotix-M board to your computer through [FTDI-USB Cable](http://www.trossenrobotics.com/store/p/6406-FTDI-Cable-5V.aspx)

Source your catkin workspace \(either `/opt/ros/kinetic` or `~/ros_ws/devel` depending on how you installed Arbotix ROS drivers\) and run:

```text
arbotix_terminal
```

{% hint style="info" %}
`arbotix_terminal` by default, assumes the board is connected on `/dev/ttyUSB0` port. If it is attached to another device, you can specify it like this: 

```text
arbotix_terminal [PORT]
```

To check which port the device connects to, you can, for instance, run `dmesg -w` \(Ctrl+C to exit\), connect the device and check kernel logs. 

If you are using Ubuntu on `Windows Subsytem for Linux` , you need to open Device Manager and look under Ports for COM port number of the device. `COM[N]` corresponds to `/dev/ttyS[N]` path. \(e.g. COM4 -&gt; /dev/ttyS4\). You might need to run:

```text
 sudo chmod 666 /dev/ttyS[N]
```
{% endhint %}

A terminal prompt should appear. Type `help` for list of commands.

![](../.gitbook/assets/image%20%2822%29.png)

Connect the Dynamixel you want to set id to. Then type `ls` to see id of connected servo and `mv [source] [target]` to change it. For example, when the servo has id 1 and we want to set it to 2, just type `mv 1 2`. 

On PhantomX pincher arm, servo ids should look like this:

![source: trossenrobotics.com](../.gitbook/assets/image%20%281%29.png)

When done, type `Ctrl+C` to exit the terminal

### 1.4 Test Arbotix ROS drivers

We will create a package that tests arbotix driver. Let's start by creating an empty package

```bash
source /opt/ros/kinetic/setup.bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
catkin create pkg arbotix_test --catkin-deps arbotix_python
```

Inside your package, create `config/test.yaml` file with following content \(change port if needed\):

```yaml
port: /dev/ttyUSB0
rate: 15
joints: {
    servo1: {id: 1},
    servo2: {id: 2},
    servo3: {id: 3},
    servo4: {id: 4},
    servo5: {id: 5}
}
```

{% hint style="info" %}
You can set more parameters for each joint, like maximum speed, minimum and maximum angle etc. A brief documentation \(Unfortunately, a little outdated\) can be found [here](http://wiki.ros.org/arbotix_python#Parameters)
{% endhint %}

And `launch/test.launch` with following:

```markup
<launch>
  <node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
    <rosparam file="$(find arbotix_test)/config/test.yaml" command="load" />
  </node>
</launch>
```

Now, build your workspace

```bash
cd ~/ros_ws
catkin build
```

{% hint style="info" %}
From now on, you need to run `source ~/ros_ws/devel/setup.bash` on every terminal session you open \(or add it to `~/.bashrc`\)
{% endhint %}

Run Master node on one terminal

```bash
roscore
```

And on another, run your [launch file](http://wiki.ros.org/roslaunch/XML)

```bash
roslaunch arbotix_test test.launch
```

Now, check available topics and services

```text
rostopic list
rosservice list
```

You should see `command` topics that let you set position \(in radians\) for each servo and `joint_states` topic that informs about position and velocity of each joint. There should also be services that allow enabling, relaxing or setting speed \(in radians/sec\) for each servo.

Let's try to move `servo1` joint. 

First, set speed to a safe value \(0.2 r/s in this case\):

```text
rosservice call /servo1/set_speed 0.2
```

Move servo to a default neutral value:

```text
rostopic pub /servo1/command std_mgs/Float64 -- 0.0
```

{% hint style="info" %}
The maximum angle range for a dynamixel servo is \[-150, 150\] degrees which is equal to approximately \[-2.62, 2.62\] in radians
{% endhint %}

Relax joint:

```text
rosservice call /servo1/relax
```

You can also use `arbotix_gui` to control each joint. Just type:

```bash
arbotix_gui
```

A graphical application should appear:

![](../.gitbook/assets/image%20%284%29.png)

Enable and relax servos by clicking on checkboxes and set position by moving sliders.

#### Summary

Here's a recording of a terminal session in which we do things mentioned in the tutorial up to this point:

[![asciicast](https://asciinema.org/a/2um3pdcf5WVzDLYvs8xI5lo09.svg)](https://asciinema.org/a/2um3pdcf5WVzDLYvs8xI5lo09)

### 1.5 Install turtlebot\_arm packages

The [turtlebot\_arm](http://wiki.ros.org/turtlebot_arm) packages contain very useful utilities for PhantomX Pincher arm such as:

* configuration for arbotix driver
* [URDF](http://wiki.ros.org/urdf) model of the arm
* [Moveit!](https://moveit.ros.org) configuration package
* [IKFast](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html) Kinematics solver plugin
* MoveIt! pick and place demo

Let's build it

```bash
source ~/ros_ws/devel/setup.bash
cd ~/ros_ws/src
git clone https://github.com/turtlebot/turtlebot_arm.git -b kinetic-devel
cd ~/ros_ws
rosdep install --from-paths src -iry
catkin build
```

To use this packages with PhantomX Pincher, you need to have `TURTLEBOT_ARM1` environment variable set to `pincher` . To set it automatically on every terminal session, add `export` command to `.bashrc` file:

```bash
echo "export TURTLEBOT_ARM1=pincher" >> ~/.bashrc
source ~/.bashrc
```

Make sure to have `roscore` running, then type:

```bash
roslaunch turtlebot_arm_bringup arm.launch
```

This should run Arbotix driver, [robot\_state\_publisher](http://wiki.ros.org/robot_state_publisher) and set `robot_description` parameter.

To view robot arm model with actual position:

* Open `rviz`
* For `Fixed Frame` select `arm_base_link`
* Click `Add` in Displays panel
* Select `RobotModel` and click `Ok`

![](../.gitbook/assets/image%20%2819%29.png)

To test Motion Planning with MoveIt! : 

* Run `turtlebot_arm_moveit` launch file:  


  ```bash
  roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=false
  ```

* Click on `Planning` tab in Motion Planning display
* Move interactive marker to intended position
* Click on `Plan` to see Motion visualization and then `Execute` or just click on `Plan and Execute`
* Run pick and place demo \(in another terminal session\)



  ```bash
  rosrun turtlebot_arm_moveit_demos pick_and_place.py
  ```

![](../.gitbook/assets/image%20%2814%29.png)

## 2. On your Rover

### 2.1 Prepare catkin workspace

Let's prepare a catkin workspace for pincher arm. Start by sourcing the workspace you want to extend. If you don't have an existing development workspace, just do:

```bash
source /opt/ros/kinetic/setup.bash
```

Download necessary packages

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/corb555/arbotix_ros.git
git clone https://github.com/turtlebot/turtlebot_arm.git
```

We only need `turtlebot_arm_bringup` and `turtlebot_arm_description` packages, so we can remove the rest

```bash
cd ~/ros_ws/src/turtlebot_arm
mv turtlebot_arm_bringup turtlebot_arm_description ~/ros_ws/src
cd ~/ros_ws/src
rm -rf turtlebot_arm
```

Use `rosdep` to install missing dependencies

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -iry
```

Now, build the workspace

```text
catkin build -j 2
```

### 2.2 Start necessary Nodes

Source your workspace

```bash
source ~/ros_ws/devel/setup.bash
```

For ROS to work on multiple machines, you need to set specific [Environment Variables](http://wiki.ros.org/ROS/EnvironmentVariables):

```bash
export TURTLEBOT_ARM1=pincher
export ROS_IP=10.0.0.1
export ROS_MASTER_URI=http://10.0.0.1:11311
```

{% hint style="info" %}
Add these lines to `~/.bashrc` to set them automatically
{% endhint %}

Now, run Master node on one terminal

```text
roscore
```

And on another, type:

```bash
roslaunch turtlebot_arm_bringup arm.launch
```

### 2.3 Control the arm from your computer

Source your workspace containing turtlebot\_arm packages

```bash
source ~/ros_ws/devel/setup.bash
```

Set required variables

```bash
export TURTLEBOT_ARM1=pincher
export ROS_IP=X.X.X.X
export ROS_MASTER_URI=http://10.0.0.1:11311
```

{% hint style="info" %}
Replace X.X.X.X with your local IP address. To check your address, you can use `ip address` command
{% endhint %}

You can now use the examples we described earlier, e.g.:

```text
arbotix_gui
```

or MoveIt! demo:

```text
roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=false
```



