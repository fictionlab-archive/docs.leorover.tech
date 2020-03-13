# WidowX 250

![](../../.gitbook/assets/blank-diagram%20%281%29.jpeg)

In this tutorial, we will show you how to integrate and remotely control the WidowX 250 robotic arm.

![](../../.gitbook/assets/image%20%2819%29.png)

## Mounting and wiring the arm

The mounting of the arm is particularly easy. If you have bought the arm with the modified support plate designed for our robot, you can use screws and nuts to connect the arm to the rover's mounting plate.

If you have the original support plate, you can get the model for 3D printing here \(addons section\):

{% page-ref page="../../documentation/3d-printed-parts.md" %}

Use the modified Battery &lt;-&gt; MEB cable, included in the set, to connect the battery to the power socket located the arm.

Last but not least, connect the arm's U2D2 driver to the rover's computer through the miniUSB socket located on the mounting plate.

## Integrating the arm with the system

We need to make sure, the U2D2 device is available at a fixed path on rover's system. To do this, you can add the following rule to `udev`:

{% code title="/etc/udev/rules.d/u2d2.rules" %}
```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL"
```
{% endcode %}

For the rule to take effect, reboot the system or just type:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The device should now be available under `/dev/ttyDXL` path.

To integrate the arm, you will need to build some additional ROS packages. Start by creating a local catkin workspace, if you don't have one yet:

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin config --extend /opt/ros/kinetic
```

The package sources for the arm are available on Github at the [interbotix\_ros\_arms](https://github.com/Interbotix/interbotix_ros_arms) repository. Clone the repository to your source space:

```bash
cd ~/ros_ws/src
git clone https://github.com/Interbotix/interbotix_ros_arms.git
```

On the rover, you will only need the driver node for the arm \(`interbotix_sdk` package\) and the URDF description \(`interbotix_description` package\). To speed up the building process, remove the unwanted packages:

```bash
mv interbotix_ros_arms/interbotix_description ./
mv interbotix_ros_arms/interbotix_sdk ./
rm -rf interbotix_ros_arms
```

Now, use `rosdep` to install any dependent packages:

```markup
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -iry
```

and build the workspace:

```markup
catkin build -j 1
```

Once the packages have been built, you can edit the environmental setup file to point to your result space:

{% code title="/etc/ros/setup.bash" %}
```bash
# source /opt/ros/kinetic/setup.bash
source /home/husarion/ros_ws/devel/setup.bash
```
{% endcode %}

and include the arm's driver in the rover's launch file, by adding these lines:

{% code title="/etc/ros/robot.launch" %}
```markup
<include file="$(find interbotix_sdk)/launch/arm_run.launch">
  <arg name="port"                        value="/dev/ttyDXL"/>
  <arg name="robot_name"                  value="wx250"/>
  <arg name="use_default_rviz"            value="false"/>
  <arg name="use_world_frame"             value="false"/>
  <arg name="use_moveit"                  value="true"/>
  <arg name="arm_operating_mode"          value="position"/>
  <arg name="arm_profile_velocity"        value="131"/>
  <arg name="arm_profile_acceleration"    value="15"/>
  <arg name="gripper_operating_mode"      value="position"/>
  <arg name="use_time_based_profile"      value="false"/>
</include>
```
{% endcode %}

{% hint style="info" %}
You can learn more about the driver's parameters and functionalities at the [interbotix\_sdk README page](https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_sdk).
{% endhint %}

You can also edit the robot's URDF file to connect the arm's base link to the rover's model.

{% code title="/etc/ros/urdf/robot.urdf.xacro" %}
```markup
<link name="wx250/base_link"/>

<joint name="arm_joint" type="fixed">
  <origin xyz="0.043 0 -0.001"/>
  <parent link="base_link"/>
  <child link="wx250/base_link"/>
</joint>
```
{% endcode %}

{% hint style="info" %}
To learn more about what the files under `/etc/ros` are used for and how do they correlate with each other, visit the **Adding additional functionality to the rover** section on **ROS Development** guide:

{% page-ref page="../../development-tutorials/ros-development/" %}
{% endhint %}

That's it! On the next boot, the arm driver node will start together with all the other nodes. You can manually restart the running nodes, by typing:

```bash
sudo systemctl restart leo
```

## Controlling the arm

Now that you have the driver running, you should see new ROS topics and services under the `/wx250` namespace. For a full description of the ROS API, visit the [interbotix\_sdk README page](https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_sdk). You can test some of the features with the `rostopic` and `rosservice` command-line tools:

Retrieve the information about the arm:

```bash
rosservice call /wx250/get_robot_info
```

Publish position command to the elbow joint:

```bash
rostopic pub /wx250/single_joint/command interbotix_sdk/SingleCommand "{joint_name: elbow, cmd: -0.5}"
```

Turn off the torque on the joints:

```bash
rosservice call /wx250/torque_joints_off
```

The [interbotix\_ros\_arms](https://github.com/Interbotix/interbotix_ros_arms) repository contains some packages that will let you control the arm in different ways. To use them on your computer, you will need to have ROS installed:

{% page-ref page="../../development-tutorials/ros-development/install-ros-on-your-computer.md" %}

and properly configured to communicate with the nodes running on the rover. For this, you can visit **Connecting other computer to ROS network** section of the ROS Development tutorial:

{% page-ref page="../../development-tutorials/ros-development/" %}

First, install some prerequisites:

```bash
sudo apt install python-catkin-tools
sudo -H pip install modern_robotics
```

{% hint style="info" %}
The `modern_robotics` python package is required to run the joystick control example.
{% endhint %}

and create a catkin workspace:

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin config --extend /opt/ros/melodic
```

Clone the `interbotix_ros_arms` and `leo_description` repositories into the source space:

```bash
cd ~/ros_ws/src
git clone https://github.com/Interbotix/interbotix_ros_arms.git -b melodic
git clone https://github.com/LeoRover/leo_description.git
```

Install dependencies using the `rosdep` tool:

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -iy
```

and build the workspace:

```bash
catkin build
```

Now, `source` the devel space to make the new packages visible in your shell environment:

```bash
source ~/ros_ws/devel/setup.bash
```

{% hint style="info" %}
You will have to do this at every terminal session you want to use the packages on, so it is convenient to add this line to the `~/.bashrc` file.
{% endhint %}

### Visualizing the model

1. Open RViz by typing `rviz` in the console.
2. Choose `base_link` as the **Fixed Frame**.
3. On the **Displays** panel click on **Add** and choose **RobotModel**.
4. For the **Robot Description** parameter, choose `robot_description`.
5. Add another **RobotModel** display, but for the **Robot Description** parameter choose `wx250/robot_description`.

The effect should look similar to this:

![](../../.gitbook/assets/image%20%288%29.png)

### Planning the trajectory with MoveIt

MoveIt motion planning framework will allow us to plan and execute a collision-free trajectory to the destination pose of the end-effector. To use it, first make sure you have the `use_moveit` parameter for the arm driver set to `true`:

{% code title="/etc/ros/robot.launch" %}
```markup
<arg name="use_moveit" value="true"/>
```
{% endcode %}

On your computer, type:

```bash
roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=wx250 rviz_frame:=wx250/base_link
```

The MoveIt GUI should appear:

![](../../.gitbook/assets/image%20%2827%29.png)

On the **MotionPlanning** panel, click on the **Planning** tab, choose `interbotix_arm` for the **Planning Group** and `<current>` for the **Start State**.

There are some predefined poses which you can choose for the **Goal State**, such as `home`, `sleep` or `upright`. To set the pose manually, navigate to the **DIsplays** panel -&gt; **MotionPlanning** - &gt; **Planning Request** and check `Query Goal State`. You should now be able to manually set the end-effector pose for the goal state.

When the goal state is set, click on the **Plan** button to plan the trajectory \(the simulated trajectory visualization should appear\) and **Execute** to send the trajectory to the driver.

If you want to use the MoveIt capabilities in a Python script or a C++ program, please look at the [interbotix\_moveit\_interface](https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_examples/interbotix_moveit_interface) example. 

### Using joystick to control the arm

The `interbotix_joy_control` example package provides the capability to control the movement of the arm \(utilizing inverse kinematics\) with a PS3 or PS4 joystick. 

To use the package with the arm connected to your rover:

1. Change the parameters for the driver node. The joy control node uses the `pwm` mode for the gripper and is more suited to work with the Time-Based-Profile.  Here are the settings that work well:

   {% code title="/etc/ros/robot.launch" %}
   ```markup
   <arg name="arm_operating_mode"          value="position"/>
   <arg name="arm_profile_velocity"        value="200"/>
   <arg name="arm_profile_acceleration"    value="200"/>
   <arg name="gripper_operating_mode"      value="pwm"/>
   <arg name="use_time_based_profile"      value="true"/>
   ```
   {% endcode %}

2. Modify the `joy_control.launch` file to add the option to not run the driver:

   {% code title="interbotix\_joy\_control/launch/joy\_control.launch" %}
   ```markup
   <arg name="run_arm" default="true"/>

   <include if="$(arg run_arm)" file="$(find interbotix_sdk)/launch/arm_run.launch">
   ```
   {% endcode %}

3. Connect the joystick to your computer. You can find the instructions on the package's [README file](https://github.com/Interbotix/interbotix_ros_arms/blob/master/interbotix_examples/interbotix_joy_control/README.md).
4. Start the `joy_control.launch` file:

   ```bash
   roslaunch interbotix_joy_control joy_control.launch robot_name:=wx250 controller:=ps3 run_arm:=false
   ```

   Change `controller` to `ps4` if you are using a PS4 joystick.

### Using the Python API

Aside from the driver, the `interbotix_sdk` package also provides a Python API for manipulating the arm. It is designed to mainly work with the `position` mode for the arm, `pwm` mode for the gripper and the Time-Based-Profile. For a start, you can set the same parameters for the driver as in the previous example.

There are some example scripts that demonstrate the use of the API at the `interbotix_examples/python_demos` directory.

```bash
cd ~/ros_ws/src/interbotix_ros_arms/interbotix_examples/python_demos
```

The `bartender.py` demo performs some pick, pour and place operations. To run it, first open the file in a text editor and search for this line:

```python
arm = InterbotixRobot(robot_name="wx250s", mrd=mrd)
```

Change `wx250s` to `wx250` and then type on the console:

```bash
python bartender.py
```

{% hint style="warning" %}
Make sure that you are not running any other script that takes control of the arm simultaneously \(e.g. the joy control node\). 
{% endhint %}

If everything went right, you should see the arm in action.

You can check the other files in the directory for more examples. To view the available functions in the API and their documentation, take a look at the [robot\_manipulation.py file](https://github.com/Interbotix/interbotix_ros_arms/blob/master/interbotix_sdk/src/interbotix_sdk/robot_manipulation.py).

