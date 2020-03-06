# WidowX 250

In this tutorial, we will show you how to integrate and remotely control the WidowX 250 robotic arm.

![](../../.gitbook/assets/image%20%2818%29.png)

## Mounting and wiring the arm

The mounting of the arm is particularly easy. If you have bought the arm with the modified support plate designed for our robot, you can use screws and nuts to connect the arm to the rover's mounting plate.

If you have the original support plate, you can get the model for 3D printing here:

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

