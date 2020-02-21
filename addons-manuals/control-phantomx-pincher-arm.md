# PhantomX Pincher Arm

In this tutorial, we will show you how to configure and remotely control PhantomX Pincher Robot connected to the Rover.

![](../.gitbook/assets/71099672_434779074055562_670313626036338688_n%20%281%29.jpg)



![source: trossenrobotics.com](../.gitbook/assets/image%20%2817%29.png)

## Prerequisites

Before you begin, make sure you have internet connection on your Rover.

{% page-ref page="../software-tutorials/connect-to-the-internet.md" %}

Also, you need to have ROS installed on your computer.

{% page-ref page="../development-tutorials/install-ros-on-your-computer.md" %}

And, of course, assembled Pincher Arm together with ArbotiX-M Robocontroller connected to your Rover. 

## Set servo IDs

To properly communicate with the Dynamixel servos, you will need to set the servo IDs like in the picture below:

![source: trossenrobotics.com](../.gitbook/assets/image%20%282%29.png)

To do this, you can follow our guide for the Arbotix controller here:

{% page-ref page="arbotix-m-robocontroller.md" %}

In there, you will also find how to configure and use the [arbotix ROS driver](http://wiki.ros.org/arbotix).

## Install the ROS driver package

The [turtlebot\_arm](http://wiki.ros.org/turtlebot_arm) packages contain very useful utilities for PhantomX Pincher arm such as:

* configuration for the arbotix driver
* [URDF](http://wiki.ros.org/urdf) model of the arm
* [Moveit!](https://moveit.ros.org) configuration package
* [IKFast](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html) Kinematics solver plugin
* MoveIt! pick and place demo

To use the mentioned features, you need to build the packages and run the driver on your Rover first.

Start by logging into your Rover's console:

{% page-ref page="../software-tutorials/connect-to-the-console-ssh.md" %}

and creating an empty catkin workspace, if you don't have one yet:

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws
catkin init
catkin config --extend /opt/ros/kinetic
```

Clone the packages into the source space:

```bash
cd ~/ros_ws/src
git clone https://github.com/corb555/arbotix_ros.git
git clone https://github.com/turtlebot/turtlebot_arm.git
```

{% hint style="info" %}
To fully support the PhantomX Pincher arm, we clone the [unofficial 0.11.0 release](https://github.com/corb555/arbotix_ros) of the arbotix driver, which adds support for prismatic joints.
{% endhint %}

Install dependencies:

```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -yi
```

Build the workspace:

```bash
catkin build
```

Source the result space:

```bash
source ~/ros_ws/devel/setup.bash
```

To use the packages with PhantomX Pincher arm, set the `TURTLEBOT_ARM1`environment variable to `pincher`.

```bash
export TURTLEBOT_ARM1=pincher
```

{% hint style="info" %}
Add this line to `~/.bashrc` if you don't want to run it on every terminal session you open:

```bash
echo "export TURTLEBOT_ARM1=pincher" >> ~/.bashrc
```
{% endhint %}

Now, you can use [roslaunch](http://wiki.ros.org/roslaunch) to run the bringup launch file:

```bash
roslaunch turtlebot_arm_bringup arm.launch
```

This should run the Arbotix driver, [robot\_state\_publisher](http://wiki.ros.org/robot_state_publisher) and set the`robot_description` parameter to the [URDF model ](https://industrial-training-master.readthedocs.io/en/melodic/_source/session3/Intro-to-URDF.html)of the arm.

You should then see new topics to which you can send position commands:

```bash
/arm_shoulder_pan_joint/command
/arm_shoulder_lift_joint/command
/arm_elbow_flex_joint/command
/arm_wrist_flex_joint/command
/gripper_joint/command
```

as well as services for setting speed and relaxing joints.  
You can learn more about them in the [ArbotiX-M Robocontroller tutorial](https://docs.leorover.tech/addons-manuals/arbotix-m-robocontroller).

The driver will also provide controllers for [FollowJointTrajectory](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html) and [GripperCommand](http://docs.ros.org/melodic/api/control_msgs/html/action/GripperCommand.html) actions \(see the [actionlib wiki](http://wiki.ros.org/actionlib) for more information\).

## Visualize and control the arm with MoveIt

You will need to have ROS installed on your computer and properly configured to communicate with the nodes running on your Rover. To learn how to do this, you an follow **Connecting other computer to ROS network** section of ROS Development tutorial:

{% page-ref page="../development-tutorials/software-development/ros-development.md" %}

To view robot arm model with actual position:

* Open `rviz`
* For `Fixed Frame` select `arm_base_link`
* Click `Add` in Displays panel
* Select `RobotModel` and click `Ok`

![](https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-Lf4v_-a_RwXihZ7ha2W%2F-LgDttR7GgctqvFzocxH%2F-LgDvjkVlD_NFk9J_htk%2Fimage.png?alt=media&token=b53e1368-84b1-4303-8142-ebc58fe47108)

To test Motion Planning with MoveIt! :

* Run `turtlebot_arm_moveit` launch file:

  ```text
  roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=false
  ```

* Click on `Planning` tab in Motion Planning display
* Move interactive marker to intended position
* Click on `Plan` to see Motion visualization and then `Execute` or just click on `Plan and Execute`
* Run pick and place demo \(in another terminal session\)

  ​

  ```text
  rosrun turtlebot_arm_moveit_demos pick_and_place.py
  ```

![](https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-Lf4v_-a_RwXihZ7ha2W%2F-LgDttR7GgctqvFzocxH%2F-LgDvqXKtwW2afxBnYUG%2Fimage.png?alt=media&token=cd32a873-4c06-4595-8a6a-822b6b578efc)

