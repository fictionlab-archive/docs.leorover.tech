---
description: >-
  Here you'll learn how to easily change motor settings with no need to set up
  any programming environment.
---

# Modify motor torque and speed settings

Beginning with [firmware version 1.0.0](https://github.com/LeoRover/leo_firmware) there's a much easier way to modify basic settings of the Rover with no need of building the firmware files. The firmware relies on a parameter file that is addressed by Core2-ROS during the operation.

## Prerequisites

{% page-ref page="../../basic-guides/connect-via-ssh.md" %}

{% page-ref page="../../basic-guides/connect-to-the-internet.md" %}

## Make sure you're running newest firmware and software

### Firmware

{% page-ref page="../../basic-guides/firmware-update.md" %}

### Software

Type in the console:

```text
sudo apt update
sudo apt install --upgrade ros-kinetic-leo-bringup
```

And wait until the install completes.

## Access launch file

Type in the console:

```text
sudo nano /etc/ros/robot.launch
```

You'll access `robot.launch` file that drives the firmware settings.

## Example: Change torque\_limit to 800

Add new line in `robot.launch` file \(before `</launch>`\):

```text
<param name="core2/motors/torque_limit" value="800.0"/>
```

This command will set new value to the motors torque limit. It will be 800 now.

Type `Ctrl+O` and `Enter` to save the modified configuration and `Ctrl+X` to exit the editor.

Restart the Rover for the modifications to work.

## Other parameters

See the parameters section of Leo Firmware readme for a list of different modifications possible. 

{% embed url="https://github.com/LeoRover/leo\_firmware" %}

Alter the launch file line you add to meet the parameter and value of your choice.

```text
<param name="[name-from-the-list]" value="[value-of-the-parameter]"/>
```

{% hint style="info" %}
Have fun playing with the parameters! And remember to share your experiments in the forum.

[https://forum.fictionlab.pl/](https://forum.fictionlab.pl/)
{% endhint %}

