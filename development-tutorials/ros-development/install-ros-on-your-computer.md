# Install ROS on your computer

Having ROS on your host machine has many advantages. First of all, it allows you to communicate with nodes running on your Rover, so it is easier to introspect ROS network. Also, it lets you run nodes on your computer, so you can launch part of the node network in your host machine such as graphical interfaces and visualization tools.

There is a couple of ROS installation options depending on your system:

## A. Linux Installation

If you have Linux distribution installed \(either natively or on virtual machine\) you can follow instructions on [ROS wiki](http://wiki.ros.org/ROS/Installation).

ROS repository contains prebuild packages for ubuntu and debian systems.  
Kinetic distribution is available for Ubuntu Xenial \(16.04 LTS\) and Debian Jessie.  
Melodic requires Ubuntu Bionic \(18.04 LTS\) or Debian Stretch.

## B. Windows 10 WSL

Ensure that `Windows Subsystem for Linux` feature is enabled by opening `PowerShell` as Administrator and running: 

```
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
```

Reboot when prompted

Install [Ubuntu 18.04](https://www.microsoft.com/pl-pl/p/ubuntu-1804-lts/9n9tngvndl3q?activetab=pivot:overviewtab) from Microsoft Store and open the application.

You can now follow steps from [ros installation tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu) on ROS wiki.

To run graphical applications, you need to install X server on Windows. We recommend using [VcXsrv](https://sourceforge.net/projects/vcxsrv/). Install it and run with default configuration except for `Native OpenGL` which should be unchecked \(otherwise, `rviz` may fail to launch\)

In your ubuntu session, type:

```bash
echo "export DISPLAY=:0" >> ~/.bashrc
source ~/.bashrc
```

Now, to test it, run `roscore` on one session and `rqt` or `rviz` on another.

## C. Bootable USB drive with Ubuntu

### Download and flash Ubuntu+ROS image to USB drive

We prepared bootable images with Ubuntu and ROS preinstalled. They support 64-bit architecture and both UEFI and Legacy boot. The images can be found [here](http://files.fictionlab.pl/ubuntu_ros_images/). Choose either Ubuntu 16.04 with ROS Kinetic or Ubuntu 18.04 with ROS Melodic.

You will need an USB drive with at least 8GB storage capacity. To flash the image you can follow the same instructions as when flashing Rover image to SD card:

{% page-ref page="../../basic-guides/software-update.md" %}

{% hint style="warning" %}
Keep in mind that this will wipe all your data from the drive.
{% endhint %}

Now boot your computer from USB drive. Choose the first option in `GRUB menu`. User should be automatically logged and desktop environment should appear.

{% hint style="info" %}
Ubuntu system password: `ubuntu`
{% endhint %}

### Extend your root partition to fill the whole drive

This way your root storage will be extended to full storage capacity of USB drive - by default it's limited and would end up filling fast will any additional software installed.

Open terminal \(`Ctrl+Shift+t` shortcut\) and type:

```bash
sudo ./extend-rootfs.sh
```

