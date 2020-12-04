---
description: >-
  This tutorial will show you how to quickly enable the Scratch 3.0 programming
  interface on the Leo Rover.
---

# Scratch 3.0

Author: Piotr Szlachcic `szlachcic@turtlerover.pl`  

## Introduction

We finally did it! Here's the long awaited Scratch integration tutorial for LeoOS. From now on you will be able to program your Rover via a graphical block-based interface and it's as easy as it looks like ;-\)

![The Scratch GUI hosted locally](../.gitbook/assets/leo-scratch.png)

#### Scratch 3.0

Scratch is a block-based visual programming language dedicated to education. It's taught and used in education centers, schools, and colleges, as well as other public knowledge institutions. It is one of the best ways to begin your programming adventure as it doesn't need you to know anything about software to start.

For more information about Scratch in general check this link: [https://scratch.mit.edu/about](https://scratch.mit.edu/about)

#### Scratch 3.0 GUI hosted locally

Normally you would use Scratch with an online editor that is available on their [website](https://scratch.mit.edu/projects/editor/?tutorial=getStarted). But in Leo Rover we'll integrate Scratch in a different way. First of all, we would like to have the possibility of using the editor offline. Second, we need to establish communication between Scratch editor and the Rover ROS layer.

We'll build the Scratch interface locally and install a ROS extension that was developed by ROS community.

![The Scratch GUI hosted locally](../.gitbook/assets/leo-scratch.png)

## Scratch editor installation and configuration

To make the installation process quick and easy, we created a package of necessary dependencies, configuration files, and scripts. So you just need to clone or download the repository from GitHub and run the script. Everything will be done automatically. No worries it will be only a few commands. Let's follow the instruction below.

#### Prerequisites

Open ssh session. It will be necessary to access a remote terminal on Leo Rover.

{% page-ref page="../basic-guides/connect-via-ssh.md" %}

Establish the Internet connection.

{% page-ref page="../basic-guides/connect-to-the-internet.md" %}

{% hint style="warning" %}
It is necessary to establish the Internet connection as some dependencies need to be downloaded during the installation process.
{% endhint %}

#### Clone the repository

Clone the leo\_scratch repository from Leo Rover GitHub page. 

We recommend you clone it to the main directory, ****but doesn't matter where you decide to put it. If you put it somewhere else, pay attention as some of the next commands may require some changes.

```text
cd ~/
git clone https://github.com/LeoRover/leo_scratch.git
```

#### [https://github.com/LeoRover/leo\_scratch](https://github.com/LeoRover/leo_scratch)

#### Run the script

Go into the cloned repository and run the script with root privileges \(sudo\). The default password to access root is `raspberry`. Installation may take up to 10 minutes.

```text
cd ~/leo_scratch
sudo bash run.sh
```

{% hint style="info" %}
During the installation you will probably see some warnings about outdated dependencies. No worries, it is fine :\)
{% endhint %}

After the installation, it's needed to restart the service that was created. Type the command below in the terminal. More information about the services will come later in this tutorial.

```text
sudo systemctl restart scratch.service
```

#### Configuration

To establish proper communication between the Scratch editor and the ROS layer, [rosapi](https://github.com/RobotWebTools/rosbridge_suite/tree/develop/rosapi) node is needed. It provides GUI service calls to allow for getting meta-information related to ROS \(like topic lists\) as well as interacting with the parameter server.

{% hint style="info" %}
The last release of leo\_bringup package doesn't include rosapi node initialization. If your package is not up to date, follow **one** of the optional step.
{% endhint %}

**Option A \(recommended\): Upgrade package list**

```text
sudo apt update
sudo apt upgrade
```

**or Option B: Add rosapi node initialization to the main launch file**

The rosapi node is a part of the rosbridge\_server package installed on the Rover by default. You need to add a few commands to the main launch file to enable rosapi node.

Open  `/etc/ros/robot.launch` and add commands under end &lt;/launch&gt; tag.

```text
<node name="rosapi" pkg="rosapi" type="rosapi_node">
    <param name="topics_glob" value="[*]" />
    <param name="services_glob" value="[*]" />
    <param name="params_glob" value="[*]" />
</node>
```

## Service

To provide control under the scratch editor hosted on the Rover  we created a service called `scratch.service`. By default installation script makes the service enable so the interface is loading each time system is starting. The main task of the service is to launch the interface during system booting. Below you can find some helpful commands to check a status and control the service.

Check the service status

```text
systemct status scratch.service
```

Stop the service

```text
sudo systemctl stop scratch.service
```

Start the service

```text
sudo systemctl start scratch.service
```

Enable the service to start every time system is booting

```text
sudo systemctl enable scratch.service
```

Disable the service to start every time system is booting

```text
sudo systemctl disable scratch.service
```

Restart the service

```text
sudo systemctl restart scratch.service
```

{% hint style="info" %}
If you find any problem with the Scratch editor in the future, start debugging by checking service status first.
{% endhint %}

## GUI

Finally, the installation and configuration of the Scratch editor are done. To load the interface you need to type in a web browser IP address of the rover and the interface port number.

{% hint style="warning" %}
Make sure your device is connected to the rover's access point or the rover is connected to the same network as your device.
{% endhint %}

Scratch editor address \(connection to the access point\)

```text
10.0.0.1:8601
```

![](../.gitbook/assets/zrzut-ekranu-z-2020-11-24-17-40-32.png)

To start creating the program you need to add ROS extension. Click the extension icon in the right bottom corner and choose ROS extension. 

The next step is to connect the Scratch editor with the rosbridge server running on the rover. Click on the icon to connect with rosbridge. Use the rover address as the master IP. 

![](../.gitbook/assets/zrzut-ekranu-z-2020-11-24-17-49-49.png)

Now what you need to do is to start creating cool stuff on the Rover using Scratch. 

## Sample project

We create an example scratch project to help you at the beginning of a Scratch adventure. You can simply load the project in the Scratch interface. Download project file from [here](https://github.com/LeoRover/leo_scratch/blob/master/example/Scratch%20Project.sb3)

![](../.gitbook/assets/leo-scratch.png)

The sample project allows you to control the rover using arrows.

## Support

In case of any trouble with the tutorial or the Scratch, please open a thread in the [forum](https://forum.fictionlab.pl/) or ask me directly `szlachcic@turtlerover.pl`  

Piotr

