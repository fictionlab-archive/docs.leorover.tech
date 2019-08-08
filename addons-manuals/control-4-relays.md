# Control 4 relays

In this tutorial, we will show you how to configure and remotely control relays via additional user's web interface.

![relays board ](../.gitbook/assets/20f89e2c6f6df9d5aecdb9c46b559d5478fdd8e2.jpg)

## Necessary items

* assembled LeoRover with Husarion board
* 4 channels relay's board 
* IDC plug 2x3
* ribbon cable \(6 cores\)
* 3D printed cover

## Prerequisites

First- connect to the Rover through SSH

{% page-ref page="../software-tutorials/connect-to-the-console-ssh.md" %}

Before you begin, make sure you have internet connection on your Rover.

{% page-ref page="../software-tutorials/connect-to-the-internet.md" %}

## 1. Connect relay's board to Husarion

In our tutorial we are using hSense1 port to connect relays to the board but in your project you can choose any hSense port.

![](../.gitbook/assets/core2_top_small%20%281%29.jpg)

![](../.gitbook/assets/zrzut-ekranu-z-2019-08-08-10-54-15.png)

According to the scheme above, connect relay's board to hSense1 port using IDC plug. Pins 1, 2, 3, 4 are for signals, 5 is +5V and the last one is GND.

{% hint style="info" %}
Well done! Hardware is ready
{% endhint %}

## 2. Flash firmware into Husarion

#### 1. Download dedicated firmware with additional functionality of handling relays. 

[https://github.com/szlachcic/leo\_firmware\_relay/releases](https://github.com/szlachcic/leo_firmware_relay/releases)

#### 2. Upload dowloaded .hex file to your Rover 

{% page-ref page="../software-tutorials/upload-files-to-your-rover.md" %}

#### 3. Flash the firmware

Make sure you are in the home directory \(`/home/husarion`\) and type:

```bash
/opt/husarion/tools/rpi-linux/core2-flasher leo_firmware.hex
```

The process of flashing should begin. After it completes, type:

```bash
sudo systemctl restart leo
```

to restart the ROS serial node.

#### 4. Troubleshooting- bootloader not flashed

If you happened to receive a not-previously-flashed CORE2 board, you might need to flash the bootloader first for a firmware to work. 

To do this, download the bootloader from [here](https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex) and upload it to your Rover, or if you connected the Rover to the Internet, just type:

```bash
wget https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex
```

Then, do the following commands:

```bash
/opt/husarion/tools/rpi-linux/core2-flasher --unprotect
/opt/husarion/tools/rpi-linux/core2-flasher bootloader_1_0_0_core2.hex
/opt/husarion/tools/rpi-linux/core2-flasher --protect
```

Now you can proceed with the firmware flashing operation.

## 3. Attach additional user's web interface

#### 1. Clone repository containing relay's interface from GitHub

```text
cd /opt
git clone https://github.com/szlachcic/leo_ui_relay.git
```

#### 2. Add configuration file for nginx server to use interface on port 90

First, find directory /etc/nginx/sites-available and create there leo\_ui\_relay file by copying it from leo\_ui

```text
cd /etc/nginx/sites-available
sudo cp leo_ui leo_ui_relay
```

Open created file as root to make changes in server configurations

```text
sudo nano leo_ui_relay 
```

Make changes according to schema below

```text
listen 80 default_server;        ==>    listen 90 default_server;
listen [::]:80 default_server;   ==>    listen [::]:90 default_server;

root /opt/leo_ui;                ==>    root /opt/leo_ui_relay;
```

{% hint style="info" %}
Do not forgot about saving file by typing ctrl+o
{% endhint %}

#### 3. Restart nginx service

```text
systemctl restart nginx
```

## How to control relays via user interface

#### 1. Connect to the Rover Wifi

#### 2. Open web browser and type 10.0.0.1:90 

![](../.gitbook/assets/zrzut-ekranu-z-2019-08-08-14-27-31.png)

