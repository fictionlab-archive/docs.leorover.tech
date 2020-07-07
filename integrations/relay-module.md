# Relay Module

In this tutorial, we will show you how to configure and remotely control relays via additional user's web interface.

![Relay board ](../.gitbook/assets/20f89e2c6f6df9d5aecdb9c46b559d5478fdd8e2.jpg)

## Items needed

* Leo Rover
* 4 channel relay board 
* IDC plug 2x3
* ribbon cable \(6 cores\)
* 3D printed cover

## Prerequisites

Connect to the Rover through SSH

{% page-ref page="../basic-guides/connect-via-ssh.md" %}

Make sure the Rover is connected to the internet

{% page-ref page="../basic-guides/connect-to-the-internet.md" %}

## 1. Connect the relay board to Core2-ROS

In the tutorial we are using hSens1 port to connect relays to the board.

Connect the relay board to hSens1 port using IDC plug. Pins 1-4 are for signals, 5 is +5V and the last one is GND.

![](../.gitbook/assets/core2_top_small%20%281%29.jpg)

![](../.gitbook/assets/zrzut-ekranu-z-2019-08-08-10-54-15.png)



{% hint style="info" %}
Well done! Hardware is ready
{% endhint %}

## 2. Flash a dedicated firmware to enable hSense1 port

### 1. Download a dedicated firmware

[https://github.com/szlachcic/leo\_firmware\_relay/releases](https://github.com/szlachcic/leo_firmware_relay/releases)

### 2. Upload the downloaded .hex file to your Rover

{% page-ref page="../basic-guides/upload-files-to-your-rover.md" %}

### 3. Flash the firmware

Make sure you are in the home directory \(`/home/pi`\) and type:

```bash
core2-flasher leo_firmware.hex
```

The process of flashing should begin. After it completes, type:

```bash
sudo systemctl restart leo
```

to restart ROS serial node.

## 3. Add user web interface to nginx server

#### 1. Clone a repository that contains a relay interface from GitHub

```text
cd /opt
git clone https://github.com/LeoRover/leo_ui_sample_relay.git
```

{% page-ref page="../development-tutorials/web-ui-development/include-additional-ui.md" %}

## How to control the relays via user interface

#### 1. Connect to the Rover wifi network

#### 2. Open web browser and type 10.0.0.1:90

![](../.gitbook/assets/zrzut-ekranu-z-2019-08-08-14-27-31.png)
