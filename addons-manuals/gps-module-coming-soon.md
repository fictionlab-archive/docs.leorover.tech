# GPS module

## Prerequisites

First what you need is GPS module ;-\) Our code is made to communicate with GPS module via serial and parse NMEA sentence- probably most of the GPS module available on market will be compatible. To make sure that your module will cooperate correctly with our firmware, we recommend you to use the u-blox neo-6m GPS module. Link to[ the datasheet](https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf). In our case, we tested firmware with off the shelf module with u-blox NEO-6M, build-in antenna and data backup battery.



![](../.gitbook/assets/u-blox-neo-6m-gps-module-robotics-bangladesh.jpg)

## Connect the GPS to Husarion

Our default firmware uses the highlighted port by default. If you need it is possible to use hSense port 4 as well but it requires code changes.

![](../.gitbook/assets/core2_top_small%20%282%29.jpg)

Connect the GPS module according to pin description below. The easiest way to connect GPS with Husarion board is to use IDC connector and four jumper cables.

![](../.gitbook/assets/gps_conection.png)

| hSense pin | GPS pin |
| :--- | :--- |
| 1 | floatigg |
| 2 | floating |
| 3 | UART\_RX |
| 4 | UART\_TX |
| 5 | +5V |
| 6 | GND |

Make sure if you connect the module properly. Check it in the picture below.

![](../.gitbook/assets/p1010915.JPG)

### Mounting GPS module on the top of the Rover

We didn't prepare any special holder for GPS- you can design it for yourself ;-\). To take GPS cables out of electronis's box use [Dev-Cover](/@leorover/s/leorover/~/edit/drafts/-LsqzlYjQE4vwsJTgjyn/development-tutorials/dev-covers-for-addons). If you put GPS inside electronic's box it will be the risk that GPS doesn't catch the satellites signal- Rover's frame is like Faraday cage ;-\).

## Turn on GPS funcionality

Open new ssh connection to Rover's console. 

{% page-ref page="../software-tutorials/connect-to-the-console-ssh.md" %}

Make sure that firmware uploaded to Husarion is up to date- GPS functionality is available only in the current version. If you have the previous version of firmware, you will need to flash a new one according to tutorial.

{% page-ref page="../software-tutorials/flash-firmware-to-core2-board.md" %}

To set GPS functionality on or off, you need to send a message to `/gps_enable` topic.

```bash
rostopic pub /gps_enable std_msgs/Bool -- "data: true"
```

Now, you need to reset the board to apply changes. You can do this by turning on and off the whole Rover, or by sending a message to `/core2/reset_board` topic.

```bash
rostopic pub /core2/reset_board std_msgs/Empty
```

After the board reset, new topics should spawn: `/gps_fix` in which GPS are publishing the data.

{% hint style="info" %}
Check if GPS is publishing by typing "rostopic" list in the console and finding gps\_fix topic on the list.
{% endhint %}

Type command in Rover's console to check if GPS is publishing data correctly.

```text
rostopic echo /gps_fix
```

{% hint style="warning" %}
New data on the topic are occurring only if GPS find position \(red LED should blinking\)
{% endhint %}

## Test 

Check if obtained coordinates are correct in Google maps. We tested GPS around our office. The precision was pretty good.

![](../.gitbook/assets/zrzut-ekranu-z-2019-11-04-19-50-56.png)

