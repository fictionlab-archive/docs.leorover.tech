# Software update/flashing

## Flashing new Raspberry Pi image

The newest Raspberry Pi image for Leo Rover was released on `7th July 2019`

### 1. Access microSD card 

To access microSD card from your Rover, open the main electronics box \(4 socket-headed screws\) and extract the card from white-tape SD-slot extension.

Put the microSD card in the adapter and then connect it to your computer.

### 2. Download newest Leo Rover image

You can find all of our images [here](http://files.fictionlab.pl/leo_images/). To download the newest one, just choose `leo.img.xz` file.

### 3. Flash image to microSD card

#### On Windows

Open [Etcher](https://www.balena.io/etcher/) and point it to your new SD card image location \(.img or .img.xz file\) via `Select image` . 

Click `Select drive` option and choose your card, then click `Flash!`.

![](../.gitbook/assets/image%20%289%29.png)

After the flashing completes, disconnect the card and put it back into the Rover.

#### On Linux or Mac

You can use `xz` and `dd` tools to copy compressed image file into your card like this:

```bash
xz -d -c [IMAGE_FILE] | sudo dd of=[SD_CARD_DEVICE] bs=4M status=progress
```

example usage:

```bash
xz -d -c leo_2019-05-17.img.xz | sudo dd of=/dev/mmcblk0 bs=4M status=progress
```

{% hint style="danger" %}
`dd` can be dangerous to your system as it can wipe your data if used incorrectly.   
If you are not sure your \[SD\_CARD\_DEVICE\] is correct, we recommend using [Etcher](https://www.balena.io/etcher/)
{% endhint %}

## Updating software

To download newest versions of system packages and ROS packages \(including[ leo\_bringup](https://github.com/LeoRover/leo_bringup)\), connect to the Rover's console:

{% page-ref page="connect-to-the-console-ssh.md" %}

make sure you are connected to the Internet:

{% page-ref page="connect-to-the-internet.md" %}

and type the following commands:

```bash
sudo apt update
sudo apt upgrade
```

#### updating Web UI

```bash
cd /opt/leo_ui
git pull
```

