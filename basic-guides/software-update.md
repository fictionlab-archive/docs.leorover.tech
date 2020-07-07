# Software update

## Flashing a new Raspberry Pi image

### 1. Access microSD card 

To access microSD card from your Rover, open the main electronics box by unscrewing the 4 socket-headed screws and extract the card from the SD card slot on the Raspberry Pi.

Put the microSD card in the adapter and then connect it to your computer.

### 2. Download the newest LeoOS image

You can find all releases of LeoOS [here](https://github.com/LeoRover/leo_os/releases). Choose the newest one and download the image \(`.img.xz` file\) you want. We recommend the `full` version. The `lite` version comes without a desktop environment, so it is an alternative if you don't need all the extra packages.

### 3. Flash image to microSD card

#### Using Etcher

Open [Etcher](https://www.balena.io/etcher/) and point it to your new SD card image location \(.img or .img.xz file\) via `Select image` . 

Click `Select drive` option and choose your card, then click `Flash!`.

![](../.gitbook/assets/image%20%2835%29.png)

After the flashing completes, disconnect the card and put it back into the Rover.

#### Using command-line tools

If you are using Linux or Mac, you can use `xz` and `dd` tools to copy compressed image file into your card like this:

```bash
xz -d -c [IMAGE_FILE] | sudo dd of=[SD_CARD_DEVICE] bs=4M status=progress
```

example usage:

```bash
xz -d -c LeoOS-0.1-2020-06-29-full.img.xz | sudo dd of=/dev/mmcblk0 bs=4M status=progress
```

{% hint style="danger" %}
`dd` can be dangerous to your system as it can wipe your data if used incorrectly.   
If you are not sure your \[SD\_CARD\_DEVICE\] is correct, we suggest using [Etcher](https://www.balena.io/etcher/) instead.
{% endhint %}

## Updating software

To download newest versions of system packages and ROS packages \(including[ leo\_bringup](https://github.com/LeoRover/leo_bringup) and [leo\_ui](https://github.com/LeoRover/leo_ui.git)\), connect to the Rover's console:

{% page-ref page="connect-via-ssh.md" %}

make sure you are connected to the Internet:

{% page-ref page="connect-to-the-internet.md" %}

and type the following commands:

```bash
sudo apt update
sudo apt upgrade
```

