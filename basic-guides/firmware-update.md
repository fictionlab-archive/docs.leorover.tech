# Firmware update

## Flashing the newest firmware

### Through RPi GPIO pins

{% hint style="warning" %}
This method requires LeoOS 0.2 or newer.
{% endhint %}

You can use the [leo\_fw](http://wiki.ros.org/leo_fw) package to flash the newest firmware from Raspberry Pi without connecting any additional cables. To do this, login to the Rover:

{% page-ref page="connect-via-ssh.md" %}

Make sure you have the latest versions of the packages:

```bash
sudo apt update && sudo apt upgrade
```

And then, run the update script by typing:

```bash
rosrun leo_fw update
```

The script will guide you through the flashing process.

### Through USB

#### Connect to the micro USB hSerial port

Make sure that your Raspberry Pi is connected to CORE2 board through a micro USB hSerial port.  
You'll need a USB A &lt;-&gt; USB micro cable provided with the Rover.

#### Download the newest Leo Rover firmware

You can find all firmware versions and the list of changes on our [GitHub releases page](https://github.com/LeoRover/leo_firmware/releases).

Check for the newest version and download the .hex file \(ex. `leo_firmware_v1.0.0.hex`\) to your computer.

#### Upload the firmware to the Rover

Place the firmware .hex file inside `/home/pi` directory of your Rover.

{% page-ref page="upload-files-to-your-rover.md" %}

#### Flash the firmware

{% page-ref page="connect-via-ssh.md" %}

Make sure you are in the home directory \(by default you should be there;`/home/pi`\) and type:

```bash
core2-flasher leo_firmware_[VERSION].hex
```

{% hint style="info" %}
Replace `[VERSION]` with the version you downloaded.
{% endhint %}

The process of flashing should begin. After it completes, you should be good to go.

## Troubleshooting

### Bootloader not flashed

{% hint style="warning" %}
The problem does not occur when using the first method to flash the firmware as the `leo_fw` update script will also flash the bootloader.
{% endhint %}

If your Core2-ROS board has not been previously flashed, you may need to flash the bootloader first for a firmware to work. 

To do this, download the bootloader from [here](https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex) and upload it to your Rover, or if you already connected the Rover to the Internet, just type:

```bash
wget https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex
```

Then, do the following commands:

```bash
core2-flasher --unprotect
core2-flasher bootloader_1_0_0_core2.hex
core2-flasher --protect
```

Now you can proceed with the firmware flashing operation.

### 

