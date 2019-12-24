# Firmware update

## Firmware flashing

### Connect to micro USB hSerial port

Make sure that your RaspberryPi is connected to CORE2 board through a micro USB hSerial port.

{% hint style="info" %}
You'll need a USB A - USB micro cable, if you get a 5 cm one you'll be able to leave it connected all the time inside the main electronics box \(MEB\).
{% endhint %}

### Check if there's a firmware file in the filesystem

Type:

```bash
ls
```

If you see any file named 'firmware \(...\) .hex' use this exact name in the next steps.

### \(otherwise\) Download the newest Leo Rover firmware

You can find all firmware versions and their changelogs in our [GitHub releases page](https://github.com/LeoRover/leo_firmware/releases).

Choose the version you want and download the .hex file \(ex. 'leo\_firmware.hex'\) to your computer.

#### Upload the firmware to the Rover

Place the firmware .hex file inside `/home/husarion` directory of your Rover.

{% page-ref page="upload-files-to-your-rover.md" %}

### Flash the firmware

{% page-ref page="connect-to-the-console-ssh.md" %}

Make sure you are in the home directory \(by default you should be there;`/home/husarion`\) and type:

```bash
/opt/husarion/tools/rpi-linux/core2-flasher leo_firmware_name.hex
```

{% hint style="warning" %}
Modify the last phrase in the line above if your file name is different.

For ex. Leo Rover 1.6 firmware for Buehler motors will be named similar to 'leo\_firmware\_v0.5\_buehler.hex' - use the full name in the command.
{% endhint %}

The process of flashing should begin. After it completes, restart the Rover.

{% hint style="success" %}
Done!
{% endhint %}

## Troubleshooting

### Core2-ROS straight out of the box = bootloader not flashed

If your Core2-ROS board has not been previously flashed, you may need to flash the bootloader first for a firmware to work. 

To do this, download the bootloader from [here](https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex) and upload it to your Rover, or if you already connected the Rover to the Internet, just type:

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

### 

