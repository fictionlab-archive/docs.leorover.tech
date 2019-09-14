# Firmware update

{% hint style="info" %}
If your Core2-ROS board has not been previously flashed, you may need to flash the bootloader first for a firmware to work.

To do that - go to 'troubleshooting' section.
{% endhint %}

### 1. Check connection to USB hSerial port

Make sure that your Raspberry Pi is connected to CORE2 board through a micro USB hSerial port.

### 2. Download the newest Leo Rover firmware

You can find all firmware versions and their changelogs on our [GitHub releases page](https://github.com/LeoRover/leo_firmware/releases).

Choose the version you want and download `leo_firmware.hex` file to your computer

### 3. Upload the firmware to the Rover

Place the `leo_firmware.hex` file inside home directory of your Rover.

{% page-ref page="upload-files-to-your-rover.md" %}

### 4. Establish SSH connection

This part is already covered in our previous tutorial:

{% page-ref page="connect-to-the-console-ssh.md" %}

### 5. Flash the firmware

Make sure you are in the home directory \(by default you should be there;`/home/husarion`\) and type:

```bash
sudo /opt/husarion/tools/rpi-linux/core2-flasher leo_firmware.hex
```

{% hint style="warning" %}
Modify the last phrase in the line above if your file name is different.
{% endhint %}

The process of flashing should begin. 

After it completes, type:

```bash
sudo systemctl restart leo
```

to restart the ROS serial node.

### Troubleshooting

#### bootloader not flashed

If your Core2-ROS board has not been previously flashed, you may need to flash the bootloader first for a firmware to work. 

To do this, download the bootloader from [here](https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex) and upload it to your Rover, or if you already connected the Rover to the Internet, just type:

```bash
wget https://files.husarion.com/bootloader/bootloader_1_0_0_core2.hex
```

Then, do the following commands:

```bash
sudo /opt/husarion/tools/rpi-linux/core2-flasher --unprotect
/opt/husarion/tools/rpi-linux/core2-flasher bootloader_1_0_0_core2.hex
/opt/husarion/tools/rpi-linux/core2-flasher --protect
```

Now you can proceed with the firmware flashing operation.

