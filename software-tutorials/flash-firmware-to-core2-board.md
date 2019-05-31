# Flash firmware to CORE2 board

### 1. Check connection to USB hSerial port

Make sure that your Raspberry Pi is connected to CORE2 board through a micro USB hSerial port.

TODO - picture

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

Make sure you are in the home directory \(`/home/husarion`\) and type:

```bash
/opt/husarion/tools/rpi-linux/core2-flasher leo_firmware.hex
```

The process of flashing should begin. After it completes, type:

```bash
sudo systemctl restart leo
```

to restart the ROS serial node.

