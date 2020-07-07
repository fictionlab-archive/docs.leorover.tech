# Firmware update

## Firmware flashing

### Connect to the micro USB hSerial port

Make sure that your Raspberry Pi is connected to CORE2 board through a micro USB hSerial port.  
You'll need a USB A &lt;-&gt; USB micro cable provided with the Rover.

{% hint style="info" %}
If you want to flash the firmware more ofter, buy a short \(5 cm\) one so you'll be able to leave it connected all the time inside the main electronics box \(MEB\).
{% endhint %}

### Download the newest Leo Rover firmware

You can find all firmware versions and their changelogs in our [GitHub releases page](https://github.com/LeoRover/leo_firmware/releases).

Check for the newest version and download the .hex file \(ex. `leo_firmware_v1.0.0.hex`\) to your computer.

#### Upload the firmware to the Rover

Place the firmware .hex file inside `/home/pi` directory of your Rover.

{% page-ref page="upload-files-to-your-rover.md" %}

### Flash the firmware

{% page-ref page="connect-via-ssh.md" %}

Make sure you are in the home directory \(by default you should be there;`/home/pi`\) and type:

```bash
core2-flasher leo_firmware_[VERSION].hex
```

{% hint style="info" %}
Replace `[VERSION]` with the version you downloaded.
{% endhint %}

The process of flashing should begin. After it completes, you should be good to go.

{% hint style="success" %}
Done!
{% endhint %}

## Troubleshooting

### Bootloader not flashed

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

