# Software update/flashing

## 1. Access microSD card 

To access microSD card from your Rover, open the main electronics box \(4 socket-headed screws\) and extract the card from white-tape SD-slot extension.

Put the microSD card in the adapter and then connect it to your computer.

## 2. Download newest LeoRover image

You can download the newest LeoRover image here:

{% embed url="https://drive.google.com/file/d/1bvsTQ5c-kzSewyeoj8cQ4t2hUqLxTYa3/view" %}

## 3. Flash image to microSD card

### On Windows

Open [Etcher](https://www.balena.io/etcher/) and point it to your new SD card image location \(.img or .img.xz file\) via `Select image` . 

Click `Select drive` option and choose your card, then click `Flash!`.

![](../.gitbook/assets/image%20%285%29.png)

After the flashing completes, disconnect the card and put it back into the Rover.

### On Linux or Mac

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

