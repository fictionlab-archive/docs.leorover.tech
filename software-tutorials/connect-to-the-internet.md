# Connect to the Internet

## A. Using Network Manager

First, connect to the Rover through SSH

{% page-ref page="connect-to-the-console-ssh.md" %}

Then, open the network manager text user interface \(you need root privileges to modify network connections\):

```bash
sudo nmtui
```

![](../.gitbook/assets/image%20%2824%29.png)

Select `Activate a connection` and choose your network from list below.

![](../.gitbook/assets/image.png)

Type in the password when prompted. Click `Esc` to quit.

## B. Using husarion hConfig

Press and hold hCfg button on CORE2 board until LR1 and LR2 LEDs start flashing yellow and blue.

#### Using your computer

Open your web browser and type in the address bar:

```http
http://192.168.50.1:8600
```

![](../.gitbook/assets/image%20%2828%29.png)

Click `Connect to Wi-Fi network`.Choose your network, type the password and click `Continue`.

Click on `Save settings`. The Rover should then automatically connect to your network.

#### Using your phone

If you have an [Android](https://play.google.com/store/apps/details?id=com.husarion.configtool2) or [IOS](https://itunes.apple.com/pl/app/hconfig/id1283536270?l=pl&mt=8) device, you can use `hConfig` app. 

The app will guide you through all the necessary steps.

