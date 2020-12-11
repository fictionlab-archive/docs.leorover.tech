# Connect to a local network and the Internet

When you connect the Rover to your local network that provides Internet connection, you'll be able to download files to the Rover, as well as forward the Internet to your computer when you're connected to the Rover's access point.

The LeoOS uses [NetworkManager](https://en.wikipedia.org/wiki/NetworkManager) to manage the Raspberry Pi's internal Wifi. The Wifi interface can connect to both 2.4 GHz and 5 GHz networks. 

There are 2 easy ways to connect to and access point:

### Using a text interface

Connect to the Rover through SSH:

{% page-ref page="connect-via-ssh.md" %}

Then, open the NetworkManager Text User Interface by typing:

```bash
nmtui
```

![](../.gitbook/assets/image%20%2839%29.png)

Select `Activate a connection` and choose your network from list below.

![](../.gitbook/assets/image%20%281%29.png)

Type in the password when prompted. Click `Esc` to quit.

### Using a graphical interface

Use remote desktop to connect to the Rover:

{% page-ref page="connect-via-remote-desktop.md" %}

Click on the NetworkManager applet icon on the systray and choose your network \(you might need to expand the `More networks` list\).

![](../.gitbook/assets/image%20%2860%29.png)

When prompted, type in the password and click `Connect`.

