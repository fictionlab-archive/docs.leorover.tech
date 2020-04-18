# Connect to a local network \(+ internet\)

It's highly recommended to connect the Rover to the Internet when you start developing. The main reason: you won't need to jump back and forth with different networks on your local device as the Rover will forward the connection to your device.

You'll be able to say connected to 'LeoRover-XXYYY' \(default\) and access tutorials and documentation stored here.

{% hint style="info" %}
Don't worry, we don't read any data from the Rover. We're not so greedy to spy on you!
{% endhint %}

## Use Network Manager

This graphical way to establish the connection is the easiest way to do. You'll connect using internal RaspberryPi modem, which means you can connect to either 2.4 GHz or 5.8 GHz network of your choice \(even to another Leo Rover\).

### 1. Connect to the Rover through SSH

{% page-ref page="connect-to-the-console-ssh.md" %}

### 2. Access network manager

Then, open the network manager text user interface \(you need root privileges to modify network connections\):

```bash
sudo nmtui
```

![](../.gitbook/assets/image%20%2839%29.png)

Select `Activate a connection` and choose your network from list below.

![](../.gitbook/assets/image%20%281%29.png)

Type in the password when prompted. Click `Esc` to quit.

