# Connect to a local network \(+ internet\)

When you add the Rover to your local network that provides Internet connection, you'll be able not only to download files to the Rover, but as well forward the access to your computer when you're connected to the Rover access point. Totally useful. 

{% hint style="info" %}
Don't worry, we don't read any data from the Rover. We're not so greedy to spy on you!
{% endhint %}

## Use Network Manager

This graphical way to establish the connection is the easiest way to do. You'll connect using internal RaspberryPi modem, which means you can connect to either 2.4 GHz or 5.8 GHz network of your choice \(even to another Leo Rover\).

### Connect to the Rover through SSH

{% page-ref page="connect-to-the-console-ssh.md" %}

### Access network manager

Then, open the network manager text user interface \(you need root privileges to modify network connections\):

```bash
sudo nmtui
```

![](../.gitbook/assets/image%20%2839%29.png)

Select `Activate a connection` and choose your network from list below.

![](../.gitbook/assets/image%20%281%29.png)

Type in the password when prompted. Click `Esc` to quit.

