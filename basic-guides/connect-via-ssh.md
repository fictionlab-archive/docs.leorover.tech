# Connect via SSH

## Prerequisites

Connect to the Rover's WiFi network.

{% page-ref page="../getting-started.md" %}

## Connect via SSH

### Using Putty

Download and install [Putty](https://www.putty.org/) - it will allow you to establish SSH connection with the Rover and open a terminal session.

Open Putty, type `10.0.0.1` as IP address and press `Open`.

![](../.gitbook/assets/image%20%2827%29.png)

The first time you try to connect, you may see the security warning shown below. You can safely ignore it and click the `Yes` button.

![](../.gitbook/assets/image%20%2817%29.png)

When asked for credentials, type:  
login: `pi`  password: `raspberry`

![](../.gitbook/assets/image%20%2863%29.png)

You'll see something like this. You're in!

### Using command line SSH client

If you are using Linux or Mac OS, you most likely have an SSH client already installed. On Windows 10, you can use [this guide](https://docs.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse#installing-openssh-with-powershell) to install OpenSSH.

To connect to your Rover, open a terminal session and type:

```bash
ssh pi@10.0.0.1
```

If you log in for the first time, you might see a similar message:

![](../.gitbook/assets/image%20%2866%29.png)

Type `yes` and enter to continue.

When prompted for password, type: `raspberry`



