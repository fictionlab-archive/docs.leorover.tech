# Connect to the console \(SSH\)

## Prerequisites

Before you can connect to Rover's console, you need to connect to WiFi network. 

{% page-ref page="../introduction/getting-started.md" %}

## on Windows via Putty

Download and install [Putty](https://www.putty.org/) - it will allow you to establish SSH connection with the Rover and open command console.

 Open Putty, type '10.0.0.1' as IP address and press `open`.

![](../.gitbook/assets/image%20%282%29.png)

The first time you try to connect, you may see the security warning shown below. You can safely ignore it and click the `Yes` button.

![](../.gitbook/assets/image%20%281%29.png)

When asked for credentials, type:  
login: `husarion`  password: `husarion`

![](../.gitbook/assets/image%20%283%29.png)

You'll see something like this. You're in!

## on Windows 10 via OpenSSH

Open `run` command line \(⊞ Win + X and click `Run` or ⊞ Win + R\)

Type

```text
cmd
```

and then type in the command window:

```bash
ssh husarion@10.0.0.1
```

password: `husarion`

## on Linux or Mac

Access command console and enter:

```bash
ssh husarion@10.0.0.1
```

password: `husarion`



