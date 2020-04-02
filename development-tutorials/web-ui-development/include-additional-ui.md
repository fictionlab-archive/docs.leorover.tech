# Include additional UI

This tutorial describes how to include an additional interface to nginx server on LeoRover. We will show you all operations step by step on the example of adding sample UI- [Make your own UI - Relay switches](/@leorover/s/leorover/~/drafts/-LvFY6eH5l95n1mjfabo/addons-manuals/control-4-relays/sample-ui-relay-funcionality). 

## Add user web interface to nginx server

#### 1. Clone a repository that contains UI to include

```text
cd /opt
git clone https://github.com/LeoRover/leo_ui_sample_relay.git
```

{% hint style="info" %}
If you like to learn how simple UI is build- check this tutoral: [Sample UI- relay funcionality](sample-ui-relay-funcionality.md)
{% endhint %}

#### 2. Add a configuration file for nginx server to use the interface on additional port

Find a directory /etc/nginx/sites-available and create there leo\_ui\_sample\_relay file by copying it from leo\_ui file

```text
cd /etc/nginx/sites-available
sudo cp leo_ui leo_ui_sample_relay
```

Open the created file

```text
sudo nano leo_ui_sample_relay 
```

and make changes in the lines as below

{% hint style="info" %}
You can choose any not registered and not assigned port as you like. For more info check this link [https://en.wikipedia.org/wiki/List\_of\_TCP\_and\_UDP\_port\_numbers](https://en.wikipedia.org/wiki/List_of_TCP_and_UDP_port_numbers)
{% endhint %}

```text
listen 80 default_server;        ==>    listen 90 default_server;
listen [::]:80 default_server;   ==>    listen [::]:90 default_server;

root /opt/leo_ui;                ==>    root /opt/leo_ui_sample_relay;
```

{% hint style="info" %}
Save the file - type ctrl+o 
{% endhint %}

#### 3. Restart nginx service

```text
systemctl restart nginx
```

## 

