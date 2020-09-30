# Include an additional UI

This tutorial describes how to include an additional UI to be running on the LeoRover. We will show you all the needed steps to include a new interface and reconfigure the HTTP server. 

{% hint style="info" %}
The tutorial will be based on the sample UI dedicated to controlling the additional relay module. You can design your own UI on the basis of [Make your own UI ](sample-ui-relay-funcionality.md)tutorial.
{% endhint %}

## Clone the UI files

The first step is to download the directory containing user interface files. Clone the additional UI from our GitHub repository. We highly recommended placing the additional UI folder in the home directory.

```text
cd ~
git clone https://github.com/LeoRover/leo_ui_sample_relay.git
```

## Configure the nginx service

The next step is to create the configuration file for the HTTP service. The file contains the path to the UI folder and the port number on which the UI will be available.

‌Create a new configuration file based on the default Leo UI configuration file. You can use the `cp` command to clone the file inside the `/etc/nginx/sites-available` folder. By default, when logging as `pi` user, you don't have permission to write files outside of the home directory. You must do all operations as a 'root' user.

```text
cd /etc/nginx/sites-available
sudo cp leo_ui leo_ui_sample
```

The nginx service is searching for the configuration file in `/etc/nginx/sites-enable.` To make nginx load your configuration at start, you need to add a symbolic link in the `sites-enabled` folder, pointing to the configuration file in `sites-available` folder.

```text
sudo ln -s /etc/nginx/sites-available/leo_ui_sample /etc/nginx/sites-enabled/
```

In case of creating the custom configuration, you must change settings in the cloned file in the `/etc/nginx/sites-available`. Use the nano text editor to make some changes. Don't forget to open the file as the root user.

```text
sudo nano leo_ui_sample
```

There are two params to change, the port number and the path to the root website's directory. Make changes according to the code block below. 

{% hint style="warning" %}
Chose a port not assigned to any other running service
{% endhint %}

```text
listen 80 default_server;        ==>    listen 90 default_server;
listen [::]:80 default_server;   ==>    listen [::]:90 default_server;

root /opt/leo_ui;                ==>    root /home/pi/leo_ui_sample_relay;
```

{% hint style="info" %}
Save the file - `CTRL+o`

Close the text editor-`CTRL+x` 
{% endhint %}

At the end of the whole process, an nginx service restart is needed. It will make the new interface available.

```text
sudo systemctl restart nginx
```

## 

