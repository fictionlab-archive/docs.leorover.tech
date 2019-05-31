# Change WiFi name & password

## 1. Connect to the Rover console

Instructions on how to do this can be found on this page:

{% page-ref page="connect-to-the-console-ssh.md" %}

## 2. Modify Access Point configuration file

You can use `nano` or `vim` editors to modify the configuration file. To edit the file with `nano`, type:

```bash
sudo nano /etc/hostapd/hostapd.conf
```

An editor interface should appear.

![](../.gitbook/assets/image%20%283%29.png)

Modify `ssid` and `wpa_passphrase` fields, then type `Ctrl+O` and `Enter` to save the modified configuration and `Ctrl+X` to exit the editor.

In order to apply the modifications, you need to restart the access points daemon. To do so, just type:

```bash
sudo systemctl restart hostapd
```

{% hint style="info" %}
In some cases, you might need to restart the Rover 
{% endhint %}

