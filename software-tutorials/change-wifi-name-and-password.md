# Change WiFi name & password

## Prerequisites

{% page-ref page="connect-to-the-console-ssh.md" %}

## Modify the access point configuration file

Modify the 'hostapd.conf' configuration file by typing:

```bash
sudo nano /etc/hostapd/hostapd.conf
```

An editor interface should appear.

![](../.gitbook/assets/image%20%2812%29.png)

Modify the `ssid` and `wpa_passphrase` fields.

`ssid` corresponds to the access point name \(default: LeoRover-XXYYY\) and  `wpa_passphrase` is the network password \(default: password\).

Type `Ctrl+O` and `Enter` to save the modified configuration and `Ctrl+X` to exit the editor.

In order to apply the modifications, you need to restart the access point. To do so, just type:

```bash
sudo systemctl restart hostapd
```

{% hint style="success" %}
Done.
{% endhint %}

{% hint style="info" %}
In some cases, the changes may appear only after you restart the Rover 
{% endhint %}

