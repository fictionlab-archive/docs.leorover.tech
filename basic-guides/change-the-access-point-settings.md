# Change the access point settings

## Prerequisites

{% page-ref page="connect-via-ssh.md" %}

## Access the hostapd configuration file

Modify the `hostapd.conf` configuration file by typing:

```bash
sudo nano /etc/hostapd/hostapd.conf
```

An editor interface should appear.

![](../.gitbook/assets/image%20%2859%29.png)

## Modify the settings

### Change the SSID and password

Modify the `ssid` and `wpa_passphrase` fields.

`ssid` corresponds to the wireless network name \(default: `LeoRover-XXXX`\) and  `wpa_passphrase` is the network password \(default: `password`\).

### Change the AP channel

{% hint style="info" %}
This may be useful if you notice the network interference in crowded areas or you want to use multiple Leo Rovers at once.
{% endhint %}

Modify the `channel` field.

`channel`corresponds to the Rover 2.4 GHz Wifi access point broadcast channel. You can choose from 1 to 11.

### Set the country code

Modify the `country_code` field.

Set to indicate country in which the device is operating \(regulatory domain\). The country codes follow the [ISO 3166-1 standard](https://en.wikipedia.org/wiki/ISO_3166-1#Current_codes).

{% hint style="warning" %}
This can limit the number of available channels to use.
{% endhint %}

### Save the changes

Type `Ctrl+O` and `Enter` to save the modified configuration and `Ctrl+X` to exit the editor.

## Restart the access point

In order to apply the modifications, you need to restart the access point daemon. To do so, just type:

```bash
sudo systemctl restart hostapd
```

