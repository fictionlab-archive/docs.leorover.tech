---
description: >-
  It's a quick guide on how to access clean video stream from the Rover - with
  no UI overlay.
---

# Access the video stream

With the steps provided you'll be able to access a clean video stream preview in the Rover network. The preview can be used to display the stream on a different screen or to be caught for further development. It's up to you.

## Access the stream via your browser

### 1. Connect to the Rover's access point

### 2. Enter the dedicated port

Type in your browser:

```text
http://10.0.0.1:8080/stream?topic=/camera/image_raw&type=ros_compressed
```

{% hint style="success" %}
Voila!
{% endhint %}

