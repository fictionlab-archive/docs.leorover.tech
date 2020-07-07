# Upload files to your Rover

## On Windows

Download and install [WinSCP](https://winscp.net/eng/download.php)

Connect to the Rover's WiFi.

Just after you open the software it will ask you for the Rover login details, as always:

IP: `10.0.0.1` \| Login: `pi` \| Password: `raspberry`

and click `login`.

Now in the right window you'll see all the files located in your Rover.

You can upload files by dragging them from your local computer and dropping them inside Leo Rover's filesystem.

## On Linux or Mac

You can use `scp` tool for this. Just type:

```text
scp [local_path] pi@10.0.0.1:[remote_path]
```

If you want to copy a whole directory you can add `-r` option

```text
scp -r [local_path] pi@10.0.0.1:[remote_path]
```

{% hint style="info" %}
if `[remote_path]` is a relative path it will be resolved in `/home/pi` namespace \(e.g. `pi@10.0.0.1:leo` will be resolved to `/home/pi/leo`\). 

To show more options, type `man scp` 
{% endhint %}

Alternatively you can use `sshfs` to mount remote file system into your local directory

```text
mkdir -p [local_path]
sshfs pi@10.0.0.1:[remote_path] [local_path]
```

Now you can just put files inside \[local\_path\] and it will be automatically synchronized with a remote directory.

When you're done just unmount the directory by typing

```text
sudo umount [local_path]
```

## File Permissions

By default, when logging as `pi` user, you have write permission only on files inside `/home/pi` directory. This should be sufficient in most cases but if you really need to upload files outside of the home directory you have a couple of options.

#### Recommended: take ownership of the directory

First, login to the Rover via ssh:

{% page-ref page="connect-via-ssh.md" %}

Create a directory if it doesn't exist

```text
mkdir -p [upload_directory]
```

Now, assign the ownership of that directory

```text
sudo chown -R pi:pi [upload_directory]
```

This will change ownership of \[upload\_directory\] and all files inside it to `pi` user

#### Not recommended: upload files by logging into root account

Root logging is disabled by default. If you want to turn this on you can follow [these](https://raspberrypi.stackexchange.com/a/48061) instructions. 

You can then get access to all files by logging to `root` instead of `pi`

{% hint style="warning" %}
Modifying the root filesystem can be a destructive operation so if you don't fully understand what you're doing, make sure to backup your files first.
{% endhint %}

