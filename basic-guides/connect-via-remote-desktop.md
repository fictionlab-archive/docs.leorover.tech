# Connect via remote desktop

If you are using the `full` version of LeoOS, you should have a desktop environment installed, as well as an RDP \(Microsoft Remote Desktop Protocol\) server. This allows you to remotely control a desktop session on your Rover from your computer.

## On Windows

Click **Start**, type `remote` and choose **Remote Desktop Connection** app. 

![](../.gitbook/assets/image%20%2867%29.png)

Type `10.0.0.1` in the **Computer** field and click **Connect**.

![](../.gitbook/assets/image%20%2868%29.png)

You should see a similar warning. Click **Yes** to proceed.

![](../.gitbook/assets/image%20%2864%29.png)

You should see the login screen. Choose `Xorg` session, type `pi` for the username, `raspberry` for the password and click **OK**.

## On Linux

Install and open [Remmina Remote Desktop Client](https://remmina.org/how-to-install-remmina/).

![](../.gitbook/assets/image%20%2857%29.png)

Click on the `+` icon and fill the following fields:

* Server: `10.0.0.1` 
* User name: `pi` 
* User password: `raspberry` 
* Color depth: `True color (32 bpp)` 
* \(Optional\) Name - if you want to save the settings
* \(Optional\) Resolution - If you want to use a custom resolution

![](../.gitbook/assets/image%20%2870%29.png)

Click on **Save and Connect**.

![](../.gitbook/assets/image%20%2862%29.png)

If you get a prompt about the certificate, click **OK**.

## On macOS

Install and open [Microsoft Remote Desktop](https://apps.apple.com/pl/app/microsoft-remote-desktop/id1295203466?l=pl&mt=12).

![](../.gitbook/assets/zrzut-ekranu-2020-07-7-o-16.24.51.png)

Click on **Add PC**, type `10.0.0.1` in the **PC name** field, `leo` in the **Friendly name** field and click on **Add**.

![](../.gitbook/assets/zrzut-ekranu-2020-07-7-o-16.26.11.png)

Double click on the `leo` computer to connect.

![](../.gitbook/assets/zrzut-ekranu-2020-07-7-o-16.26.44.png)

When asked, fill in the Username and Password.

![](../.gitbook/assets/zrzut-ekranu-2020-07-7-o-16.27.15.png)

If you get a prompt about the certificate, click **Continue**.

