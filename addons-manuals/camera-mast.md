# Camera mast

## Introduction

In this tutorial we will show you how to build and integrate another addon with Leo Rover - Camera Mast. 

It is inspired by camera mast built on top of NASA's rover Opportunity and is able to rotate camera through an angle of 360 degrees, more than 270 mm above the top of the Rover! 

Motors that are used for this application are Dynamixel AX-12A servo's which provide feedback, are quite easy to control with usage of ArbotiX and are in decent price. If you have access to a 3D printer, everything you need to build this addon is really easy to get and no special knowledge is needed. We will get through creating G-codes for 3D printer, assembling everything and at the end integrating it with Leo Rover.

## List of materials

* 3D printed parts
  * 01126v2 \(Base\)
  * 01127 \(Main tube\)
  * 01129 \(Second servo mount\)
  * 01130 \(Second servo cover\)
  * 01125 \(Rotation shaft\)
  * 01131 \(Third servo mount\)
  * 01132 \(Camera mount\)
  * 2x 01133 \(Bearing shaft\)
  * 01110 \(ArbotiX base\)
  * 01111 \(ArbotiX cover\)
* 3x Dynamixel AX-12A
* 3x Dynamixel cable
* ArbotiX-M Robocontroller
* FTDI USB-UART cable
* Leo Rover USB dongle
* 2x 6800Z ball bearing \(dimensions -&gt; 19x10x5 mm\)
* 52x40x7 mm ball bearing
* 8x M3 nut
* 4x M3 square nut
* 4x 2,2x6,5 screw
* 8x M3x10 hex bolt with button head \(actually any M3x10 will be okay\)
* 4x M3x6 hex bolt with button head \(actually any M3x6 will be okay\)
* 2x M3x10 hex bolt with head cap \(ISO 4762\)
* 26x M2x8 hex bolt with head cap \(ISO 4762\)
* 26x M2 nut

## 3D printing

You can get all of the needed files here:

{% embed url="https://drive.google.com/open?id=1-fVJVr4C96ZNTXPNRCI\_6HUhHGxSCwXg" %}

As at our company we are using Prusa 3D printers, we will show how to prepare 3D models for printing using their software - [PrusaSlicer](https://www.prusa3d.pl/prusaslicer/). This application will provide a special file for the printer \(G-code\) which tells the machine what are the settings and how it should move to create our model.

![PrusaSlicer layout](../.gitbook/assets/image%20%2829%29.png)

First thing we need to do is to import our files to the application. Click on the `Add` button \(box with a plus sign at the top of the screen\) and select the files you want to add.

After uploading the files, it should look like this:

![](../.gitbook/assets/image%20%283%29.png)

Now we need to spread the models in order to avoid interference. To automatize the process, press the `Arrange` button \(on the right from bin icon\).

![](../.gitbook/assets/image%20%2823%29.png)

The arrangement is still not perfect. Our goal is to minimize the number of walls that are "levitating" in the air to avoid support constructions that needs to be printed. Here are some hints for positioning components:

* Minimize support constructions - it brings a lot of advantages:
  * shorter printing time,
  * less post-processing,
  * smaller amount of used filament,
  * better quality of print.
* Try to use big flat surfaces as base to increase adhesive area.
* If some places in component are critical \(ex. bearing housing, round holes\) try to put them on the top of the print.

By using the buttons on the left, we can rotate and change position of every component. As you can see, we need 2 pcs of Bearing shaft \(01133\), so click on this part and press `+` button to add another instance of this model. When you are satisfied with the orientation of the models, press the `Arrange` button again for optimal arrangement of components at the 3D printer table. 

![](../.gitbook/assets/image%20%2824%29.png)

Now we can move on to the settings. You can go through all the detailed options by clicking on different tabs at the top, but if you don't have much experience in 3D printing, we recommend using system presets. 

The presets can be chosen from the panel on the right side and they consist of:

* **Print settings** They affect the layer height, printing speed and quality of the printed model. For our need, the fastest option - `0.30mm DRAFT` will be enough. If you need a better quality print and you don't care about the printing time, you can choose a more detailed preset.
* **Filament** In our case it is `Prusament PLA` and it is also our recommendation for you. For more sturdy models, you can use ABS but it is much harder in printing - it may peel of the table or the layers may not stick together and you will have to start the print from the beginning.
* **Printer** Select the printer you want to use. In our case it is `Original Prusa i3 MK3`.
* **Supports** For this option, we suggest `support on build plate only`. Printer will handle everything that is inside of the models and we will save some time and material. In case of some problems, you can use `support everywhere` option.
* **Infill** As the material we are using is not so strong, we are using 100% infill. In lower infill settings, the 3D printer will do grid structures instead of full infill.

If everything is set, click on `Slice now` button.

![](../.gitbook/assets/image%20%2828%29.png)

You can now see exactly how your print will look like. Use your mouse to rotate, move or zoom your preview and the slider on the right side to discover individual layers of the build.

When you'll finish admiring your work, you can click on the `Export G-code` button that's located on the bottom right corner. Save the file on SD card of your 3D printer, load SD card into your machine and that's it! You can now start printing. 

{% hint style="info" %}
Remember to clean 3D printer table before any print! We recommend IPA for this purpose. 
{% endhint %}

## Assembling 

![Flat layout of all elements](../.gitbook/assets/20191116_151318-min.jpg)

![Step 1: Press ball bearings into 01132 and 01127.](../.gitbook/assets/20191116_162619-min%20%281%29.jpg)

![Step 2: Assemble 01127 with 01129 using 4xM3x10 bolts and 4xM3 nuts.](../.gitbook/assets/20191116_162559-min%20%281%29.jpg)

![Step 3: Prepare AX-12A servo, 01130 and 10xM2 nuts](../.gitbook/assets/20191116_162547-min.jpg)

![Step 4: press M2 nuts into places where there will be bolt connection and assemble servo and 01130](../.gitbook/assets/20191116_162444-min%20%281%29.jpg)

![Step 5: Connect 2 cables to the servo. If you have short cables you should extend them. Especially the one which will be going to the upper servo.](../.gitbook/assets/20191116_162432-min%20%281%29.jpg)

![Step 6: Route the wire through rectangular hole in main tube](../.gitbook/assets/20191116_162413-min%20%281%29.jpg)

![Step 7: bolt together both parts using 6xM2 bolts ](../.gitbook/assets/20191116_162338-min%20%281%29.jpg)

![Step 8: Assemble servo to the main tube using 4xM2 bolts on one side and M3x10 bolt and bearing shaft on the other. Route cable through rectangular hole](../.gitbook/assets/20191116_162253-min%20%282%29.jpg)

![Step 9: Press big ball bearing into 01130.](../.gitbook/assets/20191116_162218-min%20%282%29.jpg)

![Step 10: Bolt rotation shaft to the servo using 4xM2 bolt. Make sure that shaft is parallel to the servo horn.](../.gitbook/assets/20191116_162208-min%20%281%29.jpg)

![Step 11: Bolt third servo to the 01131 using 8xM2 bolts and plug the cable.](../.gitbook/assets/20191116_162031-min.jpg)

![Step 12: Assemble 01131 to the rest of camera mast using 4xM3x10 bolts and 4xM3 nuts.](../.gitbook/assets/20191116_155343-min.jpg)

![Step 13: Assemble 01132 to the rest using 4xM2 bolts on one side and bearing shaft and M3x10 bolt on the other side - similar to the first joint.](../.gitbook/assets/20191116_155615-min.jpg)

## Integrating with Leo Rover

COMING SOON!


