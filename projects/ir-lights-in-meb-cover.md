---
description: >-
  Short tutorial about modifying MEB cover. May also be used as short 3D
  modeling guide. Toolbar of Autodesk Inventor is hidden on purpose as we wanted
  to make this tutorial as universal as possible.
---

# IR lights in MEB cover

## Introduction

![](../.gitbook/assets/assembly3.jpg)

In this tutorial we will show you how to edit Main Electronic Bay cover \(by example of IR lights\) to suit your needs. We wanted to show that all of our 3D printed parts are easily modifiable. This particular part \(MEB cover\) was designed especially for you - you can change it or use 4 mount points to attach additional PCB's. So let's get started with IR lights!

## Step-by-step modification

Always in this kind of modification the most important thing is to have 3D models of components that we want to connect. The easiest way to assemble additional component to existing component is to join two 3D models \(even if everything is interfering\) and then remove all of the interferences. In our case we want to assemble two IR lights to MEB cover - so lets find good place for them and assemble them in 3D software. 

{% hint style="warning" %}
IR diodes may vary! Check if dimensions from 3D model are matching real sample!
{% endhint %}



![](../.gitbook/assets/1.JPG)

**Step 1.** Measure diameter of IR diode in order to assemble it to MEB cover.

![](../.gitbook/assets/2.JPG)

**Step 2**. Prepare mounting points for IR lights. Dimensions are not important now - we can modify them later.

![](../.gitbook/assets/3.JPG)

**Step 3.** Cut the holes and assemble IR lights in them. You can offset them if you want.

![](../.gitbook/assets/4%20%281%29.JPG)

**Step 4.** As we can see there is interference in right IR light - to get rid of it we can simply move mounting holes.

![](../.gitbook/assets/5.JPG)

**Step 5.** Decreasing dimension from 40 to 35mm.

![](../.gitbook/assets/6.JPG)

**Step 6**. Checking position and diameter of light sensor. Position can be easily checked by X and Y distance.

![](../.gitbook/assets/7.JPG)

**Step 7.** Cutting holes for light sensors.

![](../.gitbook/assets/8.JPG)

**Step 8.** To check different interferences we can use section view. At the bottom of IR light there is small interference that we should get rid of.

![](../.gitbook/assets/9.JPG)

**Step 9.** To avoid interference we should cut out small rectangular shape. We need to check the position and size of this rectangle. Width of it should be the same as width of IR diode \(30mm\).

![](../.gitbook/assets/11.JPG)

![](../.gitbook/assets/12.JPG)

**Step 10.** In order to create rectangular cutout we need to decide from which plane we will be making it. I decided to start from the top of the cover because it will be easy to measure everything. Thickness of IR light will be height of our rectangle and distance from IR light will be position of it. 

![](../.gitbook/assets/13.JPG)

**Step 11.** Last dimension needed for this cutout is depth of extrusion that we will be making. In this case it will be distance from top of the cover \(plane of our extrusion\) and bottom of IR light.

![](../.gitbook/assets/image%20%2832%29.png)

![](../.gitbook/assets/image%20%2810%29.png)

**Step 12.** It is a good practice to make cutouts slightly bigger - I decided that it will be 0.3mm bigger. That's why dimension of rectangle is 30x1,5mm. X position of cutout is 6.15 mm \(6.3 mm was measured but I substracted half of 0.3 mm\). Y position is 9.75mm from center of hole for IR diode because it is radius of this hole. As we know from **step 11** depth of cutout should be 21.5mm + 0.3 mm = 21.8mm.

![](../.gitbook/assets/14.JPG)

**Step 13.** After all of this operations we can check results of our work by using again section view. As we can see interference was removed.

![](../.gitbook/assets/15.JPG)

![](../.gitbook/assets/16.JPG)

**Step 14.** Now it's time to create mounting points for IR lights. I guess that after previous steps your an expert in measuring dimensions of IR light so I will just skip it. Using plane that was created by making rectangular cutout we can make a sketch of our mounting points. I suggest starting with mounting points. Draw them on the sketch and then create rectangle around them. 

![](../.gitbook/assets/17.JPG)

**Step 15.** This time for extrusion we will use very useful function which is called "to next". If we don't know the distance but just want to extrude our sketch to the next solid it is perfect solution. It is one of the options in distance row \(in Autodesk Inventor but I guess that almost all of the 3D modelling softwares have it\).

![](../.gitbook/assets/18.JPG)

**Step 16.** In order to make our work faster I created mounting points only for one IR light. Second one is exactly the same so we can use rectangular pattern to copy it. We know from **step 5** that distance between two IR lights is 2x35 = 70mm. As a direction we can use any edge that is parallel to the top of the MEB cover.

![](../.gitbook/assets/19.JPG)

![](../.gitbook/assets/20.JPG)

**Step 17.** To get rid of raw look of this mounting points we can add some fillets.

![](../.gitbook/assets/21.JPG)

![](../.gitbook/assets/22.JPG)

**Step 18.** To protect IR lights we can create small dash that will serve as protection for the lens. Dimensions are totally random so you can modify them as you want ;\).

![](../.gitbook/assets/23.JPG)

**Step 19.** After adding fillets it looks quite nice!

## 3D printing

As for this part of tutorial you should visit our manual about creating Camera mast in which 3D printing issues are described. It is quite easy and after saving your 3D model as .STL file you should be able to do it. 

{% page-ref page="camera-mast.md" %}

![](../.gitbook/assets/image%20%2831%29.png)

## Assembling 

![](../.gitbook/assets/20200201_133452_compress18.jpg)

**Step 1.** Check if everything fits.

![](../.gitbook/assets/20200201_133509_compress21.jpg)

**Step 2.** Using small screws \(I used 2,2x6\) assemble IR diodes to the MEB cover.

![](../.gitbook/assets/20200201_151249_compress24.jpg)

**Step 3.** Now we need to take care about powering our IR diodes. Upper hole is VCC and lower is GND. I created small wire harness with male goldpin connector. You can use small ring connectors or solder cables directly to the pads on the diodes as I did.

![](../.gitbook/assets/20200201_144504_compress7.jpg)

**Step 4.** Unfortunately Husarion Core2ROS board doesn't have 3.3VDC on its pinout. The simpliest place from where we can take 3.3V is camera board itself. Disassemble WIFI module, Husarion Core2ROS board and camera mount in order to have more place. Pad used for mounting the camera is also power supply - the one closer to the conector is VCC and further one is GND. Here also you can use ring connector or solder cable directly to the board as I did. On the second end of cables place female goldpin connector.

![](../.gitbook/assets/20200201_144750_compress36.jpg)



**Step 5.** Fasten 2 screws holding camera mount

![](../.gitbook/assets/przechwytywanie.JPG)

**Step 6.** Place Husarion Core2ROS and fasten 4 M2,5 distances to the RaspberryPi.

![](../.gitbook/assets/przechwytywanie1.JPG)

**Step 7.** Place WIFI module and **f**asten 4 M2,5 bolts to the distances.

![](../.gitbook/assets/20200201_151322_compress37.jpg)

**Step 8.** Plug the connector.

{% hint style="warning" %}
Double check the connection. Use multimeter to check polarity - GND is for example on the female USB connector of Husarion CORE2ROS board.
{% endhint %}

![](../.gitbook/assets/20200201_151542_compress48.jpg)

**Step 9.** Assemble MEB cover to the MEB

![](../.gitbook/assets/20200201_151533_compress86.jpg)

**Step 10.** And that's it! You have Leo rover that can see even in full darkness! ENJOY!

