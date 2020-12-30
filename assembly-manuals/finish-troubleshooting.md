# Extra: Troubleshooting

Here you can find the most common problems happening during and after the assembly. You'll be able to tackle them with the solutions we provide - hopefully. If not, feel free to write [contact@turtlerover.com](mailto:contact@turtlerover.com)



## Issue table

### The Rover doesn't drive via UI joystick. and voltage meter shows 0V

There's no firmware on Core2-ROS controller.

[https://docs.leorover.tech/basic-guides/firmware-update\#firmware-flashing](https://docs.leorover.tech/basic-guides/firmware-update#firmware-flashing)

### 

### Voltage meter shows 0V

There's no firmware on Core2-ROS controller.

[https://docs.leorover.tech/basic-guides/firmware-update\#firmware-flashing](https://docs.leorover.tech/basic-guides/firmware-update#firmware-flashing)



### 'Can't find Core2-ROS board' error during flashing

Make sure Core2-ROS microUSB is connected to RaspberryPi USB during flashing.

[https://docs.leorover.tech/basic-guides/firmware-update\#connect-to-micro-usb-hserial-port](https://docs.leorover.tech/basic-guides/firmware-update#connect-to-micro-usb-hserial-port)



### Can't flash the firmware

Please flash the bootlader first.

[https://docs.leorover.tech/basic-guides/firmware-update\#core-2-ros-straight-out-of-the-box-bootloader-not-flashed](https://docs.leorover.tech/basic-guides/firmware-update#core-2-ros-straight-out-of-the-box-bootloader-not-flashed)



### Wheel doesn't work at all \(only one\)

Check if the wheel gold-pin connector is connected right way. OutA \(on Core2-ROS\) needs to be connected to the motor red cable.

[https://docs.leorover.tech/assembly-manuals/main-electronics-box-assembly\#9-core-2-ros-assembly-and-cabling](https://docs.leorover.tech/assembly-manuals/main-electronics-box-assembly#9-core-2-ros-assembly-and-cabling)



### Wheel jumps back and forth instead of driving

First: Make sure you have proper firmware settings \(Buehler motors are set by default, if you need older Pololu motors check this topic: [https://forum.fictionlab.pl/t/firmware-1-0-settings/162](https://forum.fictionlab.pl/t/firmware-1-0-settings/162)\).

Second: There may be a problem with encoder signal. Please check white and yellow cables all the way from the wheel to Core2-ROS controller. Check the wheel after each of the steps listed. 

1. Reconnect the wheel gold-pin connector on the board.
2. Move the connector in place of different wheel and see if still happens.
3. Check if any of the cable isn't routed above the electronics power segment \(near DC connector\) - there may be interference, re-route the cables if so.
4. Check the soldered connection \(between wheel cable and MEB cable sections\).
5. Open the wheel cap and check if the cables are connected to the motor encoder.Open the wheel cap and check if the cables are connected to the motor encoder.

[https://docs.leorover.tech/assembly-manuals/main-electronics-box-assembly\#9-core-2-ros-assembly-and-cabling](https://docs.leorover.tech/assembly-manuals/main-electronics-box-assembly#9-core-2-ros-assembly-and-cabling)



### Wheel squeaks during work

Lube torque plate 'fingers' of the squeaking wheel.



### Wheel hub keeps coming off

Use Loctite \(or any thread glue\) on a torque screw in the wheel hub.

[https://docs.leorover.tech/assembly-manuals/1.-wheel-assembly\#step-6-wheel-hub](https://docs.leorover.tech/assembly-manuals/1.-wheel-assembly#step-6-wheel-hub)



### Wifi signal is weak

Check if the antenna cable is properly connected to the modem inside on Main Electronics Box \(MEB\). Check if the cabe is secured by zip-ties.

[https://docs.leorover.tech/assembly-manuals/main-electronics-box-assembly\#10-wifi-modem-assembly](https://docs.leorover.tech/assembly-manuals/main-electronics-box-assembly#10-wifi-modem-assembly) 



