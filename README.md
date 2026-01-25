# Arduino-Velocity-Speed-Gun

This project is based on the work documented in [Kevin Darrah Wiki](https://www.kevindarrah.com/wiki/index.php?title=Arduino_Radar_Gun), but with my own enhancements.



### Useful notes:
K-Band radar module (not Ka, just K)
Likely beam width 12-15 degrees
Advertised specs for vehicle detection 10-200mph up to 1500ft range
Teardown: https://www.allaboutcircuits.com/news/teardown-tuesday-radar-gun/



### Rough parts list:
* Bushnell 101911 Velocity Speed Gun
* Arduino Micro (or other small ATMEGA32U4 board).  NOTE - many cheap clones don't break out all 12 analog inputs!
* USB surface mount port (if USB-C, needs resistors for PD compatibility)
* USB cable to cut and splice between Arduino and other components
* 5V SPDT relay
* 1x diode (any switching or rectifier diode)
* 5V to 3.3V regulator board
* ~2-3V to 5V boost regulator board
* 2x 10 ohm 1/2 watt resistors
* 2x 1K ohm 1/4 watt resistors
* 1x NPN transistor
* 2x 470uF capacitors (minimum 6v rated, recommend 10v rated or greater)
* 1x 100uF capacitor (minimum 6v rated, recommend 10v rated or greater)



### Wiring & Modifications:

Here's the wiring diagram I came up with.  This is more complicated because I wanted to retain full handheld battery-powered operation in addition to the modifications for computer control.  Due to how the LCD pins are tapped off, it is neccesary for the Arduino to boot up for the LCD to function at all, even when its not connected to a computer.  The relay ensures the batteries can't be accidently charged when plugged into USB and retains the full functionality of the low-battery indicator on battery power, but adds more complexity.  The resistors in line help slightly reduce the power consumption of the relay without affecting its operation (helps if it will be powered from a USB power bank)

![Schematic](Diagrams/Modification%20Schematic/Schematic.JPG)


I recommend starting with the case modification - its simple and doesn't affect operation but WILL damage the board if you forget this step and try to reassemble after soldering the power-signal wires in place.  Its reparable if you forget, but very annoying to fix the thin PCB traces after they are ripped off.

![Schematic](Diagrams/Modification%20Schematic/Case%20Modification.JPG)


The trigger board is the main location to find the positive battery connection in addition to the easiest place to tap onto the V+ supply to the main board and the trigger.  A small modification is required to cut the PCB pads between the V+ supply pin header and battery connection pads.

![Schematic](Diagrams/Modification%20Schematic/Trigger%20Board%20Modifications.JPG)


NOTE: Negative is not shown, but easiest is to run a wire down to the negative battery contact alongside the original negative battery wire.


Main board is where most of the wiring/modifications happen.  There are several LCD pins as well as two spots to connect to the power on/off pins.  The power-off pin will require carefully sanding/scratching the sodler mask down to access the PCB trace without damaging the PCB trace.
Main board view angle 1 of 2

![Schematic](Diagrams/Modification%20Schematic/Main%20Board%20Modification%20Wiring%20(angle%201).JPG)

Main board view angle 2 of 2

![Schematic](Diagrams/Modification%20Schematic/Main%20Board%20Modification%20Wiring%20(angle%202).JPG)



### Final assembly:

Additional photos of final assembly are located in the [Photos](Photos) directory.  I wrapped the relay and arduino in large heat-shrink to insulate them and then used aluminum cooking foil to provide RF shielding which helped reduce "ghost readings" but may not be strictly required.  If you are careful there's a fairly large amount of empty free space around the base of the horn that fits inside the case of the gun easily.

I also discovered we have no C-sized batteries in the house but a piece of 1/2 inch PVC water pipe makes a perfect adapter to allow use of AA batteries in the original battery compartment and the original spring is sufficiently long to make a good connection.  Rechargable AAs work fine too.

NOTE - on battery power after the modifications, when powered off the gun will draw about 5 milliamps due to the additional voltage converters and arduino in sleep mode.  That's not a big deal but will drian batteries over time even with the gun powered down so I recommend removing the batteries if it will not be used for multiple days.



### Software:

The [firmware for the arduino](radar_gun_matt.ino) bridges the radar gun controls to serial input/output for easy interfacing by a human or scripts/programs.  It requires a serial baud rate of 115200 but once configured you can send the "HELP" command to the serial console for a list of all available configuration options and commands.  It is intended to be very flexible either running configurable automatic loops and posting speed readings to the serial terminal or disabling the automatic operation and sending manual commands by serial to "push buttons" on the radar gun and "look at the display state" of the radar gun so you could interface a custom application of your own creation.

I have also created a couple very simple bash scripts that let me log speeds to tab-separated format (which MS Excel can open) and do very simple analysis summary of the speed readings file in a terminal, as well as included some simple example output files.  These are located in the [scripts](scripts) directory.

I chose to use a Raspberry Pi Zero 2 W to run the bash scripts since its compact and low power so I can run the entire device with logging and remote WiFi access off a portable USB battery pack.  It could also be modified to do nifty things like control a camera to build a radar speed camera or possibly run LPR software.
