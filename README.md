# PS3xPAD Plugin by OsirisX
A PS3 plugin by OsirisX, that allows using many unsupported Controllers (XBOX,PS4 and others)

Link: https://www.psx-place.com/resources/ps3xpad.22/

I intended on adding a couple more features into this release but would have added more delay.
Also I will be away from my main computer for the holidays and wanted to push a release before I left.

The changes in this release include:
- fixed the remapping issues where the controller seemed buggy when it was enabled
- added custom and native driver input mode
- added VSH/In-Game menu
- added wireless DS3/DS4 support (needs usb BT adapter)
- added game compatibility modes (fixes GTAV & RDR)
- added multiple remap settings
- added user configurable settings file (xpad_settings.txt)
- added user configurable controller delay which sets response time of controller
- added auto game detection and attach to game proc
- added disconnect/reset all ports option
- added screenshot option

The main focus of this release are the additions of two different modes (custom and native), the VSH/In-Game menu,
and wireless support for DualShock 4 controllers. I will first explain the difference between custom and native modes.
Custom mode is simply my implementation of the usb controller pad driver. This mode is needed for Xbox controllers
but is not needed for official PS controllers. 

Custom mode still supports official PS controllers but will need a separate usb Bluetooth adapter to connect them wirelessly.
Native mode allows connecting official controllers using the internal BT module and give it the features of the XPAD plugin such as remapping.
I only recommend using native mode if you want to connect your controller wirelessly and do not have a BT adapter.

The requirements for using native mode is you need CFW with DEX kernel and your game Eboots must be converted to debug.
These are also the requirements for the In-Game menu. The In-Game menu only works for a handful of games anyway (games with low memory).
I recommend configuring the XPAD plugin through the VSH menu before starting a game and using the button combinations during the game.

I will try making a more comprehensive guide to connecting a DualShock 4 controller later but here's a rough sketch.
This is for connecting the controller in custom mode with a BT adapter. Also note the bluetooth driver is based on the USB host shield for Arduino.
It should work for most generic adapters although I have only tested with one adapter.

First we need to set the internal bluetooth address of the controller. You only need to do this the first
time you connect the controller to the bluetooth adapter. There are two ways of doing this but I will show one way.

1. Connect your DualShock 4 through USB.
2. Start up your PS3 with the XPAD plugin loaded.
3. Press (START+SELECT+SQUARE) to open the VSH menu.
4. Go down and enter the bluetooth options menu.
5. Go down and select the "Set DS4 internal BT address" option.
6. You can now disconnect the DS4 controller and should connect wirelessly to the adapter.

Also please check the compatibility chart in the README to see what your system supports.
In general, a system that uses DEX kernel will have full support for the plugin.

-OsirisX
