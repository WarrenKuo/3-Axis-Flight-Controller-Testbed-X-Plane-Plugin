# 3-Axis-Flight-Controller-Testbed-X-Plane-Plugin
An Xplane-9 Plugin to send Euler angles of the airplane out to the 3-axis gimbal through the serial port.  

The code for the Ardiono 3-axis gimbal is here: 

The video bellow shows how the plugin works:
https://www.youtube.com/watch?v=MCeUNxH9Y80

# Requirements
1. Windows
2. Visual Studio 2017
3. X-plane SDK 1.0.2 from https://developer.x-plane.com/sdk/plugin-sdk-downloads/
4. X-plane 9 to run the plugin

# Install the Plugin
1. Copy the *.xpl plugin file in Release folder into <X-Plane 9 program directory>\Resources\plugins\.
2. Copy and put the following two SDK *.dll files into the same directory as in the first step: "XPLM.dll" and "XPWidgets.dll".
3. Run X-Plane, and the plugin will automatically start and run.

