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

# Modify the project
1. The serial port number is defined in "XPlane_GIMcom.h". The default value is COM10.
2. Main functions of the plugin are defined in "XPlaneAsSimEnvPlugin.cpp".
3. "XPlane_GIMcom.cpp" controlls the communications on the serial port.


# Wiring Diagram 

![image](https://user-images.githubusercontent.com/7514562/185743027-6c2772d2-53ea-4b12-a257-690d9687b5ee.png)
![image](https://user-images.githubusercontent.com/7514562/185743062-762ad5e6-3a0a-48f5-bc7e-353f3265c7d2.png)


