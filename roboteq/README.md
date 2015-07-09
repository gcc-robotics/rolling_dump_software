#ros-roboteq-hdc2450

A package that provides an ROS interface for the Roboteq HDC 2450 Motor Controller.

### roboteq node

Communicates with the motor controller over USB. Uses the the /dev/ttyAMC0 device by default.
You will need to either run it as root (bad idea) or be in the dialout group.

Subscribes to a ROS topic for motor controller commands.

### keyboard node

Listens for key presses on the terminal and publishes messages on the topic the _roboteq_ node listens on to control the motor controller.

Use the arrow keys to control the motors.