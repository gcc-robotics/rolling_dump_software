# Wall-Z Autonomous Trash Collection Robot
#### By Glendale Community College Robotics Academy

### Setup

* Install Ubuntu 14.04
* Install ROS Indigo
* Create workspace & clone packages

		mkdir -p wall_z_ws/src/
		cd wall_z_ws/src/
		catkin_init_workspace
		git clone git@github.com:gcc-robotics/wall_z_software.git
		cd ../

### Build

* `cd` into root of workspace
* `catkin_make`

### Run

* `source/devel/setup.bash`
* Actual robot: `roslaunch launch robot.launch`
* Simulation: `roslaunch launch simulation.launch`
