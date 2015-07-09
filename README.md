# Autonomous Trash Collection Robot
## Glendale Community College Robotics Academy

### Setup

* Install Ubuntu 14.04
* Install ROS Indigo
* Create workspace & clone packages

	mkdir -p dump_ws/src/
	cd dump_ws/src/
	catkin_init_workspace
	git clone git@github.com:gcc-robotics/rolling_dump_software.git
	cd ../

### Build

* `cd` into root of workspace
* `catkin_make`

### Run

* `source/devel/setup.bash`
* Actual robot: `roslaunch launch robot.launch`
* Simulation; `roslaunch launch simulation.launch`
