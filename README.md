# ROSbasic

The goal of this project is to develop preseason software technologies based on ROS. This project will explorer development paths and verify new technologies before more permanent hardware development. Key areas of testing are cameras, vector drive, controllers, and PID algortihms. Additionally new workflows/integration tactics are to be tested and documented (Travis CI, Docker, node documentation, doxygen). This project will expire at the beginning of the 2018-2019 robotics season.   

Release    
[![Build Status](https://travis-ci.com/JHSRobo/ROSbasic.svg?branch=release)](https://travis-ci.com/JHSRobo/ROSbasic)

Master     
[![Build Status](https://travis-ci.com/JHSRobo/ROSbasic.svg?branch=master)](https://travis-ci.com/JHSRobo/ROSbasic)

Development     
[![Build Status](https://travis-ci.com/JHSRobo/ROSbasic.svg?branch=development)](https://travis-ci.com/JHSRobo/ROSbasic)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on the main ROV system (Master).

FOLLOW:
*  https://docs.google.com/document/d/1tYhxP1HbuTF7Nzl1WnGgJKkUe-KNsBEhQZDjsyUwQTU/edit

*Always run IDE's from terminal if on Ubuntu (just type the name of the IDE in terminal and click enter ex. clion)*

### Launching/Running

Follow Test Bench Setup Steps if Running on Test Bench
* https://docs.google.com/document/d/1srYgNUE4k3DVHkUv1TwUJawfUWw6kGkveDhAvulWMZ0/edit#heading=h.wyzbdb7zgifi

Locally:
* https://docs.google.com/document/d/16LQRhCJBEe_hL-SV67Vvk3oH7I2nqO0X7NjB1t-9Mtg/edit#heading=h.v4rl9rent2ka

### Prerequisites

What things you should to install to develop and run software and how to install them

Code Blocks IDE on rpi ubuntu mate
* `sudo apt-get install codeblocks`

On the RPI turn on the CSI, SPI, I2C, and UART interfaces using `sudo raspi-config`

Setup the I2C interface on Ubutnu Mate
* `sudo apt-get install python-smbus`
* `cd /boot/config.txt`
* uncomment `dtparam=i2c_arm=off` and change to `dtparam=i2c_arm=on`
* uncomment `dtparam=i2c_arm_baudrate=100000` and change to `dtparam=i2c_arm_baudrate=400000`
* restart pi

Setup the ros_lib file for arduino serial
* See sketchbook README.md documentation

RPI Camera node setup
* See raspicam_node README.md documentation


ROS PIDs library
* `sudo apt install ros-kinetic-pid`

Turn RPI Safemode OFF
* Type cd /boot on the bottomside RPI
* Type sudo nano config.txt
* Type CTRL + W and then search for avoid_safe and hit enter
* Go to the bottom of the section and uncomment avoid_safemode=1
* Type CTRL + X and then Y to exit


### Network Setup

What things you need to do so that the ROS network operates properly

On ubuntu 16.04 go to Network Connections app and add a new ethernet connection (name the connection `ROVEthernetConnection`)
* On the topside computer have a static (manual) IP of `192.168.1.100`, netmask `24`, Gateway `92.168.1.1`, DNS server `27.0.1.1, 8.8.8.8, 192.168.1.1`
* On the bottomside computer have a static (manual) IP of `192.168.1.111`, netmask `24`, Gateway `192.168.1.1`, DNS server `127.0.1.1, 8.8.8.8, 192.168.1.1`
* Run the setupROSNetwork.sh script in the scripts folder

Once the network connection has been verified (on bottomside `ping master` / on topside `ping bottomside`)
* Run `sshSetup.sh` in the scripts folder
* Do not add any paraphrases
* on bottomside `ssh master` / on topside `ssh bottomside`
* Make sure both work without entering a password

#### Network Setup DEBUG
* IF you receive `/usr/bin/ssh-copy-id: ERROR: ssh: connect to host bottomside port 22: Connection refused` go to the opposite machine from the one you received it on and run the following:
    * `sudo rm /etc/ssh/sshd_config`
    * `sudo apt-get purge openssh-server`
    * `sudo apt-get install openssh-server`
    * `./sshSetup.sh`

* If topside and bottomside aren't communicating do the following:
    * ping 192.168.1.100 from bottomside, and 192.168.1.111 from topside (If they don't communicate, make sure all the hardware is setup correctly. Also check your virtual machine network settings, and make sure it is bridging to the network (directly connected to physical network and replicating physical connection state)

Other useful links for common problems:
* https://superuser.com/questions/421004/how-to-fix-warning-about-ecdsa-host-key
* https://askubuntu.com/questions/762541/ubuntu-16-04-ssh-sign-and-send-pubkey-signing-failed-agent-refused-operation
* https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/

### Installing OS

A step by step series of examples that tell you how to get a development env running

On your Raspberry Pi 3B make sure you are running ubuntu mate 16.04 (image here https://drive.google.com/open?id=1497jupJ2dBQqy_o_x5JBPTjY3lto7-rI)
* cat /etc/os-release

### Setup Topside Peripheral Communication
* run `sudo usermod -a -G dialout $USER` to give proper permissions to USB interface programs (similar to `sudo chmod a+rw /dev/tty...`)

### Simulation Setup

* See https://uuvsimulator.github.io/installation.html#creating-and-configuring-a-workspace
* Check to see if gazebo version 7 is installed `gazebo --version`
* Run `sudo apt-get update` and `sudo apt-get upgrade`
* In the ros_workspace run `git submodule init` and `git submodule update`
* In ros_workspace run `rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y`
* Run `sudo apt-get install protobuf-compiler protobuf-c-compiler`
* Connect a powerfull computer to the ROS Master by running `export ROS_MASTER_URI=http://ip_to_master:11311` and `export ROS_IP=ip_of_computer`
   * Make sure the master and simulation computers can ping each other and if network problems persist make sure the computers can ssh into each other
* Run `catkin_make`
   * If you receive this error `make[2]: *** No rule to make target '/home/michael/catkin_ws/src/uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_plugins/PROTOBUF_PROTOC_EXECUTABLE-NOTFOUND', needed by 'uuv_simulator/uuv_gazebo_plugins uuv_gazebo_plugins/Double.pb.cc'.  Stop. CMakeFiles/Makefile2:4699: recipe for target 'uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_plugins/CMakeFiles/uuv_gazebo_plugins_msgs.dir/all' failed` then remove the /build /devel and /install folders with `rm -R` and retry steps 1 - 8
* `catkin_make install` <- May not be needed (check on next install)
* `source devel/setup.bash`
* `roslaunch rov_description full_systems_launch.launch` or `roslaunch rov_description partial_systems_launch.launch` and `roslaunch simulate_rov.launch` on another machine
* `rosrun rov_description simulation_interface.py`

### Intel Realsense Setup instructions
## Ubuntu

* Follow the instructions here https://github.com/intel-ros/realsense
* Test using `roslaunch realsense2_camera rs_camera.launch`

Note: The resolution of the realsense camera should always run at 1280x720 for the D415 and 848x480 for the D435

## RP 3B+

* Follow the instructions here https://github.com/intel-ros/realsense
* Test using `roslaunch realsense2_camera rs_camera.launch`


##UPDATES NEEDED BELOW THIS POINT
--------------------------------

## Running the tests

Explain how to run the automated tests for this system (travis CI)

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why (coveralls)

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system (docker)

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://github.com/Michael-Equi/ROV_Test_Bench/blob/development/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## Authors

* **Michael Equi** - *Initial work*

See also the list of [contributors](https://github.com/Michael-Equi/ROV_Test_Bench/graphs/contributors) who participated in this project.

## Acknowledgments

* README Template source https://gist.github.com/PurpleBooth/109311bb0361f32d87a2
* Inspiration
* etc
