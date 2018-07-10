# ROV Test Bench

One Paragraph of project description goes here

[![Build Status](https://travis-ci.com/Michael-Equi/ROV_Test_Bench.svg?branch=master)](https://travis-ci.com/Michael-Equi/ROV_Test_Bench)

[![Coverage Status](https://coveralls.io/repos/github/Michael-Equi/ROV_Test_Bench/badge.svg?branch=master)](https://coveralls.io/github/Michael-Equi/ROV_Test_Bench?branch=master)

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

FOLLOW:
*  https://docs.google.com/document/d/1C32ucQTIAsE2H7u9OERmmCfyc6WzUvhrB2U6_GgZuWg/edit?usp=sharing

Initial SETUP:
* `cd ~/Desktop`
* `git clone https://github.com/Michael-Equi/ROV_Test_Bench.git`
* `cd ROV_Test_Bench/scripts`
* `./GitSetup,sh`
* `./setup.sh`
    
*Always run IDE's from terminal if on Ubuntu (just type the name of the IDE in terminal and click enter ex. clion)
Only develop with clion, webstorm, pycharm, and arduino*

### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```
### Network Setup

What things you need to do so that the ROS network operates properly 

On ubuntu 16.04 go to Network Connections app and add a new ethernet connection (name the connection `ROVEthernetConnection`)
* On the topside computer have a static (manual) IP of `192.168.1.100`, netmask `24`, Gateway `92.168.1.1` DNS server `27.0.1.1, 8.8.8.8, 192.168.1.1`
* On the bottomside computer have a static (manual) IP of `192.168.1.111`, netmask `24`, Gateway `192.168.1.1, DNS server 127.0.1.1, 8.8.8.8, 192.168.1.1`
* Run the setupROSNetwork.sh script in the scripts folder

```
Give examples
```

### Installing

A step by step series of examples that tell you how to get a development env running

On your Raspberry Pi 3 B make sure you are running ubuntu mate 16.04 (image here https://drive.google.com/open?id=1497jupJ2dBQqy_o_x5JBPTjY3lto7-rI)
* cat /etc/os-release
* https://www.intorobotics.com/how-to-install-ros-kinetic-on-raspberry-pi-3-running-raspbian-stretch-lite/
* http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick (replace indigo with kinetic)

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

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

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
