### Rename to README.md when placed in package (Delete this sentence)

# package_name

## Overview Description

Paragraph on how this package fits into the overall architecture. What its nodes do and what is their general (common) purpose. Why do nodes in this package belong in this package?

## Goal and User Experience

What is the goal of the code and why it was done the way it was. How does this add to the user experience. What should the user experience with this code be like (some packages will have a lot more on this than others), provide examples and walk through the features. This will help with tech report.

## Build Instructions

* See the main README for simulation setup instructions

## Nodes

### node_name

* Node Information
 * What exactly does this Node do
 * Where does it run
 * What are its dependencies
 * Any other extra information that should be known about this node (dynamic reconfigures, params, etc.)

example ---------------------------------------------------------------------------------------------

file: hw_thruster_controller_interface.py

Node name:
* hw_thruster_controller_interface

Topics:

* `rov/cmd_horizontal_vdrive`:
  Subscribes `vector_drive/thrusterPercents` gives the thruster setting from -1000 to 1000 for thrusters T1,2,3,4.
  * `rov/cmd_vertical_vdrive`:
  Subscribes `vector_drive/thrusterPercents`gives the thruster setting from -1000 to 1000 for thrusters T5,6,7,8.
* `topic_name`:
  Publishes `message_type` info.

Services:
* `service_name`: info

Parameters/Reconfigs:
*  `parameter_name`: info


### other_node_name (if applicable)

* Node Inforation
 * What exactly deos this Node do
 * Where does it run
 * What are its dependencies
 * Any other extra information that should be known about this node (dynamic reconfigures, params, etc.)


## Launch Information

Any details about launch files and what they do goes here.
Any remapping information goes here.

## Troubleshooting

* If updating the mesh from solid works make sure that you select the option "Do not translate stl output data to positive space"
* Note: The default axis in solid works are flipped -> In order to fix it add a new coordinate system from the `add reference geometry` tool and flip the y and z axis

## Contributors

* Current maintainer: Michael Equi

* Contributors:
  * Michael Equi - Initial simulation setup and programming

## Helpful Resources

* https://github.com/uuvsimulator/uuv_simulator/wiki/Explaining-the-configuration-of-the-ROV-model-description
* https://uuvsimulator.github.io/
* http://www.fossen.biz/wiley/
