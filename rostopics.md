# ROS Topic List and Information

This document provides information on each current ROS Topic, with the purpose and message type. More information can be found on each topic inside their respective package.

* `/rov/camera_select`: Current selected camera from the copilot page
  * std_msgs/Float64
* `/rov/sensitivity`: Sensitivity settings for ROV drive control
  * rov_control_interface/rov_sensitivity
* `/rov/inverstion`: Inversion status depending on camera
  * std_msgs/UInt8
* `/rov/thrusters_enabled`: Whether or not thrusters are enabled
  * std_msgs/Bool

* `/rov/cmd_vel`: Command Velocity vectors
  * geometry_msgs/Twist
* `rov/cmd_horizontal_vdrive`: Thruster Settings for Horizontals
  * vector_drive/thrusterPercents
* `/rov/cmd_vertical_vdrive`: Thruster Settings for Verticals
  * vector_drive/thrusterPercents

* `/rov/humidity`: Outputs internal humidity as a relative percentage
* `/rov/pressure`: Outputs internal pressure as millibars
* `/rov/temperature`: Outputs interal temperature as C
* `/rov/ms5837`: Outputs external pressure, temperature, and depth
  * msg5837/ms5837_data

* `tcu/tcu_data`: Data from the TCU Arduino DUE
  * tcu_board_msgs/tcu_board_data
* `tcu/leds`: LED status for TCU
  * std_msgs/ColorRGBA
* `tcu/main_relay`
  * std_msgs/Bool
* `tcu/main_solenoid`
  * std_msgs/Bool


* `/raspicam_node/compressed`: Compressed JPG image from RPI cameras
  * sensor_msgs/CompressedImage
