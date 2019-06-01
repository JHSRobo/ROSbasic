cd Github/ROSBasic/ros_workspace
source devel/setup.bash
roslaunch launch_files topside.launch
./WAIT FOR POWER
roslaunch launch_files remote_bottomside.launch &
ssh USER@HOST "./SCRIPT" &
./COPILOT SCRIPT
