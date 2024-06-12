// SITL:
cd PX4-Autopilot
make px4_sitl_default gazebo

// open another terminal
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

// open another terminal and run ros as usual, catkin_make and source if needed
rosrun dronestack offb_node_sitl



//HITL: commands from px4 official site
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic

source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world

start QGroundControl

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

rosrun dronestack offb_node_hitl

When the mode becomes unknown, run hitl code, and disarmt the drone by QGroundControl. Then, reboot the vehicle



// real aircraft flying
Turn on the Remote Control.
Adjust COM_ARM_EKF_YAW to a high enough number (like 1.00)

cd catkin_ws/src/mavros/mavros
source ~/catkin_ws/devel/setup.bash
roslaunch launch/px4.launch fcu_url:="/dev/ttyACM0:115200"

// Alternatively
// source ~/catkin_ws/devel/setup.bash
// roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200"

roslaunch vicon_bridge vicon.launch


cd ~/catkin_ws
source devel/setup.bash
rosrun dronestack offb_node_sitl



debug:
if sens_flow_* parameters disappear, you need to restart the onboard computer


If arm service fails
1. Go to HITL mode. Then, you have to check QGroundControl if it has "Flight Termination Active" before running the code.
To Solve "Flight Termination Active", you should reboot the flight control unit.
This error might happen and prevent you from arming after you terminate your code for unknown reasons.

2. You have to make sure you sent vicon data to vision_pose before switching mode and arming


// check px4 connection
rostopic echo /mavros/state

// check vicon
rostopic echo /vicon/x500_1/x500_1

// check optical flow
rostopic echo /mavros/px4flow/raw/optical_flow_rad
rostopic echo mavros/vision_speed/speed_twist


rosrun mavros mavsafety arm

cd ~/.ros/log/latest
cat roslaunch-cpsl-nuc13-3307.log


rosservice call /mavros/cmd/arming "value: true"
rqt_console
rqt_logger_level










// Other commands:
//Actuators: you must disable HITL mode, disable kill mode in Radio Controller, and connect to battery before testing actuators. Or, the motor cannot spin. 

// cd <PX4-Autopilot_clone>
// source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
// export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

// roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world

// rosrun gazebo_ros spawn_model -sdf -file $(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf -model iris -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0


// you can also run this to launch gazebo and spawn iris: 
// gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
// or this to send simulated gps data
// gazebo --verbose ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world

