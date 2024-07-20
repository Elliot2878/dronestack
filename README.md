We used and modified Mavros, Mavlink, and PX4-Autopilot packages

// SITL: <br>
cd PX4-Autopilot
make px4_sitl_default gazebo

// open another terminal <br>
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" <br>

// open another terminal and run ros as usual, catkin_make and source if needed <br>
rosrun dronestack offb_node_sitl <br>
<br>
<br>
<br>
<br>


//HITL: commands from px4 official site <br>
cd PX4-Autopilot <br>
DONT_RUN=1 make px4_sitl_default gazebo-classic <br>

source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default <br>

gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world <br>

start QGroundControl <br>

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" <br>

rosrun dronestack offb_node_hitl <br>

When the mode becomes unknown, run hitl code, and disarmt the drone by QGroundControl. Then, reboot the vehicle <br>

<br>
<br>
<br>
<br>

// real aircraft flying <br>
Turn on the Remote Control. <br>
Adjust COM_ARM_EKF_YAW to a high enough number (like 1.00) <br>
<br>
cd catkin_ws/src/mavros/mavros <br>
source ~/catkin_ws/devel/setup.bash <br>
roslaunch launch/px4.launch fcu_url:="/dev/ttyACM0:115200" <br>
<br>
// Alternatively <br>
// source ~/catkin_ws/devel/setup.bash <br>
// roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200" <br>
<br>
roslaunch vicon_bridge vicon.launch <br>
<br>
<br>
cd ~/catkin_ws <br>
source devel/setup.bash <br>
rosrun dronestack offb_node_sitl <br>

<br>
<br>
<br>
<br>

debug: <br>
if sens_flow_* parameters disappear, you need to restart the onboard computer <br>

<br>
<br>
<br>
<br>

If arm service fails <br>
1. Go to HITL mode. Then, you have to check QGroundControl if it has "Flight Termination Active" before running the code.
To Solve "Flight Termination Active", you should reboot the flight control unit. <br>
This error might happen and prevent you from arming after you terminate your code for unknown reasons. <br>

2. You have to make sure you sent vicon data to vision_pose before switching mode and arming <br>

3. Turn on your remote control <br>

4. Adjust COM_ARM_EKF_YAW to a high enough number (like 1.00) <br>
<br>
<br>
<br>
<br>

// check px4 connection <br>
rostopic echo /mavros/state <br>

// check vicon <br>
rostopic echo /vicon/x500_1/x500_1 <br>
<br>
<br>
<br>
<br>

// check optical flow <br>
rostopic echo /mavros/px4flow/raw/optical_flow_rad <br>
rostopic echo mavros/vision_speed/speed_twist <br>

<br>
<br>
<br>
<br>
 
rosrun mavros mavsafety arm <br>
<br>
cd ~/.ros/log/latest <br>
cat roslaunch-cpsl-nuc13-3307.log <br>
<br>
<br>
rosservice call /mavros/cmd/arming "value: true" <br>
rqt_console <br>
rqt_logger_level <br>







<br>
<br>
<br>
<br>



// Other commands: <br>
//Actuators: you must disable HITL mode, disable kill mode in Radio Controller, and connect to battery before testing actuators. Or, the motor cannot spin. <br>
<br>
// cd <PX4-Autopilot_clone> <br>
// source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default <br>
// export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic <br>
<br>
// roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world <br>
<br>
// rosrun gazebo_ros spawn_model -sdf -file $(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf -model iris -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 <br>
<br>
<br>
// you can also run this to launch gazebo and spawn iris:  <br>
// gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world <br>
// or this to send simulated gps data <br>
// gazebo --verbose ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world <br>

