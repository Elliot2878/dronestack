#include "dronestack/offb_node.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/SetModelState.h>
#include <Eigen/Dense>

OffboardControl::OffboardControl() {

    // subscriber initialization
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControl::state_cb, this); 
    local_pos_sub = nh.subscribe("mavros/local_position/pose", 10, &OffboardControl::localPosition_cb, this);
    gazebo_sub = nh.subscribe("/gazebo/model_states", 10, &OffboardControl::gazebo_cb, this); // get current position from gazebo
    
    // publisher initialization
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);        
    px4_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1); // publish position and velocity to vision pose for replacing GPS
    px4_vision_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/vision_pose/twist", 1);

    // client service initialization (arming and mode)
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        
}

void OffboardControl::run() {

    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected) {
        // if not connected, keep ros running and wait for instruction
        ros::spinOnce();
        rate.sleep();
    }

    // call initialize setpoint and main_loop function
    initialize_setpoint();
    main_loop();
}
    

// Callback functions
void OffboardControl::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    // current_state stores the mode of the drone (eg, arm, disarm, etc)
    current_state = *msg;
}


void OffboardControl::localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // local_position stores the current position and orientation of the drone
    local_position = *msg;
}

void OffboardControl::gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // listen to position and velocity of the drone in Gazebo

    if (msg->name.size() > 1 && msg->pose.size() > 1 && msg->twist.size() > 1) {
        // Create PoseStamped message for position
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";  // or any other frame of reference
        pose_msg.pose.position = msg->pose[1].position;
        pose_msg.pose.orientation = msg->pose[1].orientation;

        // Create TwistStamped message for velocity
        twist_msg.header.stamp = pose_msg.header.stamp;
        twist_msg.header.frame_id = "world";
        twist_msg.twist.linear = msg->twist[1].linear;
        twist_msg.twist.angular = msg->twist[1].angular;

    } else {
        ROS_WARN("Not enough models in the simulation to extract data.");
    }
}



void OffboardControl::initialize_setpoint() {
    ROS_INFO("Enter initialize_setpoint");

    // set initial hover position
    target.pose.position.x = 0;
    target.pose.position.y = 0;
    target.pose.position.z = 4;

    for(int i = 100; ros::ok() && i > 0; --i) {
        // publish position and velocity to vision pose for replacing GPS
        px4_vision_pose_pub.publish(pose_msg);
        px4_vision_vel_pub.publish(twist_msg);


        //publish hover position
        local_pos_pub.publish(target);



        ros::spinOnce();
        ros::Rate(20.0).sleep();
    }
}

void OffboardControl::main_loop() {
    ROS_INFO("Enter main loop");
    
    // initialize variables
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode land_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_mode.request.custom_mode = "AUTO.LAND";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    // Waypoints include landing by descending to z=0 after the last position
    std::vector<std::vector<double>> positions = {{3, 3, 4}, {3, -3, 4}, {-3, -3, 4}, {-3, 3, 4}, {3, 3, 4}, {0, 0, 4}, {0, 0, 0}};

    int count = 0;

    while(ros::ok()) {
        // control loop

        // check if connected to PX4
        ROS_INFO("Connected to PX4: %s", current_state.connected ? "Yes" : "No");
        if(!current_state.connected) {
            ROS_ERROR("Failed to connect PX4");
        }


        // switch mode to offb_node
        handle_mode_switch(offb_set_mode, last_request);

        // arm the drone
        handle_arm_command(arm_cmd, last_request);

        // publish position and velocity to vision pose for replacing GPS
        px4_vision_pose_pub.publish(pose_msg);
        px4_vision_vel_pub.publish(twist_msg);

        // publish target setpoint to mavros
        local_pos_pub.publish(target);

        // check local position
        ROS_INFO("Check Local position: x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);

        // check if the drone reaches the target position. If it is true, drone moves to next position
        if(isAtPosition(target.pose.position.x, target.pose.position.y, target.pose.position.z, 0.5)) {
            if(count >= positions.size()) {
                ROS_INFO("End of waypoints navigations. Preparing to land.");
                if(set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
                    ROS_INFO("Landing initiated.");
                    break; // Exit the loop to stop publishing waypoints and let the drone land.
                }
            }
            ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position.", target.pose.position.x, target.pose.position.y, target.pose.position.z);

            target.pose.position.x = positions[count][0];
            target.pose.position.y = positions[count][1];
            target.pose.position.z = positions[count][2];
            count++;
        }

        ros::spinOnce();
        ros::Rate(20.0).sleep();
    }
}

void OffboardControl::handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
    // function to switch mode to "OFFBOARD"
    ROS_INFO("Enter handle_mode_switch function");
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
        }
        else {
            ROS_ERROR("Offboard failed 1");
            }
    }
    else if(current_state.mode == "OFFBOARD") {
        ROS_INFO("Already Offbaord");
    }
    else if((ros::Time::now() - last_request <= ros::Duration(2.0))) {
        ros::Time time_now = ros::Time::now();
        ROS_ERROR("Current time: %u seconds and %u nanoseconds", time_now.sec, time_now.nsec);
        ROS_ERROR("comparison failed. The last_request is %u seconds and %u nanoseconds", last_request.sec, last_request.nsec);

    }
    else {
        ROS_ERROR("Offboard failed 2");
    }
}

void OffboardControl::handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
    // change disarm to arm

    ROS_INFO("Enter handle_arm_command function");
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        arm_cmd.request.value = true;
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            last_request = ros::Time::now();
        }
        else {
            ROS_ERROR("cannot armed 1");
            if(! arming_client.call(arm_cmd)) ROS_ERROR("arming_client call failed");
            if(! arm_cmd.response.success) ROS_ERROR("arm_cmd.response.success failed");
        }
    }
    else if(current_state.armed ) {
        ROS_INFO("Already armed");
    }
    else {
        ROS_ERROR("cannot armed 2");
    }
    
}


bool OffboardControl::isAtPosition(double x, double y, double z, double offset) {
    // a function to check if the difference between current position and target position is within a predefined offset
    ROS_DEBUG("current position | x:%.2f, y:%.2f, z:%.2f", pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    Eigen::Vector3d desired(x, y, z);
    Eigen::Vector3d pos(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    return (desired - pos).norm() < offset;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    OffboardControl offboard_control;
    offboard_control.run();
    return 0;
}



