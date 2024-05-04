#include "dronestack/offb_node_sitl.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

OffboardControlSITL::OffboardControlSITL() {
    // subscriber initialization
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControlSITL::state_cb, this);
    local_pos_sub = nh.subscribe("mavros/local_position/pose", 10, &OffboardControlSITL::localPosition_cb, this);

    // publisher initialization
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // client service initialization (arming and mode)
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void OffboardControlSITL::run() {
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
void OffboardControlSITL::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    // current_state stores the mode of the drone (eg, arm, disarm, etc)
    current_state = *msg;
}

void OffboardControlSITL::localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // local_position stores the current position and orientation of the drone
    local_position = *msg;
}

void OffboardControlSITL::initialize_setpoint() {
    // set initial hover position
    target.pose.position.x = 0;
    target.pose.position.y = 0;
    target.pose.position.z = 4;

    for(int i = 100; ros::ok() && i > 0; --i) {

        //publish hover position
        local_pos_pub.publish(target);
        ros::spinOnce();
        ros::Rate(20.0).sleep();
    }
}

void OffboardControlSITL::main_loop() {

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

        // switch mode to offb_node
        handle_mode_switch(offb_set_mode, last_request);

        // arm the drone
        handle_arm_command(arm_cmd, last_request);

        // publish target setpoint to mavros
        local_pos_pub.publish(target);
        
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

void OffboardControlSITL::handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
    // function to switch mode to "OFFBOARD"
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
        }
    }
}

void OffboardControlSITL::handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
    // change disarm to arm
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            last_request = ros::Time::now();
        }
    }
}


bool OffboardControlSITL::isAtPosition(double x, double y, double z, double offset) {
    // a function to check if the difference between current position and target position is within a predefined offset
    ROS_DEBUG("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    Eigen::Vector3d desired(x, y, z);
    Eigen::Vector3d pos(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    return (desired - pos).norm() < offset;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    OffboardControlSITL offboard_control;
    offboard_control.run();
    return 0;
}



