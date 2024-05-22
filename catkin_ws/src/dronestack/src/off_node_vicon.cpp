#include "dronestack/offb_node_vicon.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

OffboardControlSITL::OffboardControlSITL() {
    // subscriber initialization
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControlSITL::state_cb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardControlSITL::localPosition_cb, this);
    vicon_pos_sub = nh.subscribe<geometry_msgs::TransformStamped>("mavros/local_position/pose", 10, &OffboardControlSITL::viconPosition_cb, this);
    //local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, &OffboardControlSITL::localVelocity_cb, this);



    // publisher initialization
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    //local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // client service initialization (arming and mode)
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

// constexpr double MAX_VELOCITY = 0.0001; // meters per second
// Eigen::Vector3d current_velocity; // to store current velocity


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

void OffboardControlSITL::viconPosition_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    vicon_position = *msg;
}

// void OffboardControlSITL::localVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
//     current_velocity.x() = msg->twist.linear.x;
//     current_velocity.y() = msg->twist.linear.y;
//     current_velocity.z() = msg->twist.linear.z;

//     ROS_INFO("current velocity in Callback: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());
// }


void OffboardControlSITL::initialize_setpoint() {
    // set initial hover position
    target.pose.position.x = 0;
    target.pose.position.y = 0;
    target.pose.position.z = 4;

    for(int i = 100; ros::ok() && i > 0; --i) {

        //publish hover position
        local_pos_pub.publish(target);
        
        // publish_clamped_velocity(local_vel_pub);

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

        // publish_clamped_velocity(local_vel_pub);
        
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

// void OffboardControlSITL::publish_clamped_velocity(ros::Publisher& vel_pub) {
//     ROS_INFO("current velocity in my function: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());

//     Eigen::Vector3d adjusted_velocity = current_velocity;

//     // Check if current velocity exceeds the maximum allowed velocity
//     if (current_velocity.norm() > MAX_VELOCITY) {
//         // Adjust the desired velocity to reduce it
//         adjusted_velocity = MAX_VELOCITY * current_velocity.normalized();
//     }

//     // Prepare and publish the velocity message
//     geometry_msgs::TwistStamped vel_msg;
//     vel_msg.header.stamp = ros::Time::now();
//     vel_msg.twist.linear.x = adjusted_velocity.x();
//     vel_msg.twist.linear.y = adjusted_velocity.y();
//     vel_msg.twist.linear.z = adjusted_velocity.z();

//     ROS_INFO("clamped velocity: x:%.2f, y:%.2f, z:%.2f", vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z);

//     vel_pub.publish(vel_msg);
// }


bool OffboardControlSITL::isAtPosition(double x, double y, double z, double offset) {
    // a function to check if the difference between current position and target position is within a predefined offset
    ROS_INFO("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    Eigen::Vector3d desired(x, y, z);
    Eigen::Vector3d pos(vicon_position.transform.translation.x, vicon_position.transform.translation.y, vicon_position.transform.translation.z);
    return (desired - pos).norm() < offset;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    OffboardControlSITL offboard_control;
    offboard_control.run();
    return 0;
}



