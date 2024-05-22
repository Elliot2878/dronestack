#ifndef OFFB_NODE_H
#define OFFB_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/SetModelState.h>
#include <Eigen/Dense>

class OffboardControl {
public:
    OffboardControl();

    void run();

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub, local_pos_sub, gazebo_sub;
    ros::Publisher local_pos_pub, px4_vision_pose_pub, px4_vision_vel_pub;
    ros::ServiceClient arming_client, set_mode_client;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped target;
    geometry_msgs::PoseStamped local_position, gazebo_position, pose_msg;
    geometry_msgs::TwistStamped twist_msg;
    geometry_msgs::Pose global_position;
    geometry_msgs::Point position_point;

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gazebo_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void initialize_setpoint();
    void main_loop();
    void handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request);
    void handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request);
    bool isAtPosition(double x, double y, double z, double offset);
};

#endif // OFFB_NODE_H


