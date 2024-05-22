#ifndef OFFB_NODE_SITL_H
#define OFFB_NODE_SITL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

class OffboardControlSITL {
public:
    OffboardControlSITL();

    void run();

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub, local_pos_sub, vicon_pos_sub, local_vel_sub;
    ros::Publisher local_pos_pub, local_vel_pub;
    ros::ServiceClient arming_client, set_mode_client;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped target, local_position;
    geometry_msgs::Pose global_position;
    geometry_msgs::TransformStamped vicon_position;

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void viconPosition_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void localVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void initialize_setpoint();
    void main_loop();
    void handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request);
    void handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request);
    void publish_clamped_velocity(ros::Publisher& vel_pub); 
    bool isAtPosition(double x, double y, double z, double offset);
};

#endif // OFFB_NODE_SITL_H
