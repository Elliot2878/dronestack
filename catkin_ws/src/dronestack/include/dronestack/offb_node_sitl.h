#ifndef OFFB_NODE_SITL_H
#define OFFB_NODE_SITL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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
    ros::Subscriber state_sub, local_pos_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client, set_mode_client;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped target;
    geometry_msgs::PoseStamped local_position;
    geometry_msgs::Pose global_position;

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initialize_setpoint();
    void main_loop();
    void handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request);
    void handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request);
    bool isAtPosition(double x, double y, double z, double offset);
};

#endif // OFFB_NODE_SITL_H
