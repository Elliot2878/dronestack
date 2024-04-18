#ifndef MAVROS_OFFBOARD_POSCTL_TEST_H
#define MAVROS_OFFBOARD_POSCTL_TEST_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <thread>
#include <Eigen/Dense>
#include "dronestack/MavrosTestCommon.h" // Assuming this is a custom include for the base class

class MavrosOffboardPosctlTest : public MavrosTestCommon {
public:
    explicit MavrosOffboardPosctlTest(ros::NodeHandle& nh);
    ~MavrosOffboardPosctlTest();

    void setUp();
    // void tearDown(); // Uncomment if needed
    void sendPos();
    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
    bool isAtPosition(double x, double y, double z, double offset);
    void reachPosition(double x, double y, double z, int timeout);
    void testPosctl();
    geometry_msgs::PoseStamped pos;

// private:
    ros::NodeHandle nh_;
    ros::Publisher pos_setpoint_pub;
    ros::Subscriber extended_state_sub_;
    mavros_msgs::ExtendedState last_extended_state_;
    double radius;
    std::thread pos_thread;
};

#endif // MAVROS_OFFBOARD_POSCTL_TEST_H
