#ifndef MAVROS_TEST_COMMON_H
#define MAVROS_TEST_COMMON_H

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <map>
#include <vector>

// ROS message and service types
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPush.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

class MavrosTestCommon
{
public:
    MavrosTestCommon();
    ~MavrosTestCommon() = default; // Define destructor if needed

    void setUp();
    void tearDown();

    // Callback functions
    void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg);
    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void missionWpCallback(const mavros_msgs::WaypointList::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    // Methods for interaction with MAVROS
    void setArm(bool arm, int timeout);
    void setMode(const std::string& mode, int timeout);
    void setParam(const std::string& param_id, mavros_msgs::ParamValue param_value, int timeout);
    void waitForTopics(int timeout);
    // void waitForLandedState(int desired_landed_state, int timeout, int index);
    void waitForVtolState(uint8_t transition, int timeout, int index);
    void clearWps(int timeout);
    void sendWps(const std::vector<mavros_msgs::Waypoint>& waypoints, int timeout);
    void waitForMavType(int timeout);
    void logTopicVars();

// private:
    mavros_msgs::Altitude altitude;
    mavros_msgs::ExtendedState extended_state;
    sensor_msgs::NavSatFix global_position;
    sensor_msgs::Imu imu_data;
    mavros_msgs::HomePosition home_position;
    geometry_msgs::PoseStamped local_position;
    mavros_msgs::WaypointList mission_wp;
    mavros_msgs::State state;
    int mav_type;

    std::map<std::string, bool> sub_topics_ready;

    ros::NodeHandle nh;

    // ROS service clients
    ros::ServiceClient get_param_srv, set_param_srv, set_arming_srv, set_mode_srv, wp_clear_srv, wp_push_srv;

    // ROS subscribers
    ros::Subscriber alt_sub, ext_state_sub, global_pos_sub, imu_data_sub, home_pos_sub, local_pos_sub, mission_wp_sub, state_sub;
};

#endif // MAVROS_TEST_COMMON_H
