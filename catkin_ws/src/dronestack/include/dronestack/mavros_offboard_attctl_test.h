#ifndef MAVROS_OFFBOARD_ATTCTL_TEST_H
#define MAVROS_OFFBOARD_ATTCTL_TEST_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <atomic>

class MavrosOffboardAttctlTest {
public:
    MavrosOffboardAttctlTest();
    ~MavrosOffboardAttctlTest();
    void runTest();

private:
    ros::NodeHandle nh_;
    ros::Publisher att_pub_;
    ros::Subscriber state_sub_, position_sub_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_position_;
    std::atomic<bool> continue_publishing_{false};
    std::thread publishing_thread_;

    void publishAttitudeSetpoints();
    void checkState(const mavros_msgs::State::ConstPtr& msg);
    void checkPosition(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setupTest();
    void teardownTest();
    void changeMode(const std::string& mode);
    void armThrottle(bool arm);
    bool boundaryCrossed();
};

#endif // MAVROS_OFFBOARD_ATTCTL_TEST_H
