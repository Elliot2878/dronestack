#ifndef MAVROS_OFFBOARD_YAWRATE_TEST_HPP
#define MAVROS_OFFBOARD_YAWRATE_TEST_HPP

#include <ros/ros.h>
#include <atomic>
#include <thread>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

class MavrosOffboardYawrateTest {
public:
    MavrosOffboardYawrateTest();
    virtual ~MavrosOffboardYawrateTest();
    void runTest();

private:
    ros::NodeHandle nh_;
    ros::Publisher att_pub_;
    ros::Subscriber state_sub_, position_sub_, imu_sub_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_position_;
    sensor_msgs::Imu imu_data_;
    std::atomic<bool> continue_publishing_{false};
    std::thread publishing_thread_;
    double des_yawrate_;
    double yawrate_tol_;

    void publishAttitudeSetpoints();
    void checkState(const mavros_msgs::State::ConstPtr& msg);
    void checkPosition(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void checkIMUData(const sensor_msgs::Imu::ConstPtr& msg);
    void setupTest();
    void teardownTest();
    void changeMode(const std::string& mode);
    void armThrottle(bool arm);
    bool testSuccessCriteria();
};

#endif // MAVROS_OFFBOARD_YAWRATE_TEST_HPP
