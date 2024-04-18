#include "mavros_offboard_yawrate_test.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MavrosOffboardYawrateTest::MavrosOffboardYawrateTest() : des_yawrate_(0.1), yawrate_tol_(0.02) {
    att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    state_sub_ = nh_.subscribe("mavros/state", 10, &MavrosOffboardYawrateTest::checkState, this);
    position_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &MavrosOffboardYawrateTest::checkPosition, this);
    imu_sub_ = nh_.subscribe("mavros/imu/data", 10, &MavrosOffboardYawrateTest::checkIMUData, this);
}

MavrosOffboardYawrateTest::~MavrosOffboardYawrateTest() {
    continue_publishing_ = false;
    if (publishing_thread_.joinable()) publishing_thread_.join();
}

void MavrosOffboardYawrateTest::checkState(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void MavrosOffboardYawrateTest::checkPosition(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_position_ = *msg;
}

void MavrosOffboardYawrateTest::checkIMUData(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_data_ = *msg;
}

void MavrosOffboardYawrateTest::publishAttitudeSetpoints() {
    ros::Rate rate(10); // 10Hz
    mavros_msgs::AttitudeTarget att_msg;
    att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
    // Orientation is not directly controlled; yaw rate is specified instead.
    att_msg.body_rate.z = des_yawrate_;
    att_msg.thrust = 0.59;

    while (ros::ok() && continue_publishing_) {
        att_msg.header.stamp = ros::Time::now();
        att_pub_.publish(att_msg);
        rate.sleep();
    }
}

void MavrosOffboardYawrateTest::setupTest() {
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        ros::Rate(1.0).sleep();
    }

    changeMode("OFFBOARD");
    armThrottle(true);

    continue_publishing_ = true;
    publishing_thread_ = std::thread(&MavrosOffboardYawrateTest::publishAttitudeSetpoints, this);
}

void MavrosOffboardYawrateTest::teardownTest() {
    changeMode("AUTO.LAND");
    armThrottle(false);
}

void MavrosOffboardYawrateTest::changeMode(const std::string& mode) {
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = mode;
    if (!ros::service::call("mavros/set_mode", set_mode)) {
        ROS_ERROR("Failed to set mode: %s", mode.c_str());
    }
}

void MavrosOffboardYawrateTest::armThrottle(bool arm) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    if (!ros::service::call("mavros/cmd/arming", arm_cmd)) {
        ROS_ERROR("Failed to %s vehicle", arm ? "arm" : "disarm");
    }
}

bool MavrosOffboardYawrateTest::testSuccessCriteria() {
    double x = current_position_.pose.position.x;
    double y = current_position_.pose.position.y;
    double z = current_position_.pose.position.z;
    double yr = imu_data_.angular_velocity.z;

    bool within_boundaries = x < 5 && x > -5 && y < 5 && y > -5 && z > 10;
    bool yawrate_achieved = std::abs(yr - des_yawrate_) < yawrate_tol_;

    return within_boundaries && yawrate_achieved;
}

void MavrosOffboardYawrateTest::runTest() {
    setupTest();

    ros::Rate rate(2); // 2Hz
    bool test_successful = false;
    for (int i = 0; ros::ok() && i < 90 * 2; ++i) {
        if (testSuccessCriteria()) {
            ROS_INFO("Test successful. Final altitude and yaw rate achieved.");
            test_successful = true;
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }

    if (!test_successful) {
        ROS_INFO("Failed to achieve final altitude and yaw rate in time.");
    }

    teardownTest();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_offboard_yawrate_test_cpp");
    MavrosOffboardYawrateTest testNode;
    testNode.runTest();
    return 0;
}

