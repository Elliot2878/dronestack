#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dronestack/mavros_offboard_attctl_test.h"


MavrosOffboardAttctlTest::MavrosOffboardAttctlTest() {
    att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    state_sub_ = nh_.subscribe("mavros/state", 10, &MavrosOffboardAttctlTest::checkState, this);
    position_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &MavrosOffboardAttctlTest::checkPosition, this);
}

MavrosOffboardAttctlTest::~MavrosOffboardAttctlTest() {
    continue_publishing_ = false;
    if (publishing_thread_.joinable()) publishing_thread_.join();
}

void MavrosOffboardAttctlTest::checkState(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void MavrosOffboardAttctlTest::checkPosition(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_position_ = *msg;
}

void MavrosOffboardAttctlTest::publishAttitudeSetpoints() {
    ros::Rate rate(10); // 10Hz
    mavros_msgs::AttitudeTarget att_msg;
    att_msg.type_mask = 7; // Ignore body rate
    att_msg.orientation = tf2::toMsg(tf2::Quaternion(-0.25, 0.15, 0, 1));
    att_msg.thrust = 0.7;

    while (ros::ok() && continue_publishing_) {
        att_msg.header.stamp = ros::Time::now();
        att_pub_.publish(att_msg);
        rate.sleep();
    }
}

void MavrosOffboardAttctlTest::setupTest() {
    // Wait for FCU connection
    while (ros::ok() && !current_state_.connected) ros::spinOnce();

    changeMode("OFFBOARD");
    armThrottle(true);

    continue_publishing_ = true;
    publishing_thread_ = std::thread(&MavrosOffboardAttctlTest::publishAttitudeSetpoints, this);
}

void MavrosOffboardAttctlTest::teardownTest() {
    changeMode("AUTO.LAND");
    armThrottle(false);
}

void MavrosOffboardAttctlTest::changeMode(const std::string& mode) {
    mavros_msgs::SetMode srv_set_mode;
    srv_set_mode.request.custom_mode = mode;
    if (!ros::service::call("mavros/set_mode", srv_set_mode)) {
        ROS_ERROR("Set mode failed");
    }
}

void MavrosOffboardAttctlTest::armThrottle(bool arm) {
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = arm;
    if (!ros::service::call("mavros/cmd/arming", srv_arm)) {
        ROS_ERROR("Arming or disarming failed");
    }
}

bool MavrosOffboardAttctlTest::boundaryCrossed() {
    // Example boundary check
    return current_position_.pose.position.x > 200 &&
           current_position_.pose.position.y > 100 &&
           current_position_.pose.position.z > 20;
}

void MavrosOffboardAttctlTest::runTest() {
    setupTest();

    // Main test logic - wait for boundary crossing or timeout
    ros::Rate rate(2); // 2Hz
    bool crossed = false;
    for(int i = 0; ros::ok() && i < 90 * 2; ++i) {
        if (boundaryCrossed()) {
            ROS_INFO("Boundary crossed.");
            crossed = true;
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }

    if (!crossed) ROS_INFO("Failed to cross boundary in time.");

    teardownTest();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_offboard_attctl_test");
    MavrosOffboardAttctlTest testNode;
    testNode.runTest();
    return 0;
}
