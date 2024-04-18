
#include "dronestack/MavrosOffboardPosctlTest.h"

MavrosOffboardPosctlTest::MavrosOffboardPosctlTest(ros::NodeHandle& nh) : nh_(nh) {
    pos.header.frame_id = "base_footprint";
    pos_setpoint_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &MavrosOffboardPosctlTest::extendedStateCallback, this);
    pos_thread = std::thread(&MavrosOffboardPosctlTest::sendPos, this);
    pos_thread.detach();
}

MavrosOffboardPosctlTest::~MavrosOffboardPosctlTest() {
    if (pos_thread.joinable()) {
        pos_thread.join();
    }
}


void MavrosOffboardPosctlTest::setUp() {
    MavrosTestCommon::setUp();
    radius = 1.0;
}

void MavrosOffboardPosctlTest::sendPos() {
    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        pos.header.stamp = ros::Time::now();
        pos_setpoint_pub.publish(pos);

        try {
            rate.sleep();
        } catch (const ros::Exception& e) {
            ROS_ERROR("Error in sendPos: %s", e.what());
            break;
        }
    }
}

bool MavrosOffboardPosctlTest::isAtPosition(double x, double y, double z, double offset) {
    ROS_DEBUG("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    Eigen::Vector3d desired(x, y, z);
    Eigen::Vector3d pos(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    return (desired - pos).norm() < offset;
}

void MavrosOffboardPosctlTest::reachPosition(double x, double y, double z, int timeout) {
    // zyl: let it fly to position x y z.
    pos.pose.position.x = x;
    pos.pose.position.y = y;
    pos.pose.position.z = z;

    ROS_INFO("attempting to reach position | x: %f, y: %f, z: %f | current position x: %.2f, y: %.2f, z: %.2f", x, y, z, local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);

    double yaw = 0 * M_PI / 180.0; // Convert degrees to radians
    auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    pos.pose.orientation = quaternion;

    ros::Rate rate(2); // 2 Hz
    bool reached = false;
    for (int i = 0; i < timeout * 2 && ros::ok(); ++i) {
        if (isAtPosition(x, y, z, radius)) {
            ROS_INFO("position reached | seconds: %d of %d", i / 2, timeout);
            reached = true;
            break;
        }
        try {
            rate.sleep();
        } catch (const ros::Exception& e) {
            ROS_ERROR("Error in reachPosition: %s", e.what());
            break;
        }
    }
    ROS_ASSERT_MSG(reached, "took too long to get to position");
}


// Member variable to store the last known extended state
mavros_msgs::ExtendedState last_extended_state_;

// Callback for extended state
void MavrosOffboardPosctlTest::extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    last_extended_state_ = *msg;
}


void MavrosOffboardPosctlTest::testPosctl() {
    // Test logic here, similar to Python version
    ROS_INFO("Test offboard position control");

    // Make sure the simulation is ready to start the mission
    waitForTopics(60);
    
    // Subscribe to the extended state topic (add this line in your setup/constructor)
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, &MavrosOffboardPosctlTest::extendedStateCallback, this);

    // Wait for landed state
    // to replace: waitForLandedState(mavutil::MAV_LANDED_STATE_ON_GROUND, 10, -1);
    ros::Rate rate(10); // 10 Hz
    auto start = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce(); // Allow callbacks to be processed
        if (last_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
            ROS_INFO("Drone is on the ground.");
            break;
        }
        if ((ros::Time::now() - start).toSec() > 10) { // Timeout after 10 seconds
            ROS_ERROR("Timeout waiting for landed state.");
            break;
        }
        rate.sleep();
    }


    logTopicVars();
    // Exempting failsafe from lost RC to allow offboard
    mavros_msgs::ParamValue rcl_except;
    rcl_except.integer = 1 << 2;
    rcl_except.real = 0.0;
    setParam("COM_RCL_EXCEPT", rcl_except, 5);
    setMode("OFFBOARD", 5);
    setArm(true, 5);

    ROS_INFO("Run mission");
    std::vector<std::vector<double>> positions = {{0, 0, 0}, {50, 50, 20}, {50, -50, 20}, {-50, -50, 20}, {0, 0, 20}};

    for (size_t i = 0; i < positions.size(); ++i) {
        reachPosition(positions[i][0], positions[i][1], positions[i][2], 30);
    }

    setMode("AUTO.LAND", 5);
    // waitForLandedState(mavutil::MAV_LANDED_STATE_ON_GROUND, 45, 0);

    rate = ros::Rate(45); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce(); // Allow callbacks to be processed
        if (last_extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
            ROS_INFO("Drone is on the ground.");
            break;
        }
        if ((ros::Time::now() - start).toSec() > 10) { // Timeout after 10 seconds
            ROS_ERROR("Timeout waiting for landed state.");
            break;
        }
        rate.sleep();
    }

    setArm(false, 5);
}

// // main function as before
// int main(int argc, char **argv) {
//     // ros::init(argc, argv, "test_node");
//     // // MavrosOffboardPosctlTest test;
//     // ros::NodeHandle nh;
//     // MavrosOffboardPosctlTest test(nh);
//     // test.testPosctl();
//     // return 0;

//     ros::init(argc, argv, "mavros_offboard_posctl_test");
//     ros::NodeHandle nh;
//     MavrosOffboardPosctlTest test(nh);
//     ros::spin();
//     return 0;
// }


int main(int argc, char **argv) {
    ros::init(argc, argv, "mavros_offboard_posctl_test");
    
    // Instantiate the class and set up ROS connections
    // MavrosTestCommon test;
    // test.setUp();

    // Example usage of class methods. This could be more elaborate based on actual needs.
    // For instance, if you have specific tests or initialization procedures to run
    // before entering the ROS event loop, you could do that here.

    ros::NodeHandle nh;
    MavrosOffboardPosctlTest posTest(nh);

    posTest.setUp();
    posTest.testPosctl();

    // Enter the ROS event loop
    ros::spin();

    return 0;
}









