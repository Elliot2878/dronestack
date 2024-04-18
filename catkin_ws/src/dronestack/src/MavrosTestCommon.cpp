#include <ros/ros.h>
#include <math.h>
#include <string>
#include <map>
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
#include "dronestack/MavrosTestCommon.h"

// might or might not miss SetModeRequest, from pymavlink import mavutilm, from six.moves import xrange

MavrosTestCommon::MavrosTestCommon() {
    altitude = mavros_msgs::Altitude();
    extended_state = mavros_msgs::ExtendedState();
    global_position = sensor_msgs::NavSatFix();
    imu_data = sensor_msgs::Imu();
    home_position = mavros_msgs::HomePosition();
    local_position = geometry_msgs::PoseStamped();
    mission_wp = mavros_msgs::WaypointList();
    state = mavros_msgs::State();
    mav_type = 0; // Assuming 'mav_type' is some sort of integer. Need to define properly based on usage.

    sub_topics_ready["alt"] = false;
    sub_topics_ready["ext_state"] = false;
    sub_topics_ready["global_pos"] = false;
    sub_topics_ready["home_pos"] = false;
    sub_topics_ready["local_pos"] = false;
    sub_topics_ready["mission_wp"] = false;
    sub_topics_ready["state"] = false;
    sub_topics_ready["imu"] = false;

    // ROS services will be initialized in setup due to the need of a NodeHandle
}

void MavrosTestCommon::setUp() {
    ros::NodeHandle nh;
    int service_timeout = 30; // Assuming seconds
    ROS_INFO("Waiting for ROS services");
    
    // The services
    ros::service::waitForService("mavros/param/get", ros::Duration(service_timeout));
    ros::service::waitForService("mavros/param/set", ros::Duration(service_timeout));
    ros::service::waitForService("mavros/cmd/arming", ros::Duration(service_timeout));
    ros::service::waitForService("mavros/mission/push", ros::Duration(service_timeout));
    ros::service::waitForService("mavros/mission/clear", ros::Duration(service_timeout));
    ros::service::waitForService("mavros/set_mode", ros::Duration(service_timeout));
    ROS_INFO("ROS services are up");

    // might or might  not need try and except "failed to connect to services"

    get_param_srv = nh.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
    set_param_srv = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    set_arming_srv = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    wp_clear_srv = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    wp_push_srv = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    



    // The subscribers
    alt_sub = nh.subscribe("mavros/altitude", 10, &MavrosTestCommon::altitudeCallback, this);
    ext_state_sub = nh.subscribe("mavros/extended_state", 10, &MavrosTestCommon::extendedStateCallback, this);
    global_pos_sub = nh.subscribe("mavros/global_position/global", 10, &MavrosTestCommon::globalPositionCallback, this);
    imu_data_sub = nh.subscribe("mavros/imu/data", 10, &MavrosTestCommon::imuDataCallback, this);
    home_pos_sub = nh.subscribe("mavros/home_position/home", 10, &MavrosTestCommon::homePositionCallback, this);
    local_pos_sub = nh.subscribe("mavros/local_position/pose", 10, &MavrosTestCommon::localPositionCallback, this);
    mission_wp_sub = nh.subscribe("mavros/mission/waypoints", 10, &MavrosTestCommon::missionWpCallback, this);
    state_sub = nh.subscribe("mavros/state", 10, &MavrosTestCommon::stateCallback, this);
}

    // might not need tearDown for cpp since nh automatically does that
void MavrosTestCommon::tearDown() {
    // Log the state of various topic-related variables
    ROS_INFO("Shutting down, final states:");
    ROS_INFO("Altitude AMSL: %f", altitude.amsl);
    ROS_INFO("Extended State: %d", extended_state.vtol_state);
    ROS_INFO("Global Position (Lat, Lon, Alt): (%f, %f, %f)", global_position.latitude, global_position.longitude, global_position.altitude);
    ROS_INFO("IMU Data (Orientation - x, y, z, w): (%f, %f, %f, %f)", imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    ROS_INFO("Home Position (Lat, Lon, Alt): (%f, %f, %f)", home_position.geo.latitude, home_position.geo.longitude, home_position.geo.altitude);
    ROS_INFO("Local Position (Position - x, y, z): (%f, %f, %f)", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    ROS_INFO("Mission Waypoints Count: %lu", mission_wp.waypoints.size());
    ROS_INFO("State: (Connected: %s, Armed: %s, Guided: %s)", state.connected ? "Yes" : "No", state.armed ? "Yes" : "No", state.guided ? "Yes" : "No");

    // Explicitly shut down subscribers if necessary
    alt_sub.shutdown();
    ext_state_sub.shutdown();
    global_pos_sub.shutdown();
    imu_data_sub.shutdown();
    home_pos_sub.shutdown();
    local_pos_sub.shutdown();
    mission_wp_sub.shutdown();
    state_sub.shutdown();

    // Explicitly shut down service clients if necessary
    get_param_srv.shutdown();
    set_param_srv.shutdown();
    set_arming_srv.shutdown();
    set_mode_srv.shutdown();
    wp_clear_srv.shutdown();
    wp_push_srv.shutdown();

    ROS_INFO("Resources and subscribers shut down.");
}


// private:
//     mavros_msgs::Altitude altitude;
//     mavros_msgs::ExtendedState extended_state;
//     sensor_msgs::NavSatFix global_position;
//     sensor_msgs::Imu imu_data;
//     mavros_msgs::HomePosition home_position;
//     geometry_msgs::PoseStamped local_position;
//     mavros_msgs::WaypointList mission_wp;
//     mavros_msgs::State state;
//     int mav_type; // Placeholder type

//     std::map<std::string, bool> sub_topics_ready;

//     // ROS service clients
//     ros::ServiceClient get_param_srv, set_param_srv, set_arming_srv, set_mode_srv, wp_clear_srv, wp_push_srv, set_arming_client;

//     // ROS subscribers
//     ros::Subscriber alt_sub, ext_state_sub, global_pos_sub, imu_data_sub, home_pos_sub, local_pos_sub, mission_wp_sub, state_sub;


//     ros::NodeHandle nh;


// void MavrosTestCommon::extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
//     extended_state = *msg;
// }

// void MavrosTestCommon::missionWpCallback(const mavros_msgs::WaypointList::ConstPtr& msg) {
//     mission_wp = *msg;
// }

    // Helper methods for enum to string conversions (Assuming implementation exists)
    // std::string getVTOLStateName(uint8_t vtol_state);
    // std::string getLandedStateName(uint8_t landed_state);
    // std::string getSystemStatusName(uint8_t system_status);

std::string getVTOLStateName(uint8_t vtol_state) {
    switch(vtol_state) {
        case mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED:
            return "UNDEFINED";
        case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_FW:
            return "TRANSITION_TO_FW";
        case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_MC:
            return "TRANSITION_TO_MC";
        case mavros_msgs::ExtendedState::VTOL_STATE_MC:
            return "MC";
        case mavros_msgs::ExtendedState::VTOL_STATE_FW:
            return "FW";
        default:
            return "UNKNOWN";
    }
}

std::string getLandedStateName(uint8_t landed_state) {
    switch(landed_state) {
        case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED:
            return "UNDEFINED";
        case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
            return "ON_GROUND";
        case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
            return "IN_AIR";
        case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
            return "TAKEOFF";
        case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
            return "LANDING";
        default:
            return "UNKNOWN";
    }
}

std::string getSystemStatusName(uint8_t status) {
    switch (status) {
        case 0: // MAV_STATE_UNINIT, assuming 0 is the enum value for UNINIT
            return "Uninitialized";
        case 1: // MAV_STATE_BOOT
            return "Booting";
        case 2: // MAV_STATE_CALIBRATED
            return "Calibrated";
        case 3: // MAV_STATE_STANDBY
            return "Standby";
        case 4: // MAV_STATE_ACTIVE
            return "Active";
        case 5: // MAV_STATE_CRITICAL
            return "Critical";
        case 6: // MAV_STATE_EMERGENCY
            return "Emergency";
        case 7: // MAV_STATE_POWEROFF
            return "Poweroff";
        case 8: // MAV_STATE_FLIGHT_TERMINATION
            return "Flight Termination";
        default:
            return "Unknown";
    }
}


// Callback functions
void MavrosTestCommon::altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg) {
    altitude = *msg;
    if (!sub_topics_ready["alt"] && !std::isnan(msg->amsl))
    {
        sub_topics_ready["alt"] = true;
    }
}

void MavrosTestCommon::extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    if (extended_state.vtol_state != msg->vtol_state) {
        ROS_INFO_STREAM("VTOL state changed from " << getVTOLStateName(extended_state.vtol_state)
                            << " to " << getVTOLStateName(msg->vtol_state));
    }

    if (extended_state.landed_state != msg->landed_state) {
        ROS_INFO_STREAM("Landed state changed from " << getLandedStateName(extended_state.landed_state)
                            << " to " << getLandedStateName(msg->landed_state));
    }

    extended_state = *msg;

    sub_topics_ready["ext_state"] = true;
}

void MavrosTestCommon::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position = *msg;
    sub_topics_ready["global_pos"] = true;
}

void MavrosTestCommon::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_data = *msg;
    sub_topics_ready["imu"] = true;
}

void MavrosTestCommon::homePositionCallback(const mavros_msgs::HomePosition::ConstPtr& msg) {
    home_position = *msg;
    sub_topics_ready["home_pos"] = true;
}

void MavrosTestCommon::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_position = *msg;
    sub_topics_ready["local_pos"] = true;
}

void MavrosTestCommon::missionWpCallback(const mavros_msgs::WaypointList::ConstPtr& msg) {
    if (mission_wp.current_seq != msg->current_seq) {
        ROS_INFO_STREAM("Current mission waypoint sequence updated: " << msg->current_seq);
    }

    mission_wp = *msg;
    sub_topics_ready["mission_wp"] = true;
}

void MavrosTestCommon::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    if (state.armed != msg->armed) {
        ROS_INFO_STREAM("Armed state changed from " << state.armed << " to " << msg->armed);
    }

    if (state.connected != msg->connected) {
        ROS_INFO_STREAM("Connected changed from " << state.connected << " to " << msg->connected);
    }

    if (state.mode != msg->mode) {
        ROS_INFO_STREAM("Mode changed from " << state.mode << " to " << msg->mode);
    }

    if (state.system_status != msg->system_status) {
        ROS_INFO_STREAM("System status changed from " << getSystemStatusName(state.system_status)
                            << " to " << getSystemStatusName(msg->system_status));
    }

    state = *msg;
    if (!sub_topics_ready["state"] && msg->connected) {
        sub_topics_ready["state"] = true;
    }
}

void MavrosTestCommon::setArm(bool arm, int timeout) {
    ROS_INFO_STREAM("Setting FCU arm: " << arm);
    bool old_arm = state.armed;
    ros::Rate rate(1); // 1 Hz
    bool arm_set = false;

    for (int i = 0; i < timeout; ++i) {
        if (state.armed == arm) {
            arm_set = true;
            ROS_INFO_STREAM("Set arm success | Seconds: " << i << " of " << timeout);
            break;
        } else {
            mavros_msgs::CommandBool srv;
            srv.request.value = arm;

            if (!set_arming_srv.call(srv) || !srv.response.success) {
                ROS_ERROR("Failed to send arm command");
            }
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(arm_set, "Failed to set arm | New arm: %d, Old arm: %d | Timeout(seconds): %d", arm, old_arm, timeout);
}

void MavrosTestCommon::setMode(const std::string& mode, int timeout) {
    ROS_INFO_STREAM("Setting FCU mode: " << mode);
    std::string old_mode = state.mode;
    ros::Rate rate(1); // 1 Hz
    bool mode_set = false;

    for (int i = 0; i < timeout; ++i) {
        if (state.mode == mode) {
            mode_set = true;
            ROS_INFO_STREAM("Set mode success | seconds: " << i << " of " << timeout);
            break;
        } else {
            mavros_msgs::SetMode set_mode;
            set_mode.request.custom_mode = mode;
            if (!set_mode_srv.call(set_mode) || !set_mode.response.mode_sent) {
                ROS_ERROR("Failed to send mode command");
            }
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(mode_set, "Failed to set mode | new mode: %s, old mode: %s | timeout(seconds): %d", mode.c_str(), old_mode.c_str(), timeout);
}

void MavrosTestCommon::setParam(const std::string& param_id, mavros_msgs::ParamValue param_value, int timeout) {
    double value = (param_value.integer != 0) ? param_value.integer : param_value.real;
    ROS_INFO_STREAM("Setting PX4 parameter: " << param_id << " with value " << value);
    ros::Rate rate(1); // 1 Hz
    bool param_set = false;
    mavros_msgs::ParamSet srv;
    srv.request.param_id = param_id;
    srv.request.value = param_value;

    for (int i = 0; i < timeout; ++i) {
        if (set_param_srv.call(srv) && srv.response.success) {
            ROS_INFO_STREAM("Param " << param_id << " set to " << value << " | seconds: " << i << " of " << timeout);
            param_set = true;
            break;
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(param_set, "Failed to set param | param_id: %s, param_value: %f | timeout(seconds): %d", param_id.c_str(), value, timeout);
}

void MavrosTestCommon::waitForTopics(int timeout) {
    ROS_INFO("Waiting for subscribed topics to be ready");
    ros::Rate rate(1); // 1 Hz
    bool simulation_ready = false;

    for (int i = 0; i < timeout; ++i) {
        if (std::all_of(sub_topics_ready.begin(), sub_topics_ready.end(), [](const std::pair<std::string, bool>& p) { return p.second; })) {
            simulation_ready = true;
            ROS_INFO_STREAM("Simulation topics ready | seconds: " << i << " of " << timeout);
            break;
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(simulation_ready, "Failed to hear from all subscribed simulation topics | timeout(seconds): %d", timeout);
}

    // void waitForLandedState(int desired_landed_state, int timeout, int index) {
    //     ROS_INFO("Waiting for landed state | state: %d, index: %d", desired_landed_state, index);
    //     ros::Rate rate(10); // 10 Hz
    //     bool landed_state_confirmed = false;

    //     for (int i = 0; i < timeout * 10; ++i) {
    //         if (extended_state.landed_state == desired_landed_state) {
    //             landed_state_confirmed = true;
    //             ROS_INFO("Landed state confirmed | seconds: %f of %d", i / 10.0, timeout);
    //             break;
    //         }

    //         rate.sleep();
    //     }

    //     ROS_ASSERT_MSG(landed_state_confirmed, "Landed state not detected | desired: %d, current: %d | index: %d, timeout(seconds): %d",
    //                    desired_landed_state, extended_state.landed_state, index, timeout);
    // }

void MavrosTestCommon::waitForVtolState(uint8_t transition, int timeout, int index) {
    ROS_INFO("Waiting for VTOL transition | transition: %d, index: %d", transition, index);
    ros::Rate rate(10); // 10 Hz
    bool transitioned = false;

    for (int i = 0; i < timeout * 10; ++i) {
        if (transition == extended_state.vtol_state) {
            ROS_INFO("Transitioned | seconds: %f of %d", i / 10.0, timeout);
            transitioned = true;
            break;
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(transitioned, "Transition not detected | desired: %d, current: %d | index: %d timeout(seconds): %d",
                    transition, extended_state.vtol_state, index, timeout);
}

void MavrosTestCommon::clearWps(int timeout) {
    ROS_INFO("Clearing waypoints");
    ros::Rate rate(1); // 1 Hz
    bool wps_cleared = false;

    for (int i = 0; i < timeout; ++i) {
        mavros_msgs::WaypointClear srv;
        if (wp_clear_srv.call(srv) && srv.response.success) {
            wps_cleared = true;
            ROS_INFO("Clear waypoints success | seconds: %d of %d", i, timeout);
            break;
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(wps_cleared, "Failed to clear waypoints | timeout(seconds): %d", timeout);
}

void MavrosTestCommon::sendWps(const std::vector<mavros_msgs::Waypoint>& waypoints, int timeout) {
    ROS_INFO("Sending mission waypoints");
    if (!mission_wp.waypoints.empty()) {
        ROS_INFO("FCU already has mission waypoints");
    }

    ros::Rate rate(1); // 1 Hz
    bool wps_sent = false, wps_verified = false;

    for (int i = 0; i < timeout; ++i) {
        if (!wps_sent) {
            mavros_msgs::WaypointPush srv;
            srv.request.start_index = 0;
            srv.request.waypoints = waypoints; // zyl: send waypoints
            if (wp_push_srv.call(srv) && srv.response.success) {
                wps_sent = true;
                ROS_INFO("Waypoints successfully transferred");
            }
        } else {
            if (waypoints.size() == mission_wp.waypoints.size()) {
                ROS_INFO("Number of waypoints transferred: %lu", waypoints.size());
                wps_verified = true;
            }
        }

        if (wps_sent && wps_verified) {
            ROS_INFO("Send waypoints success | seconds: %d of %d", i, timeout);
            break;
        }

        rate.sleep();
    }

    ROS_ASSERT_MSG(wps_sent && wps_verified, "Mission could not be transferred and verified | timeout(seconds): %d", timeout);
}

void MavrosTestCommon::waitForMavType(int timeout) {
    ROS_INFO("Waiting for MAV_TYPE");
    ros::Rate rate(1); // 1 Hz
    bool success = false;

    for (int i = 0; i < timeout; ++i) {
        mavros_msgs::ParamGet srv;
        srv.request.param_id = "MAV_TYPE";
        if (get_param_srv.call(srv) && srv.response.success) {
            mav_type = srv.response.value.integer;
            // Assuming enum translation is handled elsewhere or differently in C++
            ROS_INFO("MAV_TYPE received | type: %d | seconds: %d of %d", mav_type, i, timeout);
            success = true;
            break;
        }

        rate.sleep();
    }

    // In C++, we generally handle errors differently than Python assertions.
    // The following is an illustrative placeholder.
    if (!success) {
        ROS_ERROR("MAV_TYPE param get failed | timeout(seconds): %d", timeout);
    }
}

void MavrosTestCommon::logTopicVars() {
    ROS_INFO("========================");
    ROS_INFO("===== topic values =====");
    ROS_INFO("========================");
    // Direct logging of complex types like this might not be straightforward in C++
    // You may need to convert each field to a string or use ROS_INFO_STREAM for logging
    ROS_INFO("altitude:\n"); // Placeholder, implement similar for actual logging
    ROS_INFO("========================");
    ROS_INFO("extended_state:\n"); // As above, placeholder
    ROS_INFO("========================");
    ROS_INFO("global_position:\n"); // Placeholder
    ROS_INFO("========================");
    ROS_INFO("home_position:\n"); // Placeholder
    ROS_INFO("========================");
    ROS_INFO("local_position:\n"); // Placeholder
    ROS_INFO("========================");
    ROS_INFO("mission_wp:\n"); // Placeholder
    ROS_INFO("========================");
    ROS_INFO("state:\n"); // Placeholder
    ROS_INFO("========================");
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "mavros_test_common");
    
//     // Instantiate the class and set up ROS connections
//     MavrosTestCommon test;
//     test.setUp();

//     // Example usage of class methods. This could be more elaborate based on actual needs.
//     // For instance, if you have specific tests or initialization procedures to run
//     // before entering the ROS event loop, you could do that here.

//     // Wait for all topics to be ready
//     int waitForTopicsTimeout = 30; // Seconds
//     test.waitForTopics(waitForTopicsTimeout);

//     // Example: wait for MAV_TYPE parameter
//     int waitForMavTypeTimeout = 10; // Seconds
//     test.waitForMavType(waitForMavTypeTimeout);

//     // Example: arm the vehicle
//     bool armVehicle = true;
//     int armTimeout = 5; // Seconds
//     test.setArm(armVehicle, armTimeout);

//     // Example: set vehicle mode
//     std::string mode = "AUTO.MISSION";
//     int modeTimeout = 5; // Seconds
//     test.setMode(mode, modeTimeout);

//     // Enter the ROS event loop
//     ros::spin();

//     return 0;
// }
