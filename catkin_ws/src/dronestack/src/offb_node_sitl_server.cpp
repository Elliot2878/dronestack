#include "dronestack/offb_node_sitl.h"
#include "dronestack/waypoint_nav.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>


// receive position and orientation from gazebo 
//LUO: Move these to the top, and make them global var, to make debugging easier
// double vicon_posi_x = 1.010000;
// double vicon_posi_y = 0.979938;
// double vicon_posi_z = 0.104262;

// double vicon_orient_r = 0.000912;
// double vicon_orient_p = 0.0;
// double vicon_orient_y = 0.0;



// double vicon_orient_r = 0;
// double vicon_orient_p = 0;
// double vicon_orient_y = 0;

// code for vicon. hardcode position and orientation in case callback has not executed immediately 
// (I recommand to read the vicon data first and hardcode the values since the origin is in the corner of the vicon room)
double vicon_posi_x = 0;
double vicon_posi_y = 0;
double vicon_posi_z = 0;

Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);

// Extract and store orientation as an Eigen quaternion
Eigen::Quaterniond q = Eigen::Quaterniond(0, 0, 0, 0).conjugate();



int count = 0;

bool data_stored = false;

// // define translation and rotation matrix
// Eigen::Vector3d translation;
// Eigen::Quaterniond q;

bool handle_waypoint_nav(dronestack::waypoint_nav::Request &req, 
                         dronestack::waypoint_nav::Response &res)
{
  OffboardControlSITL offboard_control;
  return offboard_control.control(req, res);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_nav_server");
    OffboardControlSITL offboard_control;
    // q = offboard_control.rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y).conjugate();
    
    // offboard_control.control();
    ros::NodeHandle nh; // Declare NodeHandle
    ros::ServiceServer waypoint_server = nh.advertiseService("waypoint_nav", handle_waypoint_nav);
    
    ROS_INFO("Waypoint navigation service ready");
    ros::spin(); // Spin to keep the node running
    
    return 0;
}


OffboardControlSITL::OffboardControlSITL() {
    // subscriber initialization
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControlSITL::state_cb, this);
    current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardControlSITL::currentPose_cb, this);
    vicon_pose_sub = nh.subscribe("vicon/x500_1/x500_1", 10, &OffboardControlSITL::viconPose_cb, this);
    // current_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, &OffboardControlSITL::currentVelocity_cb, this);

    // publisher initialization
    // pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // client service initialization (arming and mode)
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
}


// Callback functions
void OffboardControlSITL::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    // current_state stores the mode of the drone (eg, arm, disarm, etc)
    current_state = *msg;
}

void OffboardControlSITL::currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // local_position stores the current position and orientation of the drone
    current_pose = *msg;
}

void OffboardControlSITL::viconPose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // we only want to update position and orientation once from vicon
    if (!data_stored) {
        vicon_posi_x = msg->transform.translation.x;
        vicon_posi_y = msg->transform.translation.y;
        vicon_posi_z = msg->transform.translation.z;

        translation = Eigen::Vector3d(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);

        // Extract and store orientation as an Eigen quaternion
        q = Eigen::Quaterniond(
            msg->transform.rotation.w,  // w first in Eigen's constructor
            msg->transform.rotation.x,
            msg->transform.rotation.y,
            msg->transform.rotation.z
        );

        q = q.conjugate();
        
        data_stored = true;
    }
    
    vicon_pose = *msg;
}

// void OffboardControlSITL::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
//     current_velocity.x() = msg->twist.linear.x;
//     current_velocity.y() = msg->twist.linear.y;
//     current_velocity.z() = msg->twist.linear.z;

//     ROS_INFO("current velocity in Callback: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());
// }



bool OffboardControlSITL::control(dronestack::waypoint_nav::Request &req, dronestack::waypoint_nav::Response &res) { // type might need to be changed according to how you created the services
    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected) {
        // if not connected, keep ros running and wait for instruction
        ros::spinOnce();
        rate.sleep();
    }
    
    // initialze velocity setpoints
    vel_msg.twist.linear.x = 0;
    vel_msg.twist.linear.y = 0;
    vel_msg.twist.linear.z = 0;


    // initialize variables
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode land_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_mode.request.custom_mode = "AUTO.LAND";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    

    // Waypoints include landing by descending to z=0 after the last position
    // state 0 is leanding state, state 1 is take off state, and state 2 is moving state
    int state; 
    
    if(count == 0) {
        state = 1; // for the first waypoint, you need to take off first
    }
    else {
        state = 2; 
    }
    
    
    int num_of_task_to_run = 3;


    // // obtain translation and rotation matrix
    // const Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);
    // Eigen::Quaterniond q = rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y);
    // q = q.conjugate();

    // // set orientation
    // target.pose.orientation.x = 0;
    // target.pose.orientation.y = 0;
    // target.pose.orientation.z = 0;
    // target.pose.orientation.w = 1;

    while(ros::ok()) {
        // control loop


        // switch mode to offb_node (need to be inside the while loop to maintain offboard mode)
        handle_mode_switch(offb_set_mode, last_request);

        // arm the drone (need to be inside the while loop to maintain arming)
        handle_arm_command(arm_cmd, last_request);

        double d = 0.8; // this variable controls the speed of the drone

        if(state == 0) {
            // landing state
            ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. End of waypoints navigations. Preparing to land.", target_point[0], target_point[1], target_point[2]);
            if(set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
                ROS_INFO("Landing initiated.");
                break; // Exit the loop to stop publishing waypoints and let the drone land.
            }
        }
        else if(state == 1) {
            // take off state
            target_point = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 1);
            desired = transformPoint(translation, q, target_point);

        }
        else if(state == 2) {
            // waypoint navigation state
            target_point = Eigen::Vector3d(req.a, req.b, req.c);
            desired = transformPoint(translation, q, target_point);

        }

        // // if using pose controller
        // target.pose.position.x = desired[0];
        // target.pose.position.y = desired[1];
        // target.pose.position.z = desired[2];

        // if using gazebo and velocity controller
        vel_msg.twist.linear.x = (desired[0] - current_pose.pose.position.x) * d;
        vel_msg.twist.linear.y = (desired[1] - current_pose.pose.position.y) * d;
        vel_msg.twist.linear.z = (desired[2] - current_pose.pose.position.z) * d;


        // // if using vicon and velocity controller
        // vel_msg.twist.linear.x = (desired[0] - vicon_pose.transform.translation.x) * d;
        // vel_msg.twist.linear.y = (desired[1] - vicon_pose.transform.translation.y) * d;
        // vel_msg.twist.linear.z = (desired[2] - vicon_pose.transform.translation.z) * d;


        // // publish target setpoint to mavros
        // pos_pub.publish(target);
        vel_pub.publish(vel_msg);
        
        // check if the drone reaches the target position. If it is true, drone moves to next position
        if(isAtPosition(desired[0], desired[1], desired[2], 0.3, 0.5)) {
            if(count == num_of_task_to_run) {
                // if the state is already the last task to run, set the state to zero so that the drone can land
                state = 0;
            }
            else {
                ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position. State is:%.2d", target_point[0], target_point[1], target_point[2], state);
                if(state == 1) {
                    state = 2; // switch from take off state to waypoint navigation state, so the drone can start moving to the first waypoint
                }
                else {
                    count++; // The fact that state is not 1 means you have reached a waypoint, so you increment the count
                    break;
                }
            }
        }


        ros::spinOnce();
        ros::Rate(20.0).sleep();
    }

    res.success = true; 
    return true;
}

void OffboardControlSITL::handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
    // function to switch mode to "OFFBOARD"
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
        }
    }
}

void OffboardControlSITL::handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
    // change disarm to arm
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            last_request = ros::Time::now();
        }
    }
}

Eigen::Vector3d OffboardControlSITL::transformPoint(const Eigen::Vector3d &translation,
                               const Eigen::Quaterniond &rotation,
                               const Eigen::Vector3d &point) {

    // Create a transformation matrix from the translation and rotation
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(translation);
    transform.rotate(rotation);
    // Transform the point using the transformation matrix
    Eigen::Vector3d transformed_point = transform * point;

    return transformed_point;
}

Eigen::Quaterniond OffboardControlSITL::rpyToQuaternion(double roll, double pitch, double yaw) {
    // Eigen uses the following convention: roll about X, pitch about Y, yaw about Z
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    // Combine the angles in reverse order
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    // The quaternion is now created from the RPY angles
    return q;
}

bool OffboardControlSITL::isAtPosition(double x, double y, double z, double xy_offset, double z_offset) {
    // a function to check if the difference between current position and target position is within a predefined offset
    // ROS_INFO("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    Eigen::Vector2d desired_posi_xy(x, y);
    
    // code for vicon
    Eigen::Vector2d current_posi_xy(vicon_pose.transform.translation.x, vicon_pose.transform.translation.y);
    return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - vicon_pose.transform.translation.z) < z_offset);

    // // code for gazebo
    // Eigen::Vector2d current_posi_xy(current_pose.pose.position.x, current_pose.pose.position.y);
    // return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - current_pose.pose.position.z) < z_offset);
}





















// // version with state machine in callback function
// #include "dronestack/offb_node_sitl.h"
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <gazebo_msgs/ModelStates.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <Eigen/Dense>

// OffboardControlSITL::OffboardControlSITL() {
//     // subscriber initialization
//     state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControlSITL::state_cb, this);
//     current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardControlSITL::currentPosition_cb, this);
//     // current_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, &OffboardControlSITL::localVelocity_cb, this);

//     // publisher initialization
//     // pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//     vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

//     // client service initialization (arming and mode)
//     arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//     set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
// }

// // constexpr double MAX_VELOCITY = 0.0001; // meters per second
// // Eigen::Vector3d current_velocity; // to store current velocity


// int main(int argc, char **argv) {
//     ros::init(argc, argv, "offb_node");
//     OffboardControlSITL offboard_control;
//     offboard_control.control();

//     ros::spin();
//     // ros::Rate(50.0).sleep();

//     return 0;
// }


// // Callback functions
// void OffboardControlSITL::state_cb(const mavros_msgs::State::ConstPtr& msg) {
//     // current_state stores the mode of the drone (eg, arm, disarm, etc)
//     current_state = *msg;
// }

// void OffboardControlSITL::currentPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     // local_position stores the current position and orientation of the drone
//     current_pose = *msg;

//     ROS_INFO("reached localPosition cb");

//     // initialize variables
//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";
//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;
//     ros::Time last_request = ros::Time::now();

//     // switch mode to offb_node (need to be inside the while loop to maintain offboard mode)
//     handle_mode_switch(offb_set_mode, last_request);

//     // arm the drone (need to be inside the while loop to maintain arming)
//     handle_arm_command(arm_cmd, last_request);

//     // Waypoints include landing by descending to z=0 after the last position
//     tasks = {1, 2, 3, 4, 5, 6, 7};
//     state = 1;
//     task_indicator = 0;
//     double d = 0.4;

//     // receive vicon position and orientation from vicon
//     double vicon_posi_x = 1.010000;
//     double vicon_posi_y = 0.979938;
//     double vicon_posi_z = 0.104262;
//     const Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);

//     double vicon_orient_r = 0.000912;
//     double vicon_orient_p = 0.0;
//     double vicon_orient_y = 0.0;
//     Eigen::Quaterniond q = rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y);
//     q = q.conjugate();


//     if(state == 1) {
//         const Eigen::Vector3d target_point(vicon_posi_x, vicon_posi_y, 4);
//         desired = transformPoint(translation, q, target_point);

//     }
//     else if(state == 2) {
//         const Eigen::Vector3d target_point(3, 3, 4);
//         desired = transformPoint(translation, q, target_point);

//     }
//     else if(state == 3) {
//         const Eigen::Vector3d target_point(3, -3, 4);
//         desired = transformPoint(translation, q, target_point);
        
//     }
//     else if(state == 4) {
//         const Eigen::Vector3d target_point(-3, -3, 4);
//         desired = transformPoint(translation, q, target_point);

//     }
//     else if(state == 6) {
//         const Eigen::Vector3d target_point(3, 3, 4);
//         desired = transformPoint(translation, q, target_point);

//     }
//     else if(state == 7) {
//         const Eigen::Vector3d target_point(0, 0, 4);
//         desired = transformPoint(translation, q, target_point);

//     }

//     // target.pose.position.x = desired[0];
//     // target.pose.position.y = desired[1];
//     // target.pose.position.z = desired[2];

//     vel_msg.twist.linear.x = (desired[0] - current_pose.pose.position.x) * d;
//     vel_msg.twist.linear.y = (desired[1] - current_pose.pose.position.y) * d;
//     vel_msg.twist.linear.z = (desired[2] - current_pose.pose.position.z) * d;

//     control();

// }

// // void OffboardControlSITL::localVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
// //     current_velocity.x() = msg->twist.linear.x;
// //     current_velocity.y() = msg->twist.linear.y;
// //     current_velocity.z() = msg->twist.linear.z;

// //     ROS_INFO("current velocity in Callback: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());
// // }


// void OffboardControlSITL::control() {

//     ros::Rate rate(20.0);
//     while(ros::ok() && !current_state.connected) {
//         // if not connected, keep ros running and wait for instruction
//         ros::spinOnce();
//         rate.sleep();
//     }
    
//     // initialize velocity setpoints
//     vel_msg.twist.linear.x = 0;
//     vel_msg.twist.linear.y = 0;
//     vel_msg.twist.linear.z = 0;



//     mavros_msgs::SetMode land_mode;
//     land_mode.request.custom_mode = "AUTO.LAND";

//     // // publish target setpoint to mavros
//     // pos_pub.publish(target);

//     vel_pub.publish(vel_msg);
    
//     // check if the drone reaches the target position. If it is true, drone moves to next position
//     if(isAtPosition(desired[0], desired[1], desired[2], 0.3, 0.5)) {
//         if(task_indicator >= tasks.size()) {
//             ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. End of waypoints navigations. Preparing to land.", desired[0], desired[1], desired[2]);
//             if(set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
//                 ROS_INFO("Landing initiated.");
//                 return; // Exit the loop to stop publishing waypoints and let the drone land.
//             }
//         }
//         ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position. State is:%.2d", desired[0], desired[1], desired[2], state);
//         state = tasks[task_indicator];
//         task_indicator++;
//     }

// }

// void OffboardControlSITL::handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
//     // function to switch mode to "OFFBOARD"
//     if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
//         if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
//             ROS_INFO("Offboard enabled");
//             last_request = ros::Time::now();
//         }
//     }
// }

// void OffboardControlSITL::handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
//     // change disarm to arm
//     if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
//         if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
//             ROS_INFO("Vehicle armed");
//             last_request = ros::Time::now();
//         }
//     }
// }

// // void OffboardControlSITL::publish_clamped_velocity(ros::Publisher& vel_pub) {
// //     ROS_INFO("current velocity in my function: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());

// //     Eigen::Vector3d adjusted_velocity = current_velocity;

// //     // Check if current velocity exceeds the maximum allowed velocity
// //     if (current_velocity.norm() > MAX_VELOCITY) {
// //         // Adjust the desired velocity to reduce it
// //         adjusted_velocity = MAX_VELOCITY * current_velocity.normalized();
// //     }

// //     // Prepare and publish the velocity message
// //     geometry_msgs::TwistStamped vel_msg;
// //     vel_msg.header.stamp = ros::Time::now();
// //     vel_msg.twist.linear.x = adjusted_velocity.x();
// //     vel_msg.twist.linear.y = adjusted_velocity.y();
// //     vel_msg.twist.linear.z = adjusted_velocity.z();

// //     ROS_INFO("clamped velocity: x:%.2f, y:%.2f, z:%.2f", vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z);

// //     vel_pub.publish(vel_msg);
// // }


// Eigen::Vector3d OffboardControlSITL::transformPoint(const Eigen::Vector3d &translation,
//                                const Eigen::Quaterniond &rotation,
//                                const Eigen::Vector3d &point) {

//     // Create a transformation matrix from the translation and rotation
//     Eigen::Affine3d transform = Eigen::Affine3d::Identity();
//     transform.translate(translation);
//     transform.rotate(rotation);
//     // Transform the point using the transformation matrix
//     Eigen::Vector3d transformed_point = transform * point;

//     return transformed_point;
// }

// Eigen::Quaterniond OffboardControlSITL::rpyToQuaternion(double roll, double pitch, double yaw) {
//     // Eigen uses the following convention: roll about X, pitch about Y, yaw about Z
//     Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//     Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

//     // Combine the angles in reverse order
//     Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

//     // The quaternion is now created from the RPY angles
//     return q;
// }

// bool OffboardControlSITL::isAtPosition(double x, double y, double z, double xy_offset, double z_offset) {
//     // a function to check if the difference between current position and target position is within a predefined offset
//     // ROS_INFO("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
//     Eigen::Vector2d desired_posi_xy(x, y);
//     Eigen::Vector2d current_posi_xy(current_pose.pose.position.x, current_pose.pose.position.y);
    
//     return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - current_pose.pose.position.z) < z_offset);
// }




