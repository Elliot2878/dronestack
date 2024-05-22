#include "dronestack/offb_node_sitl.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>


int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    OffboardControlSITL offboard_control;
    offboard_control.control();
    
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
    vicon_pose = *msg;
}

// void OffboardControlSITL::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
//     current_velocity.x() = msg->twist.linear.x;
//     current_velocity.y() = msg->twist.linear.y;
//     current_velocity.z() = msg->twist.linear.z;

//     ROS_INFO("current velocity in Callback: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());
// }



void OffboardControlSITL::control() {
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
    //LUO: why not just have a landing func that can land at whatever places and whatever heights?
    std::vector<int> tasks = {1, 2, 3, 4, 5, 6, 7};

    int state = 1;
    int task_indicator = 0;

    // receive vicon position and orientation from vicon
    //LUO: Move these to the top, and make them global var, to make debugging easier
    double vicon_posi_x = 1.010000;
    double vicon_posi_y = 0.979938;
    double vicon_posi_z = 0.104262;
    const Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);

    double vicon_orient_r = 0.000912;
    double vicon_orient_p = 0.0;
    double vicon_orient_y = 0.0;
    Eigen::Quaterniond q = rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y);
    q = q.conjugate();

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
        //LUO: so there is no way to stop arming, say landing, in this loop?
        handle_arm_command(arm_cmd, last_request);

        double d = 0.8;

        if(state == 1) {
            target_point = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 4);
            desired = transformPoint(translation, q, target_point);

        }
        else if(state == 2) {
            target_point = Eigen::Vector3d(3, 3, 4);
            desired = transformPoint(translation, q, target_point);

        }
        else if(state == 3) {
            target_point = Eigen::Vector3d(3, -3, 4);
            desired = transformPoint(translation, q, target_point);
            
        }
        else if(state == 4) {
            target_point = Eigen::Vector3d(-3, -3, 4);
            desired = transformPoint(translation, q, target_point);        
            
        }
        else if(state == 5) {
            target_point = Eigen::Vector3d(-3, 3, 4);
            desired = transformPoint(translation, q, target_point);

        }
        else if(state == 6) {
            target_point = Eigen::Vector3d(3, 3, 4);
            desired = transformPoint(translation, q, target_point);

        }
        else if(state == 7) {
            target_point = Eigen::Vector3d(0, 0, 4);  //LUO: landing should not be always at (0,0)
            desired = transformPoint(translation, q, target_point);

        }

        // // if using pose controller
        // target.pose.position.x = desired[0];
        // target.pose.position.y = desired[1];
        // target.pose.position.z = desired[2];

        // // if using gazebo and velocity controller
        // vel_msg.twist.linear.x = (desired[0] - current_pose.pose.position.x) * d;
        // vel_msg.twist.linear.y = (desired[1] - current_pose.pose.position.y) * d;
        // vel_msg.twist.linear.z = (desired[2] - current_pose.pose.position.z) * d;


        // if using vicon and velocity controller
        vel_msg.twist.linear.x = (desired[0] - vicon_pose.transform.translation.x) * d;
        vel_msg.twist.linear.y = (desired[1] - vicon_pose.transform.translation.y) * d;
        vel_msg.twist.linear.z = (desired[2] - vicon_pose.transform.translation.z) * d;


        // // publish target setpoint to mavros
        // pos_pub.publish(target);
        vel_pub.publish(vel_msg);
        
        // check if the drone reaches the target position. If it is true, drone moves to next position
        if(isAtPosition(desired[0], desired[1], desired[2], 0.3, 0.5)) {
            if(task_indicator >= tasks.size()) {
                ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. End of waypoints navigations. Preparing to land.", target_point[0], target_point[1], target_point[2]);
                if(set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
                    ROS_INFO("Landing initiated.");
                    break; // Exit the loop to stop publishing waypoints and let the drone land.
                }
            }
            ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position. State is:%.2d", target_point[0], target_point[1], target_point[2], state);
            state = tasks[task_indicator];
            task_indicator++;
        }

        ros::spinOnce();
        ros::Rate(20.0).sleep();
    }
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
    Eigen::Vector2d current_posi_xy(vicon_pose.transform.translation.x, vicon_pose.transform.translation.y);
    
    return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - vicon_pose.transform.translation.z) < z_offset);


    // Eigen::Vector2d current_posi_xy(current_pose.pose.position.x, current_pose.pose.position.y);
    // return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - current_pose.pose.position.z) < z_offset);
}













