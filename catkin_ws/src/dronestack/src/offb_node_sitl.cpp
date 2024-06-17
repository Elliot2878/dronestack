#include "dronestack/offb_node_sitl.h"
#include <ros/ros.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>


// // hardcode initial position and orientation from gazebo (gazebo)
// double vicon_posi_x = 1.010000;
// double vicon_posi_y = 0.979938;
// double vicon_posi_z = 0.104262;
// double vicon_orient_r = 0.000912;
// double vicon_orient_p = 0.0;
// double vicon_orient_y = 0.0;

// define translation and rotation matrix
// Eigen::Vector3d translation; // (deleted)
// Eigen::Quaterniond q; // (gazebo)

// hardcode position and orientation in case callback has not executed immediately (vicon)
// (I recommand to read the vicon data first and hardcode the values since the origin is in the corner of the vicon room)
double vicon_posi_x = 2.40;
double vicon_posi_y = 3.55;
double vicon_posi_z = 0.4180;

// (vicon) and (gazebo)
Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);

// Extract and store orientation as an Eigen quaternion (vicon)
Eigen::Quaterniond q = Eigen::Quaterniond(0.0022, -0.00278, 0.0068, 0.99997).conjugate();


bool data_stored = false;


int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    OffboardControlSITL offboard_control;

    // // (gazebo)
    // q = offboard_control.rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y).conjugate();
    
    offboard_control.initialization();
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
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // client service initialization (arming and mode)
    set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // publish position and velocity to vision pose for replacing GPS
    px4_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1); 
    // px4_vision_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/vision_speed/speed_twist", 1);

    ros::Rate rate(20.0);
}

// Callback functions
void OffboardControlSITL::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    // current_state stores the mode of the drone (eg, arm, disarm, etc)
    current_state = *msg;
}

void OffboardControlSITL::currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // local_position stores the current position and orientation of the drone
    current_pose = *msg;

    // ROS_INFO("Current Gazebo Position: x:%.2f, y:%.2f, z:%.2f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
}

void OffboardControlSITL::viconPose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // we only store translation and orientation quaternion once
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

    // ROS_INFO("Check stored take-off vicon_pose: x:%.2f, y:%.2f, z:%.2f", vicon_posi_x, vicon_posi_y, vicon_posi_z);
    
    vicon_pose = *msg;

    // ROS_INFO("Check received vicon_pose_1: x:%.2f, y:%.2f, z:%.2f", vicon_pose.transform.translation.x, vicon_pose.transform.translation.y, vicon_pose.transform.translation.z);

    // Create and publish pose message to vision
    vision_pose_msg.header = msg->header;
    vision_pose_msg.pose.position.x = msg->transform.translation.x;
    vision_pose_msg.pose.position.y = msg->transform.translation.y;
    vision_pose_msg.pose.position.z = msg->transform.translation.z;
    vision_pose_msg.pose.orientation = msg->transform.rotation;

    // //ROS_INFO("Received Vicon position x:%.2f, y:%.2f, z:%.2f", vision_pose_msg.pose.position.x, vision_pose_msg.pose.position.y, vision_pose_msg.pose.position.z);
    // px4_vision_pose_pub.publish(vision_pose_msg);
    
}

// void OffboardControlSITL::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
//     current_velocity.x() = msg->twist.linear.x;
//     current_velocity.y() = msg->twist.linear.y;
//     current_velocity.z() = msg->twist.linear.z;

//     ROS_INFO("current velocity in Callback: x:%.2f, y:%.2f, z:%.2f", current_velocity.x(), current_velocity.y(), current_velocity.z());
// }


void OffboardControlSITL::initialization() {

    // ROS_INFO("set home");
    // mavros_msgs::CommandHome srv;

    // srv.request.current_gps = false;
    // srv.request.latitude = 0.0;
    // srv.request.longitude = 0.0;
    // srv.request.altitude = 0.0;
    // srv.request.yaw = 0.0;

    // if (set_home_client.call(srv)) {
    //     if (srv.response.success) {
    //         ROS_INFO("Home position set successfully.");
    //     } else {
    //         ROS_WARN("Failed to set home position.");
    //     }
    // } else {
    //     ROS_ERROR("Failed to call service /mavros/cmd/set_home");
    // }




    // sending a few setpoints before switching to offboard mode is necessary 
    // to ensure that the flight controller receives a stable and consistent stream of setpoints before you switch to offboard mode and arm the drone. 
    // This is an important step for safety and reliability. 

    ROS_INFO("initialize setpoint");


    // // initialze velocity setpoints
    // geometry_msgs::TwistStamped vel_msg;
    // vel_msg.twist.linear.x = 0;
    // vel_msg.twist.linear.y = 0;
    // vel_msg.twist.linear.z = 0;
    // vel_msg.twist.angular.x = 0;
    // vel_msg.twist.angular.y = 0;
    // vel_msg.twist.angular.z = 0;



    // for(int i = 100; ros::ok() && i > 0; --i) {
    //     // // publish position and velocity to vision pose for replacing GPS
    //     // px4_vision_pose_pub.publish(pose_msg);
    //     // px4_vision_vel_pub.publish(twist_msg);



    //     //publish velocity
    //     vel_pub.publish(vel_msg); // velocity controller



    //     ros::spinOnce();
    //     ros::Rate(20.0).sleep();
    // }

    // // initialize position setpoint
    // target.pose.position.x = 0;
    // target.pose.position.y = 0;
    // target.pose.position.z = 0.75;

    // for(int i = 100; ros::ok() && i > 0; --i) {

    //     //publish hover position
    //     pos_pub.publish(target);



    //     ros::spinOnce();
    //     ros::Rate(20.0).sleep();
    // }

}


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
    // mavros_msgs::CommandTOL land_cmd;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    


    int num_of_state_to_run = 2;

    int state = 1; // should not modify


    // // obtain translation and rotation matrix (have already written it on the top)
    // const Eigen::Vector3d translation(-vicon_posi_x, -vicon_posi_y, -vicon_posi_z);
    // Eigen::Quaterniond q = rpyToQuaternion(vicon_orient_r, vicon_orient_p, vicon_orient_y);
    // q = q.conjugate();

    // // set orientation (position controller)
    // target_pose.pose.orientation.x = 0;
    // target_pose.pose.orientation.y = 0;
    // target_pose.pose.orientation.z = 0;
    // target_pose.pose.orientation.w = 1;

    while(ros::ok()) {
        // control loop

        // publish position to vision pose for replacing GPS (vicon) Must write this before switching mode and arming !
        px4_vision_pose_pub.publish(vision_pose_msg);

        



        // variable d can control the velocity of the drone
        double v = 1;

        // state 0 is landing state, state 1 is take off state, others are moving state
        if(state == 0) {
            ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. End of waypoints navigations. Preparing to land.", target[0], target[1], target[2]);
            // if(set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
            //     ROS_INFO("Landing initiated.");
            //     break; // Exit the loop to stop publishing waypoints and let the drone land.
            // }
        }
        else if(state == 1) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 0.95);
            target_trans = transformPoint(translation, q, target);

        }
        else if(state == 2) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y + 0.7, 0.95);
            target_trans = transformPoint(translation, q, target);

        }
        else if(state == 3) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 1);
            target_trans = transformPoint(translation, q, target);
            
        }
        else if(state == 4) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 1);
            target_trans = transformPoint(translation, q, target);        
            
        }
        else if(state == 5) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 1);
            target_trans = transformPoint(translation, q, target);

        }
        else if(state == 6) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 1);
            target_trans = transformPoint(translation, q, target);

        }
        else if(state == 7) {
            target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, 1);  //LUO: landing should not be always at (0,0)
            target_trans = transformPoint(translation, q, target);

        }

        
        // // if using pose controller
        // target_pose.pose.position.x = target[0];
        // target_pose.pose.position.y = target[1];
        // target_pose.pose.position.z = target[2];

        
        // // if using gazebo and velocity controller (gazebo)
        // vel_msg.twist.linear.x = (target_trans[0] - current_pose.pose.position.x) * v;
        // vel_msg.twist.linear.y = (target_trans[1] - current_pose.pose.position.y) * v;
        // vel_msg.twist.linear.z = (target_trans[2] - current_pose.pose.position.z) * v;


        // if using vicon and velocity controller (vicon)
        vel_msg.twist.linear.x = (target[0] - vicon_pose.transform.translation.x) * v;
        vel_msg.twist.linear.y = (target[1] - vicon_pose.transform.translation.y) * v;
        vel_msg.twist.linear.z = (target[2] - vicon_pose.transform.translation.z) * v;

        ROS_INFO("Check received vicon_pose_2: x:%.2f, y:%.2f, z:%.2f", vicon_pose.transform.translation.x, vicon_pose.transform.translation.y, vicon_pose.transform.translation.z);
        // ROS_INFO("Check point before trans: x:%.2f, y:%.2f, z:%.2f", target[0], target[1], target[2]);
        // ROS_INFO("Check point after trans: x:%.2f, y:%.2f, z:%.2f", target_trans[0], target_trans[1], target_trans[2]);
        // ROS_INFO("Check vel_msg: x:%.2f, y:%.2f, z:%.2f", vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z);

        ROS_INFO("Check error: x:%.2f, y:%.2f, z:%.2f", (target[0] - vicon_pose.transform.translation.x), (target[1] - vicon_pose.transform.translation.y), (target[2] - vicon_pose.transform.translation.z));
        
        // // // publish target setpoint to mavros
        // pos_pub.publish(target_pose); // pose controller
        

        // vel_msg.twist.linear.x = 0;
        // vel_msg.twist.linear.y = 0;

        vel_msg = limitVelocity(vel_msg, 0.5);


        float vicon_yaw = std::get<2>(quaternionToRpy(vicon_pose.transform.rotation.x, vicon_pose.transform.rotation.y, vicon_pose.transform.rotation.z, vicon_pose.transform.rotation.w));
        
        // vel_msg.twist.angular.y = (0 - vicon_yaw) * 0.1;
        // vel_msg.twist.angular.z = (0 - vicon_yaw) * 0.1;
        


        // ROS_INFO("Check velocity after clamping: x:%.2f, y:%.2f, z:%.2f", vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z);
        vel_pub.publish(vel_msg); // velocity controller
        
        // switch mode to offb_node (need to be inside the while loop to maintain offboard mode)
        handle_mode_switch(offb_set_mode, last_request);
        
        // arm the drone (need to be inside the while loop to maintain arming)
        handle_arm_command(arm_cmd, last_request);

        
        // check if the drone reaches the target position. If it is true, drone moves to next position
        // use target for velocity controll in vicon. use target_trans otherwise
        if(isAtPosition(target[0], target[1], target[2], 0.2, 0.1)) {
            if(state >= num_of_state_to_run) {
                // if the state is already the last task to run, set the state to zero so that the drone can land
                // it is safer to write ">=" than "=="
                state = 0;
            }
            else {
                ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position. State is:%.2d", target[0], target[1], target[2], state);
                state++;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void OffboardControlSITL::handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
    // function to switch mode to "OFFBOARD"
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("OFFBOARD enabled");
            last_request = ros::Time::now();
        } else {
            ROS_WARN("Failed to set OFFBOARD mode");
        }
    } 
    // else {
    //     ROS_INFO("Current mode: %s", current_state.mode.c_str());
    // }
}

void OffboardControlSITL::handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            } else {
                ROS_WARN("Failed to arm vehicle: response success = false");
            }
        } else {
            ROS_WARN("Failed to arm vehicle: service call failed");
        }
    } 
    // else {
    //     ROS_INFO("Current armed state: %s", current_state.armed ? "ARMED" : "DISARMED");
    // }
}


geometry_msgs::TwistStamped OffboardControlSITL::limitVelocity(geometry_msgs::TwistStamped vel_msg, float max_velocity) {
    // The default max_velocity is 0.5 m/s. You can see that in the header file
    
    float vel_x = vel_msg.twist.linear.x;
    float vel_y = vel_msg.twist.linear.y;
    float vel_z = vel_msg.twist.linear.z;

    float magnitude = std::sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);

    // If the magnitude exceeds the maximum allowed velocity, scale it down
    if (magnitude > max_velocity) {
        float scale = max_velocity / magnitude;
        vel_x *= scale;
        vel_y *= scale;
        vel_z *= scale;
    }

    // Assign the limited velocity components to the message
    vel_msg.twist.linear.x = vel_x;
    vel_msg.twist.linear.y = vel_y;
    vel_msg.twist.linear.z = vel_z;

    return vel_msg;

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

std::tuple<float, float, float> OffboardControlSITL::quaternionToRpy(float x, float y, float z, float w) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    float pitch;
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return std::make_tuple(roll, pitch, yaw);
}

bool OffboardControlSITL::isAtPosition(double x, double y, double z, double xy_offset, double z_offset) {
    // a function to check if the difference between current position and target position is within a predefined offset
    // ROS_INFO("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    Eigen::Vector2d desired_posi_xy(x, y);

    // (vicon)     
    Eigen::Vector2d current_posi_xy(vicon_pose.transform.translation.x, vicon_pose.transform.translation.y);
    return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - vicon_pose.transform.translation.z) < z_offset);

    // // (gazebo)
    // Eigen::Vector2d current_posi_xy(current_pose.pose.position.x, current_pose.pose.position.y);
    // return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - current_pose.pose.position.z) < z_offset);
}















