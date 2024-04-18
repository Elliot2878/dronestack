#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

class OffboardControl {
public:
    OffboardControl() {
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardControl::state_cb, this);
        // local_pos_sub = nh.subscribe("mavros/local_position/pose", 10, &OffboardControl::localPosition_cb, this);
        // global_pos_sub = nh.subscribe("/gazebo/model_states", 1000, &OffboardControl::globalPosition_cb, this);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    }

    void run() {
        ros::Rate rate(20.0);
        while(ros::ok() && !current_state.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        initialize_setpoint();
        main_loop();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub, local_pos_sub, global_pos_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client, set_mode_client;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped local_position;
    geometry_msgs::Pose global_position;
    

    // Callback functions
    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    void localPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        local_position = *msg;
    }

    // void globalPosition_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    //     // Find the index of the model; need to know the name of the drone, which is iris in this case
    //     auto it = std::find(msg->name.begin(), msg->name.end(), "iris_with_standoffs");
    //     if (it != msg->name.end()) {
    //         int index = std::distance(msg->name.begin(), it);

    //         // Assuming the drone is at 'index' in the arrays
    //         global_position = msg->pose[index];
    //         ROS_INFO("Drone Global Position - X: %f, Y: %f, Z: %f", global_position.position.x, global_position.position.y, global_position.position.z);
    //     } else {
    //         ROS_ERROR("Drone model not found in the simulation.");
    //     }
    // }



    void initialize_setpoint() {
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 4;

        for(int i = 100; ros::ok() && i > 0; --i) {
            local_pos_pub.publish(pose);
            ros::spinOnce();
            ros::Rate(20.0).sleep();
        }
    }

    void main_loop() {
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::SetMode land_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        land_mode.request.custom_mode = "AUTO.LAND";
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();
        
        // Waypoints include landing by descending to z=0 after the last position
        std::vector<std::vector<double>> positions = {{3, 3, 4}, {3, -3, 4}, {-3, -3, 4}, {3, -3, 4}, {0, 0, 4}, {0, 0, 0}};

        int count = 0;

        while(ros::ok()) {
            handle_mode_switch(offb_set_mode, last_request);
            handle_arm_command(arm_cmd, last_request);
            local_pos_pub.publish(pose);

            if(isAtPosition(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0.5)) {
                if(count >= positions.size()) {
                    ROS_INFO("End of waypoints navigations. Preparing to land.");
                    if(set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
                        ROS_INFO("Landing initiated.");
                        break; // Exit the loop to stop publishing waypoints and let the drone land.
                    }
                }
                ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position.", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                pose.pose.position.x = positions[count][0];
                pose.pose.position.y = positions[count][1];
                pose.pose.position.z = positions[count][2];
                count++;
            }

            ros::spinOnce();
            ros::Rate(20.0).sleep();
        }
    }

    void handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
                ROS_INFO("Offboard enabled");
                last_request = ros::Time::now();
            }
        }
    }

    void handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            }
        }
    }


    bool isAtPosition(double x, double y, double z, double offset) {
        ROS_DEBUG("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
        Eigen::Vector3d desired(x, y, z);
        Eigen::Vector3d pos(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
        return (desired - pos).norm() < offset;
    }

    // Function to convert global position to body position
    Eigen::Vector3d globalToBody(const Eigen::Vector3d& globalPos, const Eigen::Vector3d& dronePos, const Eigen::Quaterniond& droneOrient) {
        // Create a translation vector from drone position to global position
        Eigen::Vector3d translation = globalPos - dronePos;

        // Convert the quaternion orientation to a rotation matrix
        Eigen::Matrix3d rotationMatrix = droneOrient.toRotationMatrix();

        // Convert global coordinates to body coordinates
        Eigen::Vector3d bodyPos = rotationMatrix.inverse() * translation;

        return bodyPos;
    }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "");
    OffboardControl offboard_control;
    offboard_control.run();
    return 0;
}

