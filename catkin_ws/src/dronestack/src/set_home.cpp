#include <ros/ros.h>
#include <mavros_msgs/CommandHome.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_home_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome srv;

    srv.request.current_gps = false;
    srv.request.latitude = 0.0;
    srv.request.longitude = 0.0;
    srv.request.altitude = 0.0;
    srv.request.yaw = 0.0;

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Home position set successfully.");
        } else {
            ROS_WARN("Failed to set home position.");
        }
    } else {
        ROS_ERROR("Failed to call service /mavros/cmd/set_home");
    }

    return 0;
}
