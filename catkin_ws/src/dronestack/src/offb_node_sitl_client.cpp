#include "ros/ros.h"
#include "dronestack/offb_node_sitl.h"
#include "dronestack/waypoint_nav.h"
#include <cstdlib>

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_nav_client");
    if (argc != 4)
    {
    ROS_INFO("usage: waypoint_nav_client X Y Z");
    return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient waypoint_client = nh.serviceClient<dronestack::waypoint_nav>("waypoint_nav");
    dronestack::waypoint_nav srv; // the type might need to be changed according to how you created the ROS service (not done yet).

    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    srv.request.c = atoll(argv[3]);


    if (waypoint_client.call(srv)) {
        ROS_INFO("Success: %d", srv.response.success);
    }
    else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;

}