#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "interpolate_points/catmull_rom_spline.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "interpolate_points");
    ros::NodeHandle nh;

    // Create publisher for the path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("spline_path", 1);

    // Define control points
    std::vector<interpolate_points::Point2D> control_points = {
        {0.0, 0.0},  // P0
        {1.0, 2.0},  // P1
        {4.0, 2.0},  // P2
        {5.0, 0.0}   // P3
    };

    // Create Catmull-Rom spline with centripetal parameter alpha = 0.5
    interpolate_points::CatmullRomSpline spline(control_points, 0.5);

    // Generate the path
    nav_msgs::Path path = spline.generatePath(0.1);  // 0.1 meter resolution

    // Verify start and end points
    if (!path.poses.empty()) {
        const auto& start_pose = path.poses.front().pose.position;
        const auto& end_pose = path.poses.back().pose.position;

        ROS_INFO("Start Point: x=%.2f, y=%.2f", start_pose.x, start_pose.y);
        ROS_INFO("End Point: x=%.2f, y=%.2f", end_pose.x, end_pose.y);
    }

    // Publish the path
    ros::Rate rate(1.0);  // 1 Hz
    while (ros::ok()) {
        path_pub.publish(path);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}