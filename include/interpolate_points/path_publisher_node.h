#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <string>
#include <vector>

struct ControlPoint2D
{
    double x, y;
};

class PathPublisherNode
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher path_pub_;
        ros::Timer timer_;
        std::string path_topic_;
        std::vector<ControlPoint2D> control_points_;
        nav_msgs::Path path_msg_;

    public:
        PathPublisherNode();
        ~PathPublisherNode();

        void publishPath();
        void loadControlPoints();
        bool createPathMessage();
};