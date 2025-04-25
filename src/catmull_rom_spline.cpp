#include "interpolate_points/catmull_rom_spline.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

namespace interpolate_points {

CatmullRomSpline::CatmullRomSpline(const std::vector<Point2D>& control_points, double alpha)
    : control_points_(control_points), alpha_(alpha) {
    if (control_points.size() < 4) {
        throw std::invalid_argument("At least 4 control points are required");
    }
}

double CatmullRomSpline::calculateParameter(const Point2D& p0, const Point2D& p1) const {
    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;
    return std::pow(dx * dx + dy * dy, alpha_ / 2.0);
}

Point2D CatmullRomSpline::calculatePoint(double t, const Point2D& p0, const Point2D& p1,
                                       const Point2D& p2, const Point2D& p3) const {
    // Calculate the centripetal parameters
    double t0 = 0.0;
    double t1 = calculateParameter(p0, p1);
    double t2 = calculateParameter(p1, p2) + t1;
    double t3 = calculateParameter(p2, p3) + t2;

    // Normalize t to [t1, t2]
    t = t1 + t * (t2 - t1);

    // Calculate the basis functions
    Point2D A1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
    Point2D A2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
    Point2D A3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;

    Point2D B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2;
    Point2D B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3;

    return (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2;
}

nav_msgs::Path CatmullRomSpline::generatePath(double resolution) const {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    // Add the first control point as the start of the path
    geometry_msgs::PoseStamped start_pose;
    start_pose.header = path.header;
    start_pose.pose.position.x = control_points_.front().x;
    start_pose.pose.position.y = control_points_.front().y;
    start_pose.pose.orientation.w = 1.0;  // No rotation
    path.poses.push_back(start_pose);

    // Generate points for each segment
    const int POINTS_PER_SEGMENT = 100;  // High resolution for accurate length calculation
    for (size_t i = 0; i < control_points_.size() - 3; ++i) {
        std::vector<Point2D> segment_points;
        double segment_length = 0.0;

        // First pass: generate high-resolution points and calculate actual segment length
        Point2D prev_point;
        for (int j = 0; j <= POINTS_PER_SEGMENT; ++j) {
            double t = static_cast<double>(j) / POINTS_PER_SEGMENT;
            Point2D point = calculatePoint(t, control_points_[i], control_points_[i + 1],
                                           control_points_[i + 2], control_points_[i + 3]);

            if (j > 0) {
                double dx = point.x - prev_point.x;
                double dy = point.y - prev_point.y;
                segment_length += std::sqrt(dx * dx + dy * dy);
            }
            prev_point = point;
            segment_points.push_back(point);
        }

        // Second pass: sample points at desired resolution
        int num_points = static_cast<int>(segment_length / resolution) + 1;
        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / (num_points - 1);
            int idx = static_cast<int>(t * POINTS_PER_SEGMENT);
            idx = std::min(idx, POINTS_PER_SEGMENT - 1);

            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = segment_points[idx].x;
            pose.pose.position.y = segment_points[idx].y;
            pose.pose.orientation.w = 1.0;  // No rotation

            path.poses.push_back(pose);
        }
    }

    // Add the last control point as the end of the path
    geometry_msgs::PoseStamped end_pose;
    end_pose.header = path.header;
    end_pose.pose.position.x = control_points_.back().x;
    end_pose.pose.position.y = control_points_.back().y;
    end_pose.pose.orientation.w = 1.0;  // No rotation
    path.poses.push_back(end_pose);

    return path;
}

}
