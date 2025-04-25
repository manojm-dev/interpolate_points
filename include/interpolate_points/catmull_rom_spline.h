#ifndef trajectory_generator_CATMULL_ROM_SPLINE_H
#define trajectory_generator_CATMULL_ROM_SPLINE_H

#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

namespace trajectory_generator {

/**
 * @brief 2D point structure with operator overloading
 */
struct Point2D {
    double x;
    double y;

    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x, double y) : x(x), y(y) {}

    // Vector addition
    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }

    // Scalar multiplication (Point2D * double)
    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }

    // Friend function for scalar multiplication (double * Point2D)
    friend Point2D operator*(double scalar, const Point2D& point) {
        return point * scalar;
    }
};

/**
 * @brief Class implementing centripetal Catmull-Rom spline interpolation
 */
class CatmullRomSpline {
public:
    /**
     * @brief Constructor
     * @param control_points Vector of control points
     * @param alpha Parameter controlling the centripetal behavior (0.5 is recommended)
     */
    CatmullRomSpline(const std::vector<Point2D>& control_points, double alpha = 0.5);

    /**
     * @brief Generate a path message from the spline
     * @param resolution Distance between consecutive points in meters
     * @return nav_msgs::Path message containing the interpolated path
     */
    nav_msgs::Path generatePath(double resolution = 0.1) const;

private:
    /**
     * @brief Calculate the centripetal parameter for a given segment
     * @param p0 First point
     * @param p1 Second point
     * @return Parameter value
     */
    double calculateParameter(const Point2D& p0, const Point2D& p1) const;

    /**
     * @brief Calculate a point on the spline
     * @param t Parameter value [0,1]
     * @param p0 First control point
     * @param p1 Second control point
     * @param p2 Third control point
     * @param p3 Fourth control point
     * @return Interpolated point
     */
    Point2D calculatePoint(double t, const Point2D& p0, const Point2D& p1,
                          const Point2D& p2, const Point2D& p3) const;

    std::vector<Point2D> control_points_;
    double alpha_;
};

} // namespace trajectory_generator

#endif // trajectory_generator_CATMULL_ROM_SPLINE_H
