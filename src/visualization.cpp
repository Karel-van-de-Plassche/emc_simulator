#include "visualization.h"
#include "world.h"
#include "robo/sensing.h"
#include "robo/doors.h"
#include <tf2/LinearMath/Vector3.h>
#include <vector>

#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

double resolution = 0.01;
cv::Point2d canvas_center;

namespace visualization
{

cv::Point2d worldToCanvas(const geo::Vector3& p)
{
    return cv::Point2d(-p.y / resolution, -p.x / resolution) + canvas_center;
}

// ----------------------------------------------------------------------------------------------------

void visualize(const World& world, Id robot_id, emc::LaserData scan)
{
    const Object& robot = world.object(robot_id);

    cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(100, 100, 100));
    canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols / 2);

    // Draw robot
    cv::Scalar robot_color(0, 0, 255);

    std::vector<geo::Vector3> robot_points;
    robot_points.push_back(geo::Vector3( 0.1,  -0.2, 0));
    robot_points.push_back(geo::Vector3( 0.1,  -0.1, 0));
    robot_points.push_back(geo::Vector3( 0.05, -0.1, 0));
    robot_points.push_back(geo::Vector3( 0.05,  0.1, 0));
    robot_points.push_back(geo::Vector3( 0.1,   0.1, 0));
    robot_points.push_back(geo::Vector3( 0.1,   0.2, 0));
    robot_points.push_back(geo::Vector3(-0.1,   0.2, 0));
    robot_points.push_back(geo::Vector3(-0.1,  -0.2, 0));

    for(unsigned int i = 0; i < robot_points.size(); ++i)
    {
        unsigned int j = (i + 1) % robot_points.size();
        cv::Point2d p1 = worldToCanvas(robot_points[i]);
        cv::Point2d p2 = worldToCanvas(robot_points[j]);
        cv::line(canvas, p1, p2, robot_color, 2);
    }

    for(std::vector<Object>::const_iterator it = world.objects().begin(); it != world.objects().end(); ++it)
    {
        const Object& obj = *it;
        if (!obj.shape)
            continue;

        const std::vector<geo::Vector3>& vertices = obj.shape->getMesh().getPoints();
        const std::vector<geo::TriangleI>& triangles = obj.shape->getMesh().getTriangleIs();

        cv::Scalar line_color(obj.color.x * 255, obj.color.y * 255, obj.color.z * 255);

        geo::Transform t =  robot.pose.inverse() * obj.pose;

        for(std::vector<geo::TriangleI>::const_iterator it2 = triangles.begin(); it2 != triangles.end(); ++it2)
        {
            const geo::TriangleI& triangle = *it2;

            cv::Point2d p1_2d = worldToCanvas(t * vertices[triangle.i1_]);
            cv::Point2d p2_2d = worldToCanvas(t * vertices[triangle.i2_]);
            cv::Point2d p3_2d = worldToCanvas(t * vertices[triangle.i3_]);

            cv::line(canvas, p1_2d, p2_2d, line_color, 2);
            cv::line(canvas, p2_2d, p3_2d, line_color, 2);
            cv::line(canvas, p1_2d, p3_2d, line_color, 2);
        }
    }


    std::vector<tf2::Vector3> points = findEdgePoints(scan);
    int pointSize = 8;
    cv::Scalar edgePointColor(0, 255, 0);
    for(std::vector<tf2::Vector3>::const_iterator it3 = points.begin(); it3 != points.end(); ++it3)
    {
        const tf2::Vector3& point = *it3;
        geo::Vector3 center(point.getX(), point.getY(), point.getZ());
        //cv::Point2d center(point.getX(), point.getY());
        cv::Point2d p_center = worldToCanvas(center);
        cv::circle(canvas, p_center, pointSize, edgePointColor, -1);
    }

    points = findMidPoints(scan);
    cv::Scalar midPointColor(255, 0, 0);
    for(std::vector<tf2::Vector3>::const_iterator it3 = points.begin(); it3 != points.end(); ++it3)
    {
        const tf2::Vector3& point = *it3;
        geo::Vector3 center(point.getX(), point.getY(), point.getZ());
        //cv::Point2d center(point.getX(), point.getY());
        cv::Point2d p_center = worldToCanvas(center);
        cv::circle(canvas, p_center, pointSize, midPointColor, -1);
    }

    std::cout << "hello";
    std::vector<int> changeRays = laserChange(scan);
    points = calculateXY(scan, changeRays);
    cv::Scalar changePointColor(122, 122, 0);
    for(std::vector<tf2::Vector3>::const_iterator it3 = points.begin(); it3 != points.end(); ++it3)
    {
        const tf2::Vector3& point = *it3;
        geo::Vector3 center(point.getX(), point.getY(), point.getZ());
        //cv::Point2d center(point.getX(), point.getY());
        cv::Point2d p_center = worldToCanvas(center);
        cv::circle(canvas, p_center, pointSize, changePointColor, -1);
    }

//    points = findJumpPoints(scan);
//    cv::Scalar jumpPointColor(122, 122, 0);
//    for(std::vector<tf2::Vector3>::const_iterator it3 = points.begin(); it3 != points.end(); ++it3)
//    {
//        const tf2::Vector3& point = *it3;
//        geo::Vector3 center(point.getX(), point.getY(), point.getZ());
//        //cv::Point2d center(point.getX(), point.getY());
//        std::cout << "(x, y, z) = " << point.getX() << ", " << point.getY() << ", " << point.getZ() << ")" << std::endl;
//        cv::Point2d p_center = worldToCanvas(center);
//        cv::circle(canvas, p_center, pointSize - 2, jumpPointColor, -1);
//    }
//
    cv::imshow("simulator", canvas);
    cv::waitKey(3);
}

}
