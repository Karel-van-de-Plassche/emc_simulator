#include "world.h"
#include "visualization.h"
#include "heightmap.h"
#include "lrf.h"
#include "door.h"

#include <unistd.h>

#include <tue/profiling/timer.h>

#include <geolib/ros/msg_conversions.h>

#include <geolib/Box.h>

// ROS
#include <ros/init.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <robo/debugging.h>

log_level_t GLOBAL_LEVEL = LOG_INFO;
geometry_msgs::Twist::ConstPtr base_ref_;
bool request_open_door_;

// ----------------------------------------------------------------------------------------------------

void baseReferenceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    base_ref_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void openDoorCallback(const std_msgs::Empty::ConstPtr& msg)
{
    request_open_door_ = true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pico_simulator");

    std::string heightmap_filename;
    if (argc > 1)
        heightmap_filename = argv[1];
    else
        heightmap_filename = ros::package::getPath("emc_simulator") + "/data/heightmap.pgm";

    World world;
    LRF lrf;
    lrf.setAngleLimits(-2, 2);
    lrf.setNumBeams(1000);
    lrf.setRangeLimits(0.01, 10);

    double cycle_freq = 30;
    double cycle_time = 1 / cycle_freq;

    bool visualize = true;

    // Load heightmap
    std::vector<Door> doors;
    geo::ShapePtr heightmap = createHeightMapShape(heightmap_filename, doors);
    if (!heightmap)
    {
        std::cout << "[PICO SIMULATOR] Heightmap could not be loaded" << std::endl;
        return 1;
    }

    world.addObject(geo::Pose3D::identity(), heightmap);

    // Add robot
    geo::Pose3D robot_pose = geo::Pose3D::identity();
    Id robot_id = world.addObject(robot_pose);

    // Add door
    for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
    {
        Door& door = *it;
        door.id = world.addObject(door.init_pose, door.shape, geo::Vector3(0, 1, 0));
    }

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_laser = nh.advertise<sensor_msgs::LaserScan>("/pico/laser", 1);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/pico/odom", 1);

    // Subscribers
    ros::Subscriber sub_base_ref = nh.subscribe<geometry_msgs::Twist>("/pico/base/reference", 1, baseReferenceCallback);
    ros::Subscriber sub_open_door = nh.subscribe<std_msgs::Empty>("/pico/open_door", 1, openDoorCallback);

    // Set laser pose (in robot frame)
    geo::Pose3D laser_pose = geo::Pose3D::identity();
    laser_pose.t.z = 0.3;
    emc::IO io;
    emc::LaserData scan;

    ros::Rate r(cycle_freq);
    while(ros::ok())
    {
        base_ref_.reset();
        request_open_door_ = false;
        ros::spinOnce();

        if (base_ref_)
        {
            // Set robot velocity
            world.setVelocity(robot_id, geo::Vector3(base_ref_->linear.x, base_ref_->linear.y, 0), base_ref_->angular.z);
        }

        if (request_open_door_)
        {
            for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
            {
                Door& door = *it;
                if (!door.closed)
                    continue;

                // Test if robot is in front of door. If not, don't open it
                geo::Pose3D rel_robot_pose = door.init_pose.inverse() * world.object(robot_id).pose;

                if (std::abs(rel_robot_pose.t.y) > 1 || std::abs(rel_robot_pose.t.x) > door.size / 2)
                    continue;

                world.setVelocity(door.id, door.open_vel, 0);
                door.closed = false;
            }
        }

        // Stop doors that have moved far enough
        for(std::vector<Door>::iterator it = doors.begin(); it != doors.end(); ++it)
        {
            Door& door = *it;
            if (!door.closed && (door.init_pose.t - world.object(door.id).pose.t).length2() > door.open_distance * door.open_distance)
                world.setVelocity(door.id, geo::Vector3(0, 0, 0), 0);
        }

        ros::Time time = ros::Time::now();

        world.update(time.toSec());

        // Create laser data
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = "/pico/laser";
        scan_msg.header.stamp = time;
        lrf.generateLaserData(world, world.object(robot_id).pose * laser_pose, scan_msg);
        pub_laser.publish(scan_msg);

        // Create odom data
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        geo::convert(world.object(robot_id).pose, odom_msg.pose.pose);

        pub_odom.publish(odom_msg);

        // Visualize
        if (visualize)
            io.readLaserData(scan);
            visualization::visualize(world, robot_id, scan);

        r.sleep();
    }

    return 0;
}
