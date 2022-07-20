/*
 * Utility for saving relevant ROS topics to a bagfile
 *
 */


#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> registeredCloudBuf;
std::mutex mBuf;

rosbag::Bag bag_out;

void laserOdomHandler(const nav_msgs::Odometry::ConstPtr &msg) {
    mBuf.lock();
    odometryBuf.push(msg);
    mBuf.unlock();
}

void registeredCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
    mBuf.lock();
    registeredCloudBuf.push(msg);
    mBuf.unlock();
}


void process() {
    while (true) {
        while (!odometryBuf.empty() && !registeredCloudBuf.empty()) {
            mBuf.lock();

            // Store odometry information
            bag_out.write(
                "/laser_odom_to_init",
                odometryBuf.front()->header.stamp,
                odometryBuf.front()
            );
            odometryBuf.pop();

            // Store registered point clouds
            bag_out.write(
                "/velodyne_cloud_registered",
                registeredCloudBuf.front()->header.stamp,
                registeredCloudBuf.front()
            );


            mBuf.unlock();
        }
        std::chrono::milliseconds duration(2);
        std::this_thread::sleep_for(duration);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "loamBagger");
    ros::NodeHandle nh;

    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdomHandler);
    ros::Subscriber subRegisteredCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100, registeredCloudHandler);

    bag_out.open("/tmp/LOAM.bag", rosbag::bagmode::Write);

    std::thread mapping_process{process};

    ros::spin();

    bag_out.close();
    return 0;
}