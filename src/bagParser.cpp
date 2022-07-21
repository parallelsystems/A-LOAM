#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>


std::ofstream txyz;

void registeredCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    for (auto point : *cloud) {
        txyz << msg->header.stamp << "," << point.x << "," << point.y << "," << point.z << "\n";
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &msg) {
    // do something
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "loam_bagger");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open("/tmp/LOAM.bag");

    txyz.open("/tmp/scene.xyz");

    uint c = 0;
    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        std::string topic = m.getTopic();

        if (topic == "/velodyne_cloud_registered") {
            sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            registeredCloudHandler(msg);
        } else if (topic == "/laser_odom_to_init") {
            nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();

        }
         else {
            std::cout << "Unrecognized topic " << topic << "\n";
        }
        c++;
    }

    txyz.close();
    bag.close();
}