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
std::ofstream odom;

// Save ROS PointCloud2 messages into txyz file
void registeredCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    for (auto point : *cloud) {
        txyz << msg->header.stamp << "," << point.x << "," << point.y << "," << point.z << "\n";
    }
}

// Save ROS odometry messages into csv file
// saves pose, orientation
// Doesn't appear that LOAM populates the velocity info but these messages would contain that
void odometryHandler(const nav_msgs::Odometry::ConstPtr &msg) {
    ros::Time time = msg->header.stamp;
    odom << msg->header.stamp << "," 
         << msg->pose.pose.position.x << ","
         << msg->pose.pose.position.y << ","
         << msg->pose.pose.position.z << ","
         << msg->pose.pose.orientation.x << ","
         << msg->pose.pose.orientation.y << ","
         << msg->pose.pose.orientation.z << ","
         << msg->pose.pose.orientation.w << "\n";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_parser");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open("/tmp/LOAM.bag");

    txyz.open("/tmp/scene.xyz");
    odom.open("/tmp/odom.csv");

    uint c = 0;
    // This is not reliable, I just previously hardcoded scenes to be 500 .xyz files long
    size_t bag_size = 1000;
    std::cout << "Parsing bagfile...\n";
    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        std::string topic = m.getTopic();

        if (topic == "/velodyne_cloud_registered") {
            sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            registeredCloudHandler(msg);
        } else if (topic == "/laser_odom_to_init") {
            nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
            odometryHandler(msg);
        }
         else {
            std::cout << "Unrecognized topic " << topic << "\n";
        }

        std::cout << "\t" <<  (100. * c) / bag_size << " %\r";
        c++;
    }
    std::cout << std::endl;

    txyz.close();
    odom.close();
    bag.close();
}