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

rosbag::Bag bag;
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

    std::string all_scenes_folder = "/root/catkin_ws/src/A-LOAM/luminar/";
    std::size_t num_scenes = 100;
    for (uint scene=0; scene<num_scenes; scene++) {
        std::string scene_folder = all_scenes_folder + std::to_string(scene) + "/";

        std::cout << "\tParsing bagfile in" << scene_folder << "...\n";

        bag.open(scene_folder + "LOAM.bag");

        txyz.open(scene_folder + "scene.xyz");
        odom.open(scene_folder + "odom.csv");

        uint c = 0;
        size_t bag_size = 100;
        for (rosbag::MessageInstance const m: rosbag::View(bag)) {
            std::string topic = m.getTopic();

            if (topic == "/laser_cloud_map") {
                sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
                registeredCloudHandler(msg);
            } else if (topic == "/aft_mapped_to_init") {
                nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
                odometryHandler(msg);
            }
            else {
                continue;
            }

            std::cout << "\t\t" <<  (100. * c) / bag_size << " %\r";
            c++;
        }
        std::cout << std::endl;

        txyz.close();
        odom.close();
        bag.close();

    }
}