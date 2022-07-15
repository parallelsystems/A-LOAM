#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

std::vector<std::vector<float>> read_lidar_data(const std::string lidar_data_path) {
    std::vector<std::vector<float>> points;

    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in);
    std::string line;

    // iterate over lines
    while (std::getline(lidar_data_file, line)) {
        std::istringstream sstream(line);
        std::string elem;
        std::vector<float> point;

        // iterate over entries in (t, x, y, z) tuple
        while (std::getline(sstream, elem, ',')) {
            point.push_back(stof(elem));
        }

        points.push_back(point);
    }

    return points;
}

int main(int argc, char** argv) {
    std::cout << "Begin luminarHelper\n";

    // initialize ROS node
    ros::init(argc, argv, "luminar_helper");
    ros::NodeHandle n("~");

    std::string dataset_folder, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);

    std::cout << "Reading from " << dataset_folder << "\n";
    
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag) {
        n.getParam("output_bag_file", output_bag_file);
        std::cout << "Creating output bag " << output_bag_file << "\n";
    }
    
    // scaling for publishing rate
    int publish_delay;
    n.getParam("publishDelay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    rosbag::Bag bag_out;
    if (to_bag) {
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    }

    // index for file load order
    std::ifstream time_file(dataset_folder + "times.txt", std::ifstream::in);
    std::ifstream map_file(dataset_folder + "files.txt", std::ifstream::in);

    std::size_t line_num = 0;
    ros::Rate r(10.0 / publish_delay);

    std::string time, fname, lidar_data_path;

    while (std::getline(time_file, time) && std::getline(map_file, fname) && ros::ok()) {

        std::cout << "\tReading lidar point cloud...\n";
        std::cout << "\t\tTime: " << time << "\n";
        std::cout << "\t\tFilename: " << fname << "\n";

        lidar_data_path = dataset_folder + fname;
        std::vector<std::vector<float>> points = read_lidar_data(lidar_data_path);
        std::cout << "\t\tPoints in file: " << points.size() << "\n";

        pcl::PointCloud<pcl::PointXYZI> laser_cloud;

        for (std::size_t i=0; i < points.size(); i++) {
            pcl::PointXYZI point;
            point.x = points[i][1];
            point.y = points[i][2];
            point.z = points[i][3];
            point.intensity = 1.;
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(points[0][0]);
        laser_cloud_msg.header.frame_id = "camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        if (to_bag) {
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
        }

        line_num++;
        // r.sleep();
    }

    bag_out.close();
    std::cout << "Done\n";


    return 0;
}