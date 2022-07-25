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

std::vector<std::vector<std::string>> read_lidar_data(const std::string lidar_data_path) {
    std::vector<std::vector<std::string>> points;

    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in);
    std::string line;

    // iterate over lines
    while (std::getline(lidar_data_file, line)) {
        std::istringstream sstream(line);
        std::string elem;
        std::vector<std::string> point;

        // iterate over entries in (t, x, y, z) tuple
        while (std::getline(sstream, elem, ',')) {
            point.push_back(elem);
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

    std::string base_dataset_folder = "/root/catkin_ws/src/A-LOAM/luminar/";
    std::size_t num_scenes = 100;
    for (uint scene=0; scene<num_scenes; scene++) {
        std::string dataset_folder = base_dataset_folder + std::to_string(scene) + "/";
        std::cout << "Reading from " << dataset_folder << "\n";
        
        std::string output_bag_file = dataset_folder + "luminar.bag";
        
        // scaling for publishing rate
        int publish_delay;
        n.getParam("publishDelay", publish_delay);
        publish_delay = publish_delay <= 0 ? 1 : publish_delay;

        ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

        rosbag::Bag bag_out;
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

        // index for file load order
        // std::ifstream time_file(dataset_folder + "times.txt", std::ifstream::in);
        std::ifstream map_file(dataset_folder + "files.txt", std::ifstream::in);

        ros::Rate r(10.0 / publish_delay);

        std::string fname, lidar_data_path;

        while (std::getline(map_file, fname) && ros::ok()) {
            lidar_data_path = dataset_folder + fname;
            std::vector<std::vector<std::string>> points = read_lidar_data(lidar_data_path);

            if (points.size() == 0) {
                continue;
            }

            // queue for current scan
            pcl::PointCloud<pcl::PointXYZI> laser_cloud;
            // used for determining if a point belongs to same scan
            std::string current_point_time, current_cloud_time;
            current_cloud_time  = points[0][0];

            uint n_clouds = 0;
            for (uint i=0; i<points.size(); i++) {
                pcl::PointXYZI point;
                point.x = stod(points[i][1]);
                point.y = stod(points[i][2]);
                point.z = stod(points[i][3]);
                point.intensity = 1;
                current_point_time = points[i][0];

                if (current_point_time != current_cloud_time) {
                    // broadcast existing point cloud and reset
                    sensor_msgs::PointCloud2 laser_cloud_msg;
                    pcl::toROSMsg(laser_cloud, laser_cloud_msg);
                    laser_cloud_msg.header.stamp = ros::Time().fromSec(stod(current_cloud_time));
                    laser_cloud_msg.header.frame_id = "camera_init";
                    pub_laser_cloud.publish(laser_cloud_msg);

                    bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
                    laser_cloud.clear();

                    n_clouds++;
                    current_cloud_time = current_point_time;
                }

                laser_cloud.push_back(point);
            }
            // publish last scan
            sensor_msgs::PointCloud2 laser_cloud_msg;
            pcl::toROSMsg(laser_cloud, laser_cloud_msg);
            laser_cloud_msg.header.stamp = ros::Time().fromSec(stod(current_cloud_time));
            laser_cloud_msg.header.frame_id = "camera_init";
            pub_laser_cloud.publish(laser_cloud_msg);

            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            laser_cloud.clear();
            n_clouds++;

            std::cout << "\t\tNum clouds " << n_clouds << "\n";
        }

        bag_out.close();
    }

    std::cout << "Done\n";
    return 0;
}