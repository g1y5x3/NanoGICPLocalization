#include <memory>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include <nano_gicp/nano_gicp.hpp>

class LocalizationPublisher : public rclcpp::Node
{
    public: 
        LocalizationPublisher()
        : Node("pcd_localization"),
          map_cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
          input_scan(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
          T(Eigen::Matrix4f::Identity())
        {
            // nano_gicp init
            this->gicp.setCorrespondenceRandomness(20);
            this->gicp.setMaxCorrespondenceDistance(0.5);
            this->gicp.setMaximumIterations(32);
            this->gicp.setTransformationEpsilon(0.01);
            this->gicp.setEuclideanFitnessEpsilon(0.01);
            this->gicp.setRANSACIterations(5);
            this->gicp.setRANSACOutlierRejectionThreshold(1.0);

            // Declare parameters with default values
            this->declare_parameter("pointcloud_topic", "/spot/lidar/points");
            this->declare_parameter("map_path", "src/spot_ros2_gazebo/spot_navigation/maps/simple_tunnel.pcd");

            // Subcribe to LiDAR inputs
            std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
            lidarsub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pointcloud_topic, 10, 
                std::bind(&LocalizationPublisher::icp_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Subscribed to /spot/lidar/points");

            // Load map file
            std::string map_path = this->get_parameter("map_path").as_string(); 
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", map_path.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully loaded map with %zu points", map_cloud->size());
            RCLCPP_INFO(this->get_logger(), "Localization node initialized.");

        }

    private:
        void icp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received point cloud:");
            RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", msg->header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "  Width: %u", msg->width);
            RCLCPP_INFO(this->get_logger(), "  Height: %u", msg->height);

            // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
            pcl::fromROSMsg(*msg, *this->input_scan);
            if (this->input_scan->points.size() < 10)
            {
                RCLCPP_FATAL(this->get_logger(), "Low number of points!");
                return;          
            }

            // Time related stuff 
            // double then = this->now().seconds();

            // Inspect the input scan and the map point clounds
            int num_points_to_print = std::min(10, static_cast<int>(this->input_scan->points.size()));
            for (int i = 0; i < num_points_to_print; ++i) {
              RCLCPP_INFO(this->get_logger(), "Point %d: x=%.3f, y=%.3f, z=%.3f", 
                          i, this->input_scan->points[i].x, this->input_scan->points[i].y, this->input_scan->points[i].z);
            }

            // filter points

            // Quick test for nano_gicp
            pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();

            // set the current input scan from lidar as the source point cloud
            this->gicp.setInputSource(this->input_scan);
            this->gicp.calculateSourceCovariances();

            // Set the map point cloud as the target point cloud
            this->gicp.setInputTarget(this->map_cloud);
            this->gicp.calculateTargetCovariances();

            this->gicp.align(*aligned);
            this->T = this->gicp.getFinalTransformation();

            // Convert the matrix to a string
            std::stringstream ss;
            ss << this->T;

            // Log the matrix
            RCLCPP_INFO(this->get_logger(), "Matrix T:\n%s", ss.str().c_str());

    }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarsub_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_scan;
        nano_gicp::NanoGICP<PointType, PointType> gicp;
        Eigen::Matrix4f T;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationPublisher>());
    rclcpp::shutdown();

    return 0;
}