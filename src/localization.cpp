#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
// #include <nano_gicp/nano_gicp.hpp>

class LocalizationPublisher : public rclcpp::Node
{
    public: 
        LocalizationPublisher()
        : Node("pcd_localization"),
          map_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
          current_scan_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
        {
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
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud_) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", map_path.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Successfully loaded map with %zu points", map_cloud_->size());
            
            RCLCPP_INFO(this->get_logger(), "Localization node initialized.");

        }

    private:
        void icp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {
            // Basic information about the point cloud
            RCLCPP_INFO(this->get_logger(), "Received point cloud:");
            RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", msg->header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "  Width: %u", msg->width);
            RCLCPP_INFO(this->get_logger(), "  Height: %u", msg->height);

            // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
            pcl::fromROSMsg(*msg, *this->current_scan_);
            if (this->current_scan_->points.size() < 100)
            {
                RCLCPP_FATAL(this->get_logger(), "Low number of points!");
                return;          
            }
            
            // Get current times 
            // double then = this->now().seconds();
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarsub_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_;
        // double curr_frame_stamp;
        // double prev_frame_stamp;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationPublisher>());
    rclcpp::shutdown();

    return 0;
}