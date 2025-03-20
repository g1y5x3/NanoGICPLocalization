#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

class LocalizationPublisher : public rclcpp::Node
{
    public: 
        LocalizationPublisher()
        : Node("pcd_localization"),
          map_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
        {
            // Subcribe to LiDAR inputs
            lidarsub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/spot/lidar/points", 10, 
                std::bind(&LocalizationPublisher::icp_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Subscribed to /spot/lidar/points");

            // Load map file
            const std::string map_path = "/root/ros2_ws/src/spot_ros2_gazebo/spot_navigation/maps/simple_tunnel.pcd";
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
            RCLCPP_INFO(this->get_logger(), "  Width: %u", msg->width);
            RCLCPP_INFO(this->get_logger(), "  Height: %u", msg->height);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarsub_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationPublisher>());
    rclcpp::shutdown();

    return 0;
}