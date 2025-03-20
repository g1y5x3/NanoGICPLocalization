#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

class LocalizationPublisher : public rclcpp::Node
{
    public: 
        LocalizationPublisher()
        : Node("pcd_localization")
        {
            lidarsub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/spot/lidar/points", 10, 
                std::bind(&LocalizationPublisher::topic_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Localization node initialized. Subscribed to /spot/lidar/points");
        }

    private:
        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {
            // Basic information about the point cloud
            RCLCPP_INFO(this->get_logger(), "Received point cloud:");
            RCLCPP_INFO(this->get_logger(), "  Width: %u", msg->width);
            RCLCPP_INFO(this->get_logger(), "  Height: %u", msg->height);
            RCLCPP_INFO(this->get_logger(), "  Point step: %u", msg.point_step);
            RCLCPP_INFO(this->get_logger(), "  Row step: %u", msg.row_step);
            RCLCPP_INFO(this->get_logger(), "  Total points: %u", msg.width * msg.height);
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarsub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationPublisher>());
    rclcpp::shutdown();

    return 0;
}