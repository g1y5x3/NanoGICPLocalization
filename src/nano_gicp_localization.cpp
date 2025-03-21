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

class NanoGICPLocalization : public rclcpp::Node
{
    public: 
        NanoGICPLocalization()
        : Node("nano_gicp_loc"),
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
                pointcloud_topic, 1, std::bind(&NanoGICPLocalization::icp_callback, this, std::placeholders::_1)
            );

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
            RCLCPP_INFO(this->get_logger(), "Frame ID: %s", msg->header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "Width: %u", msg->width);
            RCLCPP_INFO(this->get_logger(), "Height: %u", msg->height);

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
            // int num_points_to_print = std::min(10, static_cast<int>(this->input_scan->points.size()));
            // for (int i = 0; i < num_points_to_print; ++i) {
            //   RCLCPP_INFO(this->get_logger(), "Point %d: x=%.3f, y=%.3f, z=%.3f", 
            //               i, this->input_scan->points[i].x, this->input_scan->points[i].y, this->input_scan->points[i].z);
            // }

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
            RCLCPP_INFO(this->get_logger(), "Matrix T:\n%s", ss.str().c_str());
        }

        /*
        void debug() 
        {
            // Total length traversed
            double length_traversed = 0.;
            Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
            Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
            for (const auto& t : this->trajectory) {
              if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
                p_prev = t.first;
                continue;
              }
              p_curr = t.first;
              double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));
        
              if (l >= 0.05) {
                length_traversed += l;
                p_prev = p_curr;
              }
            }
      
            if (length_traversed == 0) {
              this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
              this->publish_keyframe_thread.detach();
            }
      
            // Average computation time
            double avg_comp_time = std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();
      
            // RAM Usage
            double vm_usage = 0.0;
            double resident_set = 0.0;
            std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
            std::string pid, comm, state, ppid, pgrp, session, tty_nr;
            std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
            std::string utime, stime, cutime, cstime, priority, nice;
            std::string num_threads, itrealvalue, starttime;
            unsigned long vsize;
            long rss;
            stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                        >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                        >> utime >> stime >> cutime >> cstime >> priority >> nice
                        >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
            stat_stream.close();
            long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
            vm_usage = vsize / 1024.0;
            resident_set = rss * page_size_kb;
      
            // CPU Usage
            struct tms timeSample;
            clock_t now;
            double cpu_percent;
            now = times(&timeSample);
            if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
                timeSample.tms_utime < this->lastUserCPU) {
                cpu_percent = -1.0;
            } else {
                cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
                cpu_percent /= (now - this->lastCPU);
                cpu_percent /= this->numProcessors;
                cpu_percent *= 100.;
            }
            this->lastCPU = now;
            this->lastSysCPU = timeSample.tms_stime;
            this->lastUserCPU = timeSample.tms_utime;
            this->cpu_percents.push_back(cpu_percent);
            double avg_cpu_usage = std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();
      
            // Print to terminal
            printf("\033[2J\033[1;1H");
      
            std::cout << std::endl << "==== Direct LiDAR Odometry v" << this->version_ << " ====" << std::endl;
      
            if (!this->cpu_type.empty()) {
              std::cout << std::endl << this->cpu_type << " x " << this->numProcessors << std::endl;
            }
      
            std::cout << std::endl << std::setprecision(4) << std::fixed;
            std::cout << "Position    [xyz]  :: " << this->pose[0] << " " << this->pose[1] << " " << this->pose[2] << std::endl;
            std::cout << "Orientation [wxyz] :: " << this->rotq.w() << " " << this->rotq.x() << " " << this->rotq.y() << " " << this->rotq.z() << std::endl;
            std::cout << "Distance Traveled  :: " << length_traversed << " meters" << std::endl;
            std::cout << "Distance to Origin :: " << sqrt(pow(this->pose[0]-this->origin[0],2) + pow(this->pose[1]-this->origin[1],2) + pow(this->pose[2]-this->origin[2],2)) << " meters" << std::endl;
      
            std::cout << std::endl << std::right << std::setprecision(2) << std::fixed;
            std::cout << "Computation Time :: " << std::setfill(' ') << std::setw(6) << this->comp_times.back()*1000. << " ms    // Avg: " << std::setw(5) << avg_comp_time*1000. << std::endl;
            std::cout << "Cores Utilized   :: " << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: " << std::setw(5) << (avg_cpu_usage/100.) * this->numProcessors << std::endl;
            std::cout << "CPU Load         :: " << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: " << std::setw(5) << avg_cpu_usage << std::endl;
            std::cout << "RAM Allocation   :: " << std::setfill(' ') << std::setw(6) << resident_set/1000. << " MB    // VSZ: " << vm_usage/1000. << " MB" << std::endl;
        }
        */

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarsub_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_scan;
        nano_gicp::NanoGICP<PointType, PointType> gicp;
        Eigen::Matrix4f T;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NanoGICPLocalization>());
    rclcpp::shutdown();

    return 0;
}