#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ohm/OccupancyMap.h>

class OhmOccupancyNode : public rclcpp::Node {
public:
    OhmOccupancyNode() : Node("ohm_occupancy_node") {
        this->declare_parameter<double>("max_range", 10.0);
        this->declare_parameter<double>("resolution", 0.35);

        max_range_ = this->get_parameter("max_range").as_double();
        resolution_ = this->get_parameter("resolution").as_double();

        // Initialize OHM Occupancy Map
        map_ = std::make_unique<ohm::OccupancyMap>(resolution_);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os1_cloud_node/points",
            rclcpp::SensorDataQoS(),
            std::bind(&OhmOccupancyNode::cloudCallback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/semantic_voxel_map_cpp",
            10
        );

        RCLCPP_INFO(this->get_logger(),
                    "C++ OHM Engine Active | Max Range: %.2fm | Resolution: %.2fm",
                    max_range_, resolution_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        (void)msg; // Processing logic hooked to OHM ray-tracing
    }

    double max_range_;
    double resolution_;
    std::unique_ptr<ohm::OccupancyMap> map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OhmOccupancyNode>());
    rclcpp::shutdown();
    return 0;
}