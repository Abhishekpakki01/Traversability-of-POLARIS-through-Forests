#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>
#include <ohm/MapLayout.h>
#include <glm/glm.hpp>

class OhmOccupancyNode : public rclcpp::Node {
public:
    OhmOccupancyNode() : Node("ohm_occupancy_node") {
        this->declare_parameter<double>("max_range", 10.0);
        this->declare_parameter<double>("resolution", 0.35);

        max_range_ = this->get_parameter("max_range").as_double();
        resolution_ = this->get_parameter("resolution").as_double();

        // Initialize OHM Map
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
        // 1. Parse PointCloud2
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        std::vector<glm::dvec3> valid_targets;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                continue;
            }

            float dist = std::sqrt(x * x + y * y + z * z);
            if (dist <= max_range_ && dist > 0.1f) {
                valid_targets.emplace_back(x, y, z);
            }
        }

        if (valid_targets.empty()) {
            return;
        }

        // 2. Format rays as pairs: [origin_1, target_1, origin_2, target_2, ...]
        std::vector<glm::dvec3> rays;
        rays.reserve(valid_targets.size() * 2);
        glm::dvec3 sensor_origin(0.0, 0.0, 0.0);

        for (const auto& target : valid_targets) {
            rays.push_back(sensor_origin);
            rays.push_back(target);
        }

        // 3. Integrate Rays using RayMapperOccupancy
        ohm::RayMapperOccupancy mapper(map_.get());
        mapper.integrateRays(rays.data(), valid_targets.size(), nullptr, nullptr, 0);

        // 4. Construct RViz Cube List Marker
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "ohm_voxels";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = resolution_;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6f;

        // 5. Query Occupancy Voxel Values
        ohm::Voxel<float> occupancy_voxel(map_.get(), map_->layout().occupancyLayer());
        glm::dvec3 region_spatial_dim = glm::dvec3(map_->regionVoxelDimensions()) * map_->resolution();

        for (auto iter = map_->begin(); iter != map_->end(); ++iter) {
            occupancy_voxel.setKey(iter.key());
            float occ_value = 0.0f;
            occupancy_voxel.read(&occ_value);

            if (occupancy_voxel.isValid() && occ_value > 0.0f) {
                glm::dvec3 voxel_center = ohm::OccupancyMap::voxelCentre(
                    iter.key(),
                    map_->resolution(),
                    map_->origin(),
                    region_spatial_dim
                );
                geometry_msgs::msg::Point p;
                p.x = voxel_center.x;
                p.y = voxel_center.y;
                p.z = voxel_center.z;
                marker.points.push_back(p);
            }
        }

        marker_pub_->publish(marker);
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