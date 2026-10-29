#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>
#include <ohm/VoxelOccupancy.h>
#include <ohm/MapLayout.h>
#include <glm/glm.hpp>

class OhmOccupancyNode : public rclcpp::Node {
public:
    OhmOccupancyNode() : Node("ohm_occupancy_pose_node") {
        this->declare_parameter<double>("max_range", 10.0);
        this->declare_parameter<double>("resolution", 0.35);
        fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        max_range_ = this->get_parameter("max_range").as_double();
        resolution_ = this->get_parameter("resolution").as_double();

        // Initialize OHM Map
        map_ = std::make_unique<ohm::OccupancyMap>(resolution_);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // A dedicated listener thread lets TF arrive while a scan waits for its pose.
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os1_cloud_node/points",
            rclcpp::SensorDataQoS(),
            std::bind(&OhmOccupancyNode::cloudCallback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/semantic_voxel_map_cpp",
            10
        );
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle_pose", 10);

        RCLCPP_INFO(this->get_logger(),
                    "C++ OHM Engine Active | Frame: %s | Max Range: %.2fm | Resolution: %.2fm",
                    fixed_frame_.c_str(), max_range_, resolution_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Time zero means "latest" in TF, which must not be used for scan integration.
        if (msg->header.frame_id.empty() ||
            (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Skipping cloud without a frame or nonzero timestamp");
            return;
        }

        geometry_msgs::msg::TransformStamped sensor_pose;
        const rclcpp::Time scan_time(msg->header.stamp, this->get_clock()->get_clock_type());
        try {
            sensor_pose = tf_buffer_->lookupTransform(
                fixed_frame_, msg->header.frame_id, scan_time,
                rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Skipping cloud: no %s <- %s pose at scan time: %s",
                                fixed_frame_.c_str(), msg->header.frame_id.c_str(), error.what());
            return;
        }

        const auto &translation = sensor_pose.transform.translation;
        const auto &rotation = sensor_pose.transform.rotation;
        const tf2::Transform world_from_sensor(
            tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
            tf2::Vector3(translation.x, translation.y, translation.z));
        const glm::dvec3 sensor_origin(translation.x, translation.y, translation.z);

        // 1. Parse PointCloud2
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        std::vector<glm::dvec3> valid_targets;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            float dist = std::sqrt(x * x + y * y + z * z);
            if (dist <= max_range_ && dist > 0.1f) {
                const tf2::Vector3 target = world_from_sensor * tf2::Vector3(x, y, z);
                valid_targets.emplace_back(target.x(), target.y(), target.z());
            }
        }

        if (valid_targets.empty()) {
            return;
        }

        // 2. Format rays as pairs: [origin_1, target_1, origin_2, target_2, ...]
        std::vector<glm::dvec3> rays;
        rays.reserve(valid_targets.size() * 2);

        for (const auto& target : valid_targets) {
            rays.push_back(sensor_origin);
            rays.push_back(target);
        }

        // 3. Integrate Rays using RayMapperOccupancy
        ohm::RayMapperOccupancy mapper(map_.get());
        // OHM expects the number of vector elements, i.e. twice the ray count.
        mapper.integrateRays(rays.data(), rays.size(), nullptr, nullptr, 0);

        // Expose the vehicle pose (not the offset LiDAR pose) at this scan's timestamp.
        try {
            const auto vehicle_pose = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, scan_time);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = fixed_frame_;
            pose.header.stamp = msg->header.stamp;
            pose.pose.position.x = vehicle_pose.transform.translation.x;
            pose.pose.position.y = vehicle_pose.transform.translation.y;
            pose.pose.position.z = vehicle_pose.transform.translation.z;
            pose.pose.orientation = vehicle_pose.transform.rotation;
            pose_pub_->publish(pose);
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Cannot publish vehicle pose for %s: %s", base_frame_.c_str(), error.what());
        }

        // 4. Construct RViz Cube List Marker
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.header.frame_id = fixed_frame_;
        marker.ns = "ohm_voxels";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = resolution_;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6f;

        // 5. Query Occupancy Voxel Values
        ohm::Voxel<float> occupancy_voxel(map_.get(), map_->layout().occupancyLayer());

        for (auto iter = map_->begin(); iter != map_->end(); ++iter) {
            occupancy_voxel.setKey(iter.key());
            // Unknown OHM cells contain +infinity, so a plain > 0 check renders them.
            // Use OHM's threshold and explicitly exclude unobserved cells.
            if (ohm::isOccupied(occupancy_voxel)) {
                const glm::dvec3 voxel_center = map_->voxelCentreGlobal(iter.key());
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
    std::string fixed_frame_;
    std::string base_frame_;
    std::unique_ptr<ohm::OccupancyMap> map_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OhmOccupancyNode>());
    rclcpp::shutdown();
    return 0;
}
