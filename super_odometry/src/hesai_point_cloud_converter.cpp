#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>

// Custom point structure matching Velodyne format expected by SuperOdometry
struct VelodynePoint {
    float x, y, z;
    float intensity;
    float time;  // time offset in seconds
    uint16_t ring;
};

class HesaiPointCloudConverter : public rclcpp::Node {
public:
    HesaiPointCloudConverter() : Node("hesai_point_cloud_converter") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points",
            10,
            std::bind(&HesaiPointCloudConverter::point_cloud_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_points_corrected",
            10
        );

        RCLCPP_INFO(this->get_logger(), 
            "Hesai Point Cloud Converter started - converting to Velodyne-compatible format");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Find field offsets in the Hesai message
        uint32_t x_offset = 0, y_offset = 0, z_offset = 0;
        uint32_t intensity_offset = 0, ring_offset = 0, timestamp_offset = 0;
        
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "intensity") intensity_offset = field.offset;
            else if (field.name == "ring") ring_offset = field.offset;
            else if (field.name == "timestamp") timestamp_offset = field.offset;
        }

        // Create output message
        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        output_msg->header = msg->header;
        output_msg->height = msg->height;
        output_msg->width = msg->width;
        output_msg->is_bigendian = false;
        output_msg->point_step = sizeof(VelodynePoint);
        output_msg->row_step = output_msg->point_step * msg->width;

        // Define fields for output (Velodyne-compatible format)
        output_msg->fields.resize(6);
        output_msg->fields[0].name = "x";
        output_msg->fields[0].offset = offsetof(VelodynePoint, x);
        output_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg->fields[0].count = 1;

        output_msg->fields[1].name = "y";
        output_msg->fields[1].offset = offsetof(VelodynePoint, y);
        output_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg->fields[1].count = 1;

        output_msg->fields[2].name = "z";
        output_msg->fields[2].offset = offsetof(VelodynePoint, z);
        output_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg->fields[2].count = 1;

        output_msg->fields[3].name = "intensity";
        output_msg->fields[3].offset = offsetof(VelodynePoint, intensity);
        output_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg->fields[3].count = 1;

        output_msg->fields[4].name = "time";
        output_msg->fields[4].offset = offsetof(VelodynePoint, time);
        output_msg->fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg->fields[4].count = 1;

        output_msg->fields[5].name = "ring";
        output_msg->fields[5].offset = offsetof(VelodynePoint, ring);
        output_msg->fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
        output_msg->fields[5].count = 1;

        // Convert points
        output_msg->data.resize(output_msg->row_step * msg->height);

        double first_timestamp = 0.0;
        bool first_timestamp_initialized = false;
        
        for (uint32_t i = 0; i < msg->width; ++i) {
            uint32_t input_offset = i * msg->point_step;
            uint32_t output_offset = i * sizeof(VelodynePoint);

            VelodynePoint point;
            
            // Extract fields from input
            std::memcpy(&point.x, &msg->data[input_offset + x_offset], sizeof(float));
            std::memcpy(&point.y, &msg->data[input_offset + y_offset], sizeof(float));
            std::memcpy(&point.z, &msg->data[input_offset + z_offset], sizeof(float));
            std::memcpy(&point.intensity, &msg->data[input_offset + intensity_offset], sizeof(float));
            std::memcpy(&point.ring, &msg->data[input_offset + ring_offset], sizeof(uint16_t));
            
            double timestamp;
            std::memcpy(&timestamp, &msg->data[input_offset + timestamp_offset], sizeof(double));
            if (!first_timestamp_initialized) {
                first_timestamp = timestamp;
                first_timestamp_initialized = true;
            }
            point.time = static_cast<float>(timestamp - first_timestamp);

            // Write point to output
            std::memcpy(&output_msg->data[output_offset], &point, sizeof(VelodynePoint));
        }

        publisher_->publish(*output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HesaiPointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
