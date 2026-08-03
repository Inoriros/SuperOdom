#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace super_odometry
{

class CampusMapNode final : public rclcpp::Node
{
public:
  explicit CampusMapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CampusMapNode() override;

  CampusMapNode(const CampusMapNode &) = delete;
  CampusMapNode & operator=(const CampusMapNode &) = delete;

private:
  struct TileKey
  {
    std::int32_t x{};
    std::int32_t y{};

    bool operator==(const TileKey & other) const noexcept
    {
      return x == other.x && y == other.y;
    }
  };

  struct TileKeyHash
  {
    std::size_t operator()(const TileKey & key) const noexcept;
  };

  struct VoxelKey
  {
    std::int32_t x{};
    std::int32_t y{};
    std::int32_t z{};

    bool operator==(const VoxelKey & other) const noexcept
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const noexcept;
  };

  struct VoxelAccumulator
  {
    float x{};
    float y{};
    float z{};
    float intensity{};
    std::uint32_t observations{};
  };

  struct PendingVoxel
  {
    double sum_x{};
    double sum_y{};
    double sum_z{};
    double sum_intensity{};
    std::uint32_t observations{};
  };

  struct Tile
  {
    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    bool dirty{false};
  };

  struct TileMetadata
  {
    std::uint64_t point_count{};
    bool has_bounds{false};
    double min_x{};
    double min_y{};
    double min_z{};
    double max_x{};
    double max_y{};
    double max_z{};
  };

  enum class ControlOperation
  {
    Save,
    ExportPcd
  };

  using ControlResult = std::pair<bool, std::string>;

  struct ControlRequest
  {
    ControlOperation operation;
    std::promise<ControlResult> completion;
  };

  void declareAndValidateParameters();
  void createSessionDirectory();
  void createRosInterfaces();

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void requestCheckpoint();
  void requestLocalPublication();
  void saveService(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void exportService(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void executeControlRequest(
    ControlOperation operation,
    std_srvs::srv::Trigger::Response & response);

  void workerLoop();
  void processScan(const sensor_msgs::msg::PointCloud2 & message);
  void updateResidency(double robot_x, double robot_y);
  void evictTile(const TileKey & key);
  void enforceResidentTileLimit(double robot_x, double robot_y);
  void publishLocalMap();

  Tile & getOrLoadTile(const TileKey & key);
  bool loadTile(const TileKey & key, Tile & tile, std::string & error) const;
  bool saveTile(const TileKey & key, Tile & tile, std::string & error);
  bool flushAllTiles(std::string & error);
  bool writeManifest(std::string & error) const;
  bool exportPcd(std::string & error);
  void refreshMetadata(const TileKey & key, const Tile & tile);

  TileKey tileKeyForVoxel(const VoxelKey & voxel) const;
  double distanceToTile(double x, double y, const TileKey & key) const;
  std::filesystem::path tilePath(const TileKey & key) const;
  bool frameMatches(const std::string & frame_id) const;
  std::uint64_t totalPointCount() const;

  std::string input_cloud_topic_;
  std::string odometry_topic_;
  std::string local_map_topic_;
  std::string map_frame_;
  std::filesystem::path map_directory_;
  std::filesystem::path session_directory_;
  std::filesystem::path tile_directory_;
  std::string session_name_;
  std::string session_start_utc_;

  double voxel_size_{};
  double tile_size_{};
  double min_insertion_range_{};
  double max_insertion_range_{};
  double resident_radius_{};
  double eviction_radius_{};
  double local_publish_radius_{};
  double publish_period_{};
  double checkpoint_period_{};
  std::size_t scan_stride_{};
  std::size_t max_pending_scans_{};
  std::size_t max_resident_tiles_{};
  std::uint32_t max_observations_per_voxel_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_service_;
  rclcpp::TimerBase::SharedPtr checkpoint_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex work_mutex_;
  std::condition_variable work_condition_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> scan_queue_;
  std::deque<std::shared_ptr<ControlRequest>> control_queue_;
  bool checkpoint_requested_{false};
  bool publication_requested_{false};
  bool stopping_{false};
  std::thread worker_;

  std::unordered_map<TileKey, Tile, TileKeyHash> resident_tiles_;
  std::unordered_map<TileKey, TileMetadata, TileKeyHash> tile_catalog_;

  std::atomic<double> robot_x_{0.0};
  std::atomic<double> robot_y_{0.0};
  std::atomic<double> robot_z_{0.0};
  std::atomic<bool> pose_available_{false};
  std::atomic<std::uint64_t> received_scans_{0};
  std::atomic<std::uint64_t> processed_scans_{0};
  std::atomic<std::uint64_t> dropped_scans_{0};
  std::atomic<bool> warned_cloud_frame_{false};
  std::atomic<bool> warned_odom_frame_{false};
  builtin_interfaces::msg::Time last_cloud_stamp_;
};

}  // namespace super_odometry
