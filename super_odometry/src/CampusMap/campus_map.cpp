#include "super_odometry/CampusMap/campus_map.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace
{

constexpr std::array<char, 8> kTileMagic{{'C', 'M', 'A', 'P', 'T', 'I', 'L', 'E'}};
constexpr std::uint32_t kTileFormatVersion = 1;
constexpr std::uint64_t kTileHeaderBytes = 44;
constexpr std::uint64_t kTileEntryBytes = 32;

template<typename T>
bool writeBinary(std::ostream & stream, const T & value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return stream.good();
}

template<typename T>
bool readBinary(std::istream & stream, T & value)
{
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return stream.good();
}

std::string utcTimestamp(const char * format)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm utc_time{};
  gmtime_r(&now_time, &utc_time);
  std::ostringstream stream;
  stream << std::put_time(&utc_time, format);
  return stream.str();
}

std::string normalizeFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

std::string yamlQuote(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
}

std::size_t mixHash(std::size_t seed, std::uint32_t value) noexcept
{
  seed ^= static_cast<std::size_t>(value) + static_cast<std::size_t>(0x9e3779b9U) +
    (seed << 6U) + (seed >> 2U);
  return seed;
}

}  // namespace

namespace super_odometry
{

std::size_t CampusMapNode::TileKeyHash::operator()(const TileKey & key) const noexcept
{
  std::size_t seed = 0;
  seed = mixHash(seed, static_cast<std::uint32_t>(key.x));
  return mixHash(seed, static_cast<std::uint32_t>(key.y));
}

std::size_t CampusMapNode::VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  std::size_t seed = 0;
  seed = mixHash(seed, static_cast<std::uint32_t>(key.x));
  seed = mixHash(seed, static_cast<std::uint32_t>(key.y));
  return mixHash(seed, static_cast<std::uint32_t>(key.z));
}

CampusMapNode::CampusMapNode(const rclcpp::NodeOptions & options)
: Node("campus_map_node", options)
{
  declareAndValidateParameters();
  createSessionDirectory();

  std::string error;
  if (!writeManifest(error)) {
    throw std::runtime_error("Could not write initial campus-map manifest: " + error);
  }

  createRosInterfaces();
  worker_ = std::thread(&CampusMapNode::workerLoop, this);

  RCLCPP_INFO(
    get_logger(),
    "Campus map session '%s' started in %s (voxel %.3f m, tile %.1f m)",
    session_name_.c_str(), session_directory_.c_str(), voxel_size_, tile_size_);
}

CampusMapNode::~CampusMapNode()
{
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    stopping_ = true;
  }
  work_condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void CampusMapNode::declareAndValidateParameters()
{
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/registered_scan0");
  odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/laser_odometry");
  local_map_topic_ = declare_parameter<std::string>("local_map_topic", "/campus_map/local");
  map_frame_ = normalizeFrame(declare_parameter<std::string>("map_frame", "map"));
  map_directory_ = declare_parameter<std::string>(
    "map_directory", "/root/ros2_ws/src/SuperOdom/maps");

  voxel_size_ = declare_parameter<double>("voxel_size", 0.20);
  tile_size_ = declare_parameter<double>("tile_size", 50.0);
  min_insertion_range_ = declare_parameter<double>("min_insertion_range", 0.8);
  max_insertion_range_ = declare_parameter<double>("max_insertion_range", 80.0);
  resident_radius_ = declare_parameter<double>("resident_radius", 120.0);
  eviction_radius_ = declare_parameter<double>("eviction_radius", 150.0);
  local_publish_radius_ = declare_parameter<double>("local_publish_radius", 60.0);
  publish_period_ = declare_parameter<double>("publish_period", 2.0);
  checkpoint_period_ = declare_parameter<double>("checkpoint_period", 60.0);

  const auto scan_stride = declare_parameter<int>("scan_stride", 2);
  const auto max_pending_scans = declare_parameter<int>("max_pending_scans", 2);
  const auto max_resident_tiles = declare_parameter<int>("max_resident_tiles", 64);
  const auto max_observations = declare_parameter<int>("max_observations_per_voxel", 100);

  if (input_cloud_topic_.empty() || odometry_topic_.empty() || local_map_topic_.empty()) {
    throw std::invalid_argument("Campus-map topic parameters must not be empty");
  }
  if (map_frame_.empty()) {
    throw std::invalid_argument("map_frame must not be empty");
  }
  if (map_directory_.empty()) {
    throw std::invalid_argument("map_directory must not be empty");
  }
  if (!(voxel_size_ > 0.0) || !(tile_size_ > 0.0)) {
    throw std::invalid_argument("voxel_size and tile_size must be positive");
  }
  if (min_insertion_range_ < 0.0 || max_insertion_range_ <= min_insertion_range_) {
    throw std::invalid_argument("Insertion range must satisfy 0 <= min < max");
  }
  if (!(local_publish_radius_ > 0.0) || resident_radius_ < local_publish_radius_ ||
    eviction_radius_ <= resident_radius_)
  {
    throw std::invalid_argument(
            "Map radii must satisfy 0 < local_publish <= resident < eviction");
  }
  if (!(publish_period_ > 0.0) || !(checkpoint_period_ > 0.0)) {
    throw std::invalid_argument("publish_period and checkpoint_period must be positive");
  }
  if (scan_stride < 1 || max_pending_scans < 1 || max_resident_tiles < 1 ||
    max_observations < 1)
  {
    throw std::invalid_argument(
            "scan_stride, queue/tile limits, and observation limit must be at least one");
  }

  scan_stride_ = static_cast<std::size_t>(scan_stride);
  max_pending_scans_ = static_cast<std::size_t>(max_pending_scans);
  max_resident_tiles_ = static_cast<std::size_t>(max_resident_tiles);
  max_observations_per_voxel_ = static_cast<std::uint32_t>(max_observations);
}

void CampusMapNode::createSessionDirectory()
{
  std::error_code error;
  std::filesystem::create_directories(map_directory_, error);
  if (error) {
    throw std::runtime_error(
            "Could not create map directory '" + map_directory_.string() + "': " +
            error.message());
  }

  session_start_utc_ = utcTimestamp("%Y-%m-%dT%H:%M:%SZ");
  const std::string base_name = "session_" + utcTimestamp("%Y%m%d_%H%M%S");
  for (std::size_t suffix = 0; suffix < 10000; ++suffix) {
    session_name_ = suffix == 0 ? base_name : base_name + "_" + std::to_string(suffix);
    session_directory_ = map_directory_ / session_name_;
    error.clear();
    if (std::filesystem::create_directory(session_directory_, error)) {
      break;
    }
    if (error) {
      throw std::runtime_error(
              "Could not create map session '" + session_directory_.string() + "': " +
              error.message());
    }
    if (suffix == 9999) {
      throw std::runtime_error("Could not allocate a unique campus-map session directory");
    }
  }

  tile_directory_ = session_directory_ / "tiles";
  error.clear();
  std::filesystem::create_directory(tile_directory_, error);
  if (error) {
    throw std::runtime_error(
            "Could not create tile directory '" + tile_directory_.string() + "': " +
            error.message());
  }
}

void CampusMapNode::createRosInterfaces()
{
  rclcpp::SensorDataQoS sensor_qos;
  sensor_qos.keep_last(5);
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, sensor_qos,
    std::bind(&CampusMapNode::cloudCallback, this, std::placeholders::_1));
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, sensor_qos,
    std::bind(&CampusMapNode::odometryCallback, this, std::placeholders::_1));

  auto local_map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    local_map_topic_, local_map_qos);

  save_service_ = create_service<std_srvs::srv::Trigger>(
    "/campus_map/save",
    std::bind(
      &CampusMapNode::saveService, this, std::placeholders::_1, std::placeholders::_2));
  export_service_ = create_service<std_srvs::srv::Trigger>(
    "/campus_map/export_pcd",
    std::bind(
      &CampusMapNode::exportService, this, std::placeholders::_1, std::placeholders::_2));

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_),
    std::bind(&CampusMapNode::requestLocalPublication, this));
  checkpoint_timer_ = create_wall_timer(
    std::chrono::duration<double>(checkpoint_period_),
    std::bind(&CampusMapNode::requestCheckpoint, this));
}

void CampusMapNode::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  const std::uint64_t scan_number = received_scans_.fetch_add(1) + 1;
  if (!frameMatches(message->header.frame_id)) {
    if (!warned_cloud_frame_.exchange(true)) {
      RCLCPP_ERROR(
        get_logger(), "Ignoring clouds in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
    }
    return;
  }
  if ((scan_number - 1) % scan_stride_ != 0) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    if (stopping_) {
      return;
    }
    if (scan_queue_.size() >= max_pending_scans_) {
      scan_queue_.pop_front();
      const std::uint64_t dropped = dropped_scans_.fetch_add(1) + 1;
      if (dropped == 1 || dropped % 100 == 0) {
        RCLCPP_WARN(
          get_logger(), "Campus-map worker is behind; dropped %lu queued scans",
          static_cast<unsigned long>(dropped));
      }
    }
    scan_queue_.push_back(std::move(message));
  }
  work_condition_.notify_one();
}

void CampusMapNode::odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  if (!frameMatches(message->header.frame_id)) {
    if (!warned_odom_frame_.exchange(true)) {
      RCLCPP_ERROR(
        get_logger(), "Ignoring odometry in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
    }
    return;
  }

  const auto & position = message->pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z))
  {
    return;
  }
  robot_x_.store(position.x);
  robot_y_.store(position.y);
  robot_z_.store(position.z);
  pose_available_.store(true);
}

void CampusMapNode::requestCheckpoint()
{
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    if (stopping_) {
      return;
    }
    checkpoint_requested_ = true;
  }
  work_condition_.notify_one();
}

void CampusMapNode::requestLocalPublication()
{
  if (local_map_publisher_->get_subscription_count() == 0) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    if (stopping_) {
      return;
    }
    publication_requested_ = true;
  }
  work_condition_.notify_one();
}

void CampusMapNode::saveService(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  executeControlRequest(ControlOperation::Save, *response);
}

void CampusMapNode::exportService(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  executeControlRequest(ControlOperation::ExportPcd, *response);
}

void CampusMapNode::executeControlRequest(
  ControlOperation operation, std_srvs::srv::Trigger::Response & response)
{
  auto request = std::make_shared<ControlRequest>();
  request->operation = operation;
  auto completion = request->completion.get_future();
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    if (stopping_) {
      response.success = false;
      response.message = "Campus-map node is shutting down";
      return;
    }
    control_queue_.push_back(request);
  }
  work_condition_.notify_one();

  try {
    const auto result = completion.get();
    response.success = result.first;
    response.message = result.second;
  } catch (const std::exception & exception) {
    response.success = false;
    response.message = std::string("Campus-map operation failed: ") + exception.what();
  }
}

void CampusMapNode::workerLoop()
{
  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scan;
    std::shared_ptr<ControlRequest> control;
    bool checkpoint = false;
    bool publish = false;
    {
      std::unique_lock<std::mutex> lock(work_mutex_);
      work_condition_.wait(lock, [this]() {
        return stopping_ || !control_queue_.empty() || checkpoint_requested_ ||
               publication_requested_ || !scan_queue_.empty();
      });
      if (stopping_) {
        break;
      }
      if (!control_queue_.empty()) {
        control = control_queue_.front();
        control_queue_.pop_front();
      } else if (checkpoint_requested_) {
        checkpoint_requested_ = false;
        checkpoint = true;
      } else if (publication_requested_) {
        publication_requested_ = false;
        publish = true;
      } else if (!scan_queue_.empty()) {
        scan = scan_queue_.front();
        scan_queue_.pop_front();
      }
    }

    try {
      if (control) {
        std::string error;
        bool success = false;
        std::string message;
        if (control->operation == ControlOperation::Save) {
          success = flushAllTiles(error);
          message = success ?
            "Saved " + std::to_string(totalPointCount()) + " map points to " +
            session_directory_.string() : error;
        } else {
          success = exportPcd(error);
          message = success ?
            "Exported " + std::to_string(totalPointCount()) + " map points to " +
            (session_directory_ / "map.pcd").string() : error;
        }
        control->completion.set_value({success, message});
      } else if (checkpoint) {
        std::string error;
        if (!flushAllTiles(error)) {
          RCLCPP_ERROR(get_logger(), "Campus-map checkpoint failed: %s", error.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(), "Campus-map checkpoint: %lu points in %zu tiles (%zu resident)",
            static_cast<unsigned long>(totalPointCount()), tile_catalog_.size(),
            resident_tiles_.size());
        }
      } else if (publish) {
        publishLocalMap();
      } else if (scan) {
        processScan(*scan);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Campus-map worker error: %s", exception.what());
      if (control) {
        try {
          control->completion.set_value({false, exception.what()});
        } catch (const std::future_error &) {
          // The request already received a result.
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    for (const auto & request : control_queue_) {
      request->completion.set_value({false, "Campus-map node is shutting down"});
    }
    control_queue_.clear();
    scan_queue_.clear();
  }

  std::string error;
  if (!flushAllTiles(error)) {
    RCLCPP_ERROR(get_logger(), "Final campus-map save failed: %s", error.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "Campus-map session saved with %lu points in %zu tiles",
      static_cast<unsigned long>(totalPointCount()), tile_catalog_.size());
  }
}

void CampusMapNode::processScan(const sensor_msgs::msg::PointCloud2 & message)
{
  if (!pose_available_.load()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for map-frame odometry before inserting registered scans");
    return;
  }

  const double robot_x = robot_x_.load();
  const double robot_y = robot_y_.load();
  const double robot_z = robot_z_.load();
  const double minimum_range_squared = min_insertion_range_ * min_insertion_range_;
  const double maximum_range_squared = max_insertion_range_ * max_insertion_range_;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(message, cloud);
  std::unordered_map<VoxelKey, PendingVoxel, VoxelKeyHash> pending_voxels;
  pending_voxels.reserve(cloud.size() / 2 + 1);

  const double minimum_index = static_cast<double>(std::numeric_limits<std::int32_t>::min());
  const double maximum_index = static_cast<double>(std::numeric_limits<std::int32_t>::max());
  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double dx = static_cast<double>(point.x) - robot_x;
    const double dy = static_cast<double>(point.y) - robot_y;
    const double dz = static_cast<double>(point.z) - robot_z;
    const double range_squared = dx * dx + dy * dy + dz * dz;
    if (range_squared < minimum_range_squared || range_squared > maximum_range_squared) {
      continue;
    }

    const double voxel_x = std::floor(static_cast<double>(point.x) / voxel_size_);
    const double voxel_y = std::floor(static_cast<double>(point.y) / voxel_size_);
    const double voxel_z = std::floor(static_cast<double>(point.z) / voxel_size_);
    if (voxel_x < minimum_index || voxel_x > maximum_index ||
      voxel_y < minimum_index || voxel_y > maximum_index ||
      voxel_z < minimum_index || voxel_z > maximum_index)
    {
      continue;
    }

    const VoxelKey key{
      static_cast<std::int32_t>(voxel_x),
      static_cast<std::int32_t>(voxel_y),
      static_cast<std::int32_t>(voxel_z)};
    auto & pending = pending_voxels[key];
    pending.sum_x += point.x;
    pending.sum_y += point.y;
    pending.sum_z += point.z;
    pending.sum_intensity += std::isfinite(point.intensity) ? point.intensity : 0.0F;
    if (pending.observations < std::numeric_limits<std::uint32_t>::max()) {
      ++pending.observations;
    }
  }

  for (const auto & item : pending_voxels) {
    const VoxelKey & voxel_key = item.first;
    const PendingVoxel & pending = item.second;
    if (pending.observations == 0) {
      continue;
    }
    const TileKey tile_key = tileKeyForVoxel(voxel_key);
    Tile & tile = getOrLoadTile(tile_key);
    auto insertion = tile.voxels.try_emplace(voxel_key);
    VoxelAccumulator & accumulated = insertion.first->second;
    const double inverse_count = 1.0 / static_cast<double>(pending.observations);
    const float mean_x = static_cast<float>(pending.sum_x * inverse_count);
    const float mean_y = static_cast<float>(pending.sum_y * inverse_count);
    const float mean_z = static_cast<float>(pending.sum_z * inverse_count);
    const float mean_intensity = static_cast<float>(pending.sum_intensity * inverse_count);

    bool changed = false;
    if (insertion.second) {
      const std::uint32_t accepted = std::min(
        pending.observations, max_observations_per_voxel_);
      accumulated = {mean_x, mean_y, mean_z, mean_intensity, accepted};
      ++tile_catalog_[tile_key].point_count;
      changed = true;
    } else if (accumulated.observations < max_observations_per_voxel_) {
      const std::uint32_t accepted = std::min(
        pending.observations, max_observations_per_voxel_ - accumulated.observations);
      const double previous_weight = accumulated.observations;
      const double combined_weight = previous_weight + accepted;
      accumulated.x = static_cast<float>(
        (accumulated.x * previous_weight + mean_x * accepted) / combined_weight);
      accumulated.y = static_cast<float>(
        (accumulated.y * previous_weight + mean_y * accepted) / combined_weight);
      accumulated.z = static_cast<float>(
        (accumulated.z * previous_weight + mean_z * accepted) / combined_weight);
      accumulated.intensity = static_cast<float>(
        (accumulated.intensity * previous_weight + mean_intensity * accepted) /
        combined_weight);
      accumulated.observations += accepted;
      changed = accepted > 0;
    }

    if (changed) {
      tile.dirty = true;
      TileMetadata & metadata = tile_catalog_[tile_key];
      if (!metadata.has_bounds) {
        metadata.has_bounds = true;
        metadata.min_x = metadata.max_x = accumulated.x;
        metadata.min_y = metadata.max_y = accumulated.y;
        metadata.min_z = metadata.max_z = accumulated.z;
      } else {
        metadata.min_x = std::min(metadata.min_x, static_cast<double>(accumulated.x));
        metadata.min_y = std::min(metadata.min_y, static_cast<double>(accumulated.y));
        metadata.min_z = std::min(metadata.min_z, static_cast<double>(accumulated.z));
        metadata.max_x = std::max(metadata.max_x, static_cast<double>(accumulated.x));
        metadata.max_y = std::max(metadata.max_y, static_cast<double>(accumulated.y));
        metadata.max_z = std::max(metadata.max_z, static_cast<double>(accumulated.z));
      }
    }
  }

  last_cloud_stamp_ = message.header.stamp;
  const std::uint64_t processed = processed_scans_.fetch_add(1) + 1;
  updateResidency(robot_x, robot_y);
  if (processed % 100 == 0) {
    RCLCPP_INFO(
      get_logger(), "Campus map: %lu scans, %lu points, %zu tiles (%zu resident), %lu dropped",
      static_cast<unsigned long>(processed), static_cast<unsigned long>(totalPointCount()),
      tile_catalog_.size(), resident_tiles_.size(),
      static_cast<unsigned long>(dropped_scans_.load()));
  }
}

void CampusMapNode::updateResidency(double robot_x, double robot_y)
{
  const auto minimum_tile_x = static_cast<std::int64_t>(
    std::floor((robot_x - resident_radius_) / tile_size_));
  const auto maximum_tile_x = static_cast<std::int64_t>(
    std::floor((robot_x + resident_radius_) / tile_size_));
  const auto minimum_tile_y = static_cast<std::int64_t>(
    std::floor((robot_y - resident_radius_) / tile_size_));
  const auto maximum_tile_y = static_cast<std::int64_t>(
    std::floor((robot_y + resident_radius_) / tile_size_));

  for (std::int64_t x = minimum_tile_x; x <= maximum_tile_x; ++x) {
    for (std::int64_t y = minimum_tile_y; y <= maximum_tile_y; ++y) {
      if (x < std::numeric_limits<std::int32_t>::min() ||
        x > std::numeric_limits<std::int32_t>::max() ||
        y < std::numeric_limits<std::int32_t>::min() ||
        y > std::numeric_limits<std::int32_t>::max())
      {
        continue;
      }
      const TileKey key{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
      if (distanceToTile(robot_x, robot_y, key) <= resident_radius_ &&
        tile_catalog_.find(key) != tile_catalog_.end() &&
        resident_tiles_.find(key) == resident_tiles_.end())
      {
        try {
          getOrLoadTile(key);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            get_logger(), "Could not reload campus-map tile (%d, %d): %s",
            key.x, key.y, exception.what());
        }
      }
    }
  }

  std::vector<TileKey> distant_tiles;
  for (const auto & item : resident_tiles_) {
    if (distanceToTile(robot_x, robot_y, item.first) > eviction_radius_) {
      distant_tiles.push_back(item.first);
    }
  }
  for (const TileKey & key : distant_tiles) {
    evictTile(key);
  }
  enforceResidentTileLimit(robot_x, robot_y);
}

void CampusMapNode::evictTile(const TileKey & key)
{
  auto tile = resident_tiles_.find(key);
  if (tile == resident_tiles_.end()) {
    return;
  }
  if (tile->second.dirty) {
    std::string error;
    if (!saveTile(key, tile->second, error)) {
      RCLCPP_ERROR(
        get_logger(), "Keeping tile (%d, %d) in memory after save failure: %s",
        key.x, key.y, error.c_str());
      return;
    }
  }
  resident_tiles_.erase(tile);
}

void CampusMapNode::enforceResidentTileLimit(double robot_x, double robot_y)
{
  if (resident_tiles_.size() <= max_resident_tiles_) {
    return;
  }
  std::vector<std::pair<double, TileKey>> candidates;
  candidates.reserve(resident_tiles_.size());
  for (const auto & item : resident_tiles_) {
    candidates.emplace_back(distanceToTile(robot_x, robot_y, item.first), item.first);
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const auto & left, const auto & right) {return left.first > right.first;});
  for (const auto & candidate : candidates) {
    if (resident_tiles_.size() <= max_resident_tiles_) {
      break;
    }
    evictTile(candidate.second);
  }
}

void CampusMapNode::publishLocalMap()
{
  if (!pose_available_.load() || processed_scans_.load() == 0 ||
    local_map_publisher_->get_subscription_count() == 0)
  {
    return;
  }
  const double robot_x = robot_x_.load();
  const double robot_y = robot_y_.load();
  const double radius_squared = local_publish_radius_ * local_publish_radius_;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const auto & tile_item : resident_tiles_) {
    if (distanceToTile(robot_x, robot_y, tile_item.first) > local_publish_radius_) {
      continue;
    }
    for (const auto & voxel_item : tile_item.second.voxels) {
      const VoxelAccumulator & voxel = voxel_item.second;
      const double dx = static_cast<double>(voxel.x) - robot_x;
      const double dy = static_cast<double>(voxel.y) - robot_y;
      if (dx * dx + dy * dy > radius_squared) {
        continue;
      }
      pcl::PointXYZI point;
      point.x = voxel.x;
      point.y = voxel.y;
      point.z = voxel.z;
      point.intensity = voxel.intensity;
      cloud.push_back(point);
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = false;

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = map_frame_;
  message.header.stamp = last_cloud_stamp_;
  local_map_publisher_->publish(message);
}

CampusMapNode::Tile & CampusMapNode::getOrLoadTile(const TileKey & key)
{
  auto resident = resident_tiles_.find(key);
  if (resident != resident_tiles_.end()) {
    return resident->second;
  }

  Tile tile;
  const auto metadata = tile_catalog_.find(key);
  if (metadata != tile_catalog_.end()) {
    std::string error;
    if (!loadTile(key, tile, error)) {
      throw std::runtime_error(error);
    }
  } else {
    tile_catalog_.emplace(key, TileMetadata{});
  }
  return resident_tiles_.emplace(key, std::move(tile)).first->second;
}

bool CampusMapNode::loadTile(const TileKey & key, Tile & tile, std::string & error) const
{
  const auto path = tilePath(key);
  std::error_code file_error;
  const std::uint64_t file_size = std::filesystem::file_size(path, file_error);
  if (file_error) {
    error = "Could not inspect tile '" + path.string() + "': " + file_error.message();
    return false;
  }
  if (file_size < kTileHeaderBytes || (file_size - kTileHeaderBytes) % kTileEntryBytes != 0) {
    error = "Tile has an invalid file size: " + path.string();
    return false;
  }

  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "Could not open tile for reading: " + path.string();
    return false;
  }
  std::array<char, 8> magic{};
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  std::uint32_t version{};
  std::int32_t stored_x{};
  std::int32_t stored_y{};
  double stored_voxel_size{};
  double stored_tile_size{};
  std::uint64_t point_count{};
  if (magic != kTileMagic || !readBinary(stream, version) || !readBinary(stream, stored_x) ||
    !readBinary(stream, stored_y) || !readBinary(stream, stored_voxel_size) ||
    !readBinary(stream, stored_tile_size) || !readBinary(stream, point_count))
  {
    error = "Tile header is invalid: " + path.string();
    return false;
  }
  if (version != kTileFormatVersion || stored_x != key.x || stored_y != key.y ||
    std::abs(stored_voxel_size - voxel_size_) > 1e-9 ||
    std::abs(stored_tile_size - tile_size_) > 1e-9 ||
    point_count != (file_size - kTileHeaderBytes) / kTileEntryBytes)
  {
    error = "Tile metadata does not match the active map configuration: " + path.string();
    return false;
  }

  tile.voxels.clear();
  tile.voxels.reserve(static_cast<std::size_t>(point_count));
  for (std::uint64_t index = 0; index < point_count; ++index) {
    VoxelKey voxel_key;
    VoxelAccumulator voxel;
    if (!readBinary(stream, voxel_key.x) || !readBinary(stream, voxel_key.y) ||
      !readBinary(stream, voxel_key.z) || !readBinary(stream, voxel.x) ||
      !readBinary(stream, voxel.y) || !readBinary(stream, voxel.z) ||
      !readBinary(stream, voxel.intensity) || !readBinary(stream, voxel.observations))
    {
      error = "Tile data is truncated: " + path.string();
      return false;
    }
    if (voxel.observations == 0 || voxel.observations > max_observations_per_voxel_ ||
      !std::isfinite(voxel.x) || !std::isfinite(voxel.y) || !std::isfinite(voxel.z) ||
      !std::isfinite(voxel.intensity) ||
      !(tileKeyForVoxel(voxel_key) == key) || !tile.voxels.emplace(voxel_key, voxel).second)
    {
      error = "Tile contains invalid or duplicate voxel data: " + path.string();
      return false;
    }
  }
  tile.dirty = false;
  return true;
}

bool CampusMapNode::saveTile(const TileKey & key, Tile & tile, std::string & error)
{
  refreshMetadata(key, tile);
  const auto final_path = tilePath(key);
  auto temporary_path = final_path;
  temporary_path += ".tmp";

  std::vector<std::pair<VoxelKey, VoxelAccumulator>> entries;
  entries.reserve(tile.voxels.size());
  for (const auto & item : tile.voxels) {
    entries.push_back(item);
  }
  std::sort(entries.begin(), entries.end(), [](const auto & left, const auto & right) {
    if (left.first.x != right.first.x) {
      return left.first.x < right.first.x;
    }
    if (left.first.y != right.first.y) {
      return left.first.y < right.first.y;
    }
    return left.first.z < right.first.z;
  });

  std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    error = "Could not open temporary tile for writing: " + temporary_path.string();
    return false;
  }
  stream.write(kTileMagic.data(), static_cast<std::streamsize>(kTileMagic.size()));
  const std::uint64_t point_count = entries.size();
  if (!writeBinary(stream, kTileFormatVersion) || !writeBinary(stream, key.x) ||
    !writeBinary(stream, key.y) || !writeBinary(stream, voxel_size_) ||
    !writeBinary(stream, tile_size_) || !writeBinary(stream, point_count))
  {
    error = "Could not write tile header: " + temporary_path.string();
    return false;
  }
  for (const auto & item : entries) {
    const VoxelKey & voxel_key = item.first;
    const VoxelAccumulator & voxel = item.second;
    if (!writeBinary(stream, voxel_key.x) || !writeBinary(stream, voxel_key.y) ||
      !writeBinary(stream, voxel_key.z) || !writeBinary(stream, voxel.x) ||
      !writeBinary(stream, voxel.y) || !writeBinary(stream, voxel.z) ||
      !writeBinary(stream, voxel.intensity) || !writeBinary(stream, voxel.observations))
    {
      error = "Could not write tile data: " + temporary_path.string();
      return false;
    }
  }
  stream.close();
  if (!stream) {
    error = "Could not finish writing tile: " + temporary_path.string();
    return false;
  }

  std::error_code rename_error;
  std::filesystem::rename(temporary_path, final_path, rename_error);
  if (rename_error) {
    error = "Could not atomically replace tile '" + final_path.string() + "': " +
      rename_error.message();
    return false;
  }
  tile.dirty = false;
  return true;
}

bool CampusMapNode::flushAllTiles(std::string & error)
{
  for (auto & item : resident_tiles_) {
    if (item.second.dirty) {
      if (!saveTile(item.first, item.second, error)) {
        return false;
      }
    } else {
      refreshMetadata(item.first, item.second);
    }
  }
  return writeManifest(error);
}

bool CampusMapNode::writeManifest(std::string & error) const
{
  const auto final_path = session_directory_ / "manifest.yaml";
  auto temporary_path = final_path;
  temporary_path += ".tmp";
  std::ofstream stream(temporary_path, std::ios::trunc);
  if (!stream) {
    error = "Could not open manifest for writing: " + temporary_path.string();
    return false;
  }

  bool has_bounds = false;
  double minimum_x = 0.0;
  double minimum_y = 0.0;
  double minimum_z = 0.0;
  double maximum_x = 0.0;
  double maximum_y = 0.0;
  double maximum_z = 0.0;
  std::vector<TileKey> sorted_keys;
  sorted_keys.reserve(tile_catalog_.size());
  for (const auto & item : tile_catalog_) {
    sorted_keys.push_back(item.first);
    const TileMetadata & metadata = item.second;
    if (!metadata.has_bounds) {
      continue;
    }
    if (!has_bounds) {
      has_bounds = true;
      minimum_x = metadata.min_x;
      minimum_y = metadata.min_y;
      minimum_z = metadata.min_z;
      maximum_x = metadata.max_x;
      maximum_y = metadata.max_y;
      maximum_z = metadata.max_z;
    } else {
      minimum_x = std::min(minimum_x, metadata.min_x);
      minimum_y = std::min(minimum_y, metadata.min_y);
      minimum_z = std::min(minimum_z, metadata.min_z);
      maximum_x = std::max(maximum_x, metadata.max_x);
      maximum_y = std::max(maximum_y, metadata.max_y);
      maximum_z = std::max(maximum_z, metadata.max_z);
    }
  }
  std::sort(sorted_keys.begin(), sorted_keys.end(), [](const TileKey & left, const TileKey & right) {
    return left.x == right.x ? left.y < right.y : left.x < right.x;
  });

  stream << "format: super_odometry_campus_map\n";
  stream << "format_version: " << kTileFormatVersion << "\n";
  stream << "session: " << yamlQuote(session_name_) << "\n";
  stream << "started_utc: " << yamlQuote(session_start_utc_) << "\n";
  stream << "updated_utc: " << yamlQuote(utcTimestamp("%Y-%m-%dT%H:%M:%SZ")) << "\n";
  stream << "map_frame: " << yamlQuote(map_frame_) << "\n";
  stream << std::setprecision(12);
  stream << "voxel_size: " << voxel_size_ << "\n";
  stream << "tile_size: " << tile_size_ << "\n";
  stream << "parameters:\n";
  stream << "  input_cloud_topic: " << yamlQuote(input_cloud_topic_) << "\n";
  stream << "  odometry_topic: " << yamlQuote(odometry_topic_) << "\n";
  stream << "  min_insertion_range: " << min_insertion_range_ << "\n";
  stream << "  max_insertion_range: " << max_insertion_range_ << "\n";
  stream << "  scan_stride: " << scan_stride_ << "\n";
  stream << "  max_observations_per_voxel: " << max_observations_per_voxel_ << "\n";
  stream << "  local_publish_radius: " << local_publish_radius_ << "\n";
  stream << "  resident_radius: " << resident_radius_ << "\n";
  stream << "  eviction_radius: " << eviction_radius_ << "\n";
  stream << "  max_resident_tiles: " << max_resident_tiles_ << "\n";
  stream << "  max_pending_scans: " << max_pending_scans_ << "\n";
  stream << "  publish_period: " << publish_period_ << "\n";
  stream << "  checkpoint_period: " << checkpoint_period_ << "\n";
  stream << "point_count: " << totalPointCount() << "\n";
  stream << "tile_count: " << tile_catalog_.size() << "\n";
  stream << "processed_scans: " << processed_scans_.load() << "\n";
  stream << "dropped_scans: " << dropped_scans_.load() << "\n";
  stream << "bounds:\n";
  stream << "  valid: " << (has_bounds ? "true" : "false") << "\n";
  if (has_bounds) {
    stream << "  min: [" << minimum_x << ", " << minimum_y << ", " << minimum_z << "]\n";
    stream << "  max: [" << maximum_x << ", " << maximum_y << ", " << maximum_z << "]\n";
  }
  stream << "tiles:\n";
  for (const TileKey & key : sorted_keys) {
    const TileMetadata & metadata = tile_catalog_.at(key);
    stream << "  - index: [" << key.x << ", " << key.y << "]\n";
    stream << "    point_count: " << metadata.point_count << "\n";
    stream << "    file: " << yamlQuote("tiles/" + tilePath(key).filename().string()) << "\n";
  }
  stream.close();
  if (!stream) {
    error = "Could not finish writing manifest: " + temporary_path.string();
    return false;
  }

  std::error_code rename_error;
  std::filesystem::rename(temporary_path, final_path, rename_error);
  if (rename_error) {
    error = "Could not atomically replace manifest: " + rename_error.message();
    return false;
  }
  return true;
}

bool CampusMapNode::exportPcd(std::string & error)
{
  if (!flushAllTiles(error)) {
    return false;
  }
  const auto final_path = session_directory_ / "map.pcd";
  auto temporary_path = final_path;
  temporary_path += ".tmp";
  std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    error = "Could not open PCD export for writing: " + temporary_path.string();
    return false;
  }

  const std::uint64_t expected_points = totalPointCount();
  stream << "# .PCD v0.7 - Point Cloud Data file format\n";
  stream << "VERSION 0.7\n";
  stream << "FIELDS x y z intensity\n";
  stream << "SIZE 4 4 4 4\n";
  stream << "TYPE F F F F\n";
  stream << "COUNT 1 1 1 1\n";
  stream << "WIDTH " << expected_points << "\n";
  stream << "HEIGHT 1\n";
  stream << "VIEWPOINT 0 0 0 1 0 0 0\n";
  stream << "POINTS " << expected_points << "\n";
  stream << "DATA binary\n";

  std::vector<TileKey> sorted_keys;
  sorted_keys.reserve(tile_catalog_.size());
  for (const auto & item : tile_catalog_) {
    sorted_keys.push_back(item.first);
  }
  std::sort(sorted_keys.begin(), sorted_keys.end(), [](const TileKey & left, const TileKey & right) {
    return left.x == right.x ? left.y < right.y : left.x < right.x;
  });

  std::uint64_t written_points = 0;
  for (const TileKey & key : sorted_keys) {
    Tile loaded_tile;
    const Tile * tile = nullptr;
    const auto resident = resident_tiles_.find(key);
    if (resident != resident_tiles_.end()) {
      tile = &resident->second;
    } else {
      if (!loadTile(key, loaded_tile, error)) {
        return false;
      }
      tile = &loaded_tile;
    }
    for (const auto & item : tile->voxels) {
      const VoxelAccumulator & voxel = item.second;
      if (!writeBinary(stream, voxel.x) || !writeBinary(stream, voxel.y) ||
        !writeBinary(stream, voxel.z) || !writeBinary(stream, voxel.intensity))
      {
        error = "Could not write PCD point data: " + temporary_path.string();
        return false;
      }
      ++written_points;
    }
  }
  stream.close();
  if (!stream || written_points != expected_points) {
    error = "PCD point count changed during export";
    return false;
  }

  std::error_code rename_error;
  std::filesystem::rename(temporary_path, final_path, rename_error);
  if (rename_error) {
    error = "Could not atomically replace PCD export: " + rename_error.message();
    return false;
  }
  return true;
}

void CampusMapNode::refreshMetadata(const TileKey & key, const Tile & tile)
{
  TileMetadata & metadata = tile_catalog_[key];
  metadata = TileMetadata{};
  metadata.point_count = tile.voxels.size();
  for (const auto & item : tile.voxels) {
    const VoxelAccumulator & voxel = item.second;
    if (!metadata.has_bounds) {
      metadata.has_bounds = true;
      metadata.min_x = metadata.max_x = voxel.x;
      metadata.min_y = metadata.max_y = voxel.y;
      metadata.min_z = metadata.max_z = voxel.z;
    } else {
      metadata.min_x = std::min(metadata.min_x, static_cast<double>(voxel.x));
      metadata.min_y = std::min(metadata.min_y, static_cast<double>(voxel.y));
      metadata.min_z = std::min(metadata.min_z, static_cast<double>(voxel.z));
      metadata.max_x = std::max(metadata.max_x, static_cast<double>(voxel.x));
      metadata.max_y = std::max(metadata.max_y, static_cast<double>(voxel.y));
      metadata.max_z = std::max(metadata.max_z, static_cast<double>(voxel.z));
    }
  }
}

CampusMapNode::TileKey CampusMapNode::tileKeyForVoxel(const VoxelKey & voxel) const
{
  const double center_x = (static_cast<double>(voxel.x) + 0.5) * voxel_size_;
  const double center_y = (static_cast<double>(voxel.y) + 0.5) * voxel_size_;
  return TileKey{
    static_cast<std::int32_t>(std::floor(center_x / tile_size_)),
    static_cast<std::int32_t>(std::floor(center_y / tile_size_))};
}

double CampusMapNode::distanceToTile(double x, double y, const TileKey & key) const
{
  const double minimum_x = static_cast<double>(key.x) * tile_size_;
  const double minimum_y = static_cast<double>(key.y) * tile_size_;
  const double maximum_x = minimum_x + tile_size_;
  const double maximum_y = minimum_y + tile_size_;
  const double dx = x < minimum_x ? minimum_x - x : (x > maximum_x ? x - maximum_x : 0.0);
  const double dy = y < minimum_y ? minimum_y - y : (y > maximum_y ? y - maximum_y : 0.0);
  return std::hypot(dx, dy);
}

std::filesystem::path CampusMapNode::tilePath(const TileKey & key) const
{
  return tile_directory_ /
         ("tile_" + std::to_string(key.x) + "_" + std::to_string(key.y) + ".cmap");
}

bool CampusMapNode::frameMatches(const std::string & frame_id) const
{
  return normalizeFrame(frame_id) == map_frame_;
}

std::uint64_t CampusMapNode::totalPointCount() const
{
  std::uint64_t count = 0;
  for (const auto & item : tile_catalog_) {
    count += item.second.point_count;
  }
  return count;
}

}  // namespace super_odometry
