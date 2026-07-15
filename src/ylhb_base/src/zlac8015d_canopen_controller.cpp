#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "ylhb_base/zlac_drive_mapping.hpp"

using namespace std::chrono_literals;
using ylhb_base::ChannelFaults;
using ylhb_base::ChannelRpm;
using ylhb_base::DifferentialDriveKinematics;
using ylhb_base::LogicalWheelRpm;
using ylhb_base::ZlacChannelMapping;

namespace
{
constexpr uint16_t kControlWord = 0x6040;
constexpr uint16_t kStatusWord = 0x6041;
constexpr uint16_t kModesOfOperation = 0x6060;
constexpr uint16_t kActualVelocity = 0x606C;
constexpr uint16_t kTargetVelocity = 0x60FF;
constexpr uint16_t kProfileAcceleration = 0x6083;
constexpr uint16_t kProfileDeceleration = 0x6084;
constexpr uint16_t kFaultCode = 0x603F;
constexpr uint16_t kVendorControlMode = 0x200F;

std::string frame_to_string(const can_frame & frame)
{
  std::ostringstream out;
  out << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << frame.can_id << "#";
  for (int i = 0; i < frame.can_dlc; ++i) {
    out << std::setw(2) << static_cast<int>(frame.data[i]);
  }
  return out.str();
}

std::string fault_text(uint16_t code)
{
  switch (code) {
    case 0x0000: return "no fault";
    case 0x0001: return "over voltage";
    case 0x0002: return "under voltage";
    case 0x0100: return "EEPROM read/write fault";
    case 0x0004: return "over current";
    case 0x0008: return "over load";
    case 0x0020: return "encoder deviation";
    case 0x0040: return "speed deviation";
    case 0x0080: return "reference voltage fault";
    case 0x0200: return "hall fault";
    case 0x0400: return "motor over temperature";
    case 0x0800: return "encoder fault";
    case 0x2000: return "target speed fault";
    default: break;
  }
  std::ostringstream out;
  out << "vendor fault 0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << code;
  return out.str();
}
}  // namespace

class Zlac8015DCanopenClient
{
public:
  struct SdoResponse
  {
    uint16_t index = 0;
    uint8_t subindex = 0;
    uint8_t command = 0;
    uint32_t raw = 0;
    uint8_t size = 0;
  };

  enum class EventType
  {
    Heartbeat,
    SdoResponse
  };

  struct CanEvent
  {
    EventType type = EventType::SdoResponse;
    uint8_t heartbeat_state = 0;
    SdoResponse sdo {};
  };

  Zlac8015DCanopenClient(
    const std::string & interface_name, uint8_t node_id, int sdo_timeout_ms,
    double target_velocity_unit_per_rpm, double actual_velocity_unit_per_rpm,
    rclcpp::Logger logger)
  : interface_name_(interface_name),
    node_id_(node_id),
    sdo_timeout_ms_(sdo_timeout_ms),
    target_velocity_unit_per_rpm_(target_velocity_unit_per_rpm),
    actual_velocity_unit_per_rpm_(actual_velocity_unit_per_rpm),
    logger_(logger)
  {
  }

  ~Zlac8015DCanopenClient()
  {
    close_socket();
  }

  bool open_socket()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(logger_, "Failed to create SocketCAN socket: %s", std::strerror(errno));
      return false;
    }

    ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(logger_, "CAN interface %s is not available: %s",
        interface_name_.c_str(), std::strerror(errno));
      close_socket();
      return false;
    }

    sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(logger_, "Failed to bind %s: %s", interface_name_.c_str(), std::strerror(errno));
      close_socket();
      return false;
    }
    return true;
  }

  bool is_open() const { return socket_fd_ >= 0; }

  void close_socket()
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  bool send_nmt_start()
  {
    can_frame frame {};
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = node_id_;
    return send_frame(frame);
  }

  bool write_u8(uint16_t index, uint8_t subindex, uint8_t value)
  {
    return send_sdo_write(index, subindex, 0x2F, value);
  }

  bool write_u16(uint16_t index, uint8_t subindex, uint16_t value)
  {
    return send_sdo_write(index, subindex, 0x2B, value);
  }

  bool write_i32(uint16_t index, uint8_t subindex, int32_t value)
  {
    return send_sdo_write(index, subindex, 0x23, static_cast<uint32_t>(value));
  }

  bool request_read(uint16_t index, uint8_t subindex)
  {
    can_frame frame {};
    frame.can_id = 0x600 + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    return send_frame(frame);
  }

  bool write_target_velocity(double low_channel_rpm, double high_channel_rpm)
  {
    const uint16_t low = static_cast<uint16_t>(rpm_to_target_units(low_channel_rpm));
    const uint16_t high = static_cast<uint16_t>(rpm_to_target_units(high_channel_rpm));
    const uint32_t packed = static_cast<uint32_t>(low) | (static_cast<uint32_t>(high) << 16);
    return write_i32(kTargetVelocity, 0x03, static_cast<int32_t>(packed));
  }

  bool initialize(bool fault_reset_on_start, int profile_acceleration, int profile_deceleration)
  {
    if (fault_reset_on_start && !write_and_wait_u16(kControlWord, 0x00, 0x0080)) {
      return false;
    }
    if (!send_nmt_start()) {
      return false;
    }
    rclcpp::sleep_for(50ms);

    return write_and_wait_u16(kVendorControlMode, 0x00, 1) &&
           write_and_wait_u8(kModesOfOperation, 0x00, 3) &&
           write_and_wait_i32(kProfileAcceleration, 0x01, profile_acceleration) &&
           write_and_wait_i32(kProfileAcceleration, 0x02, profile_acceleration) &&
           write_and_wait_i32(kProfileDeceleration, 0x01, profile_deceleration) &&
           write_and_wait_i32(kProfileDeceleration, 0x02, profile_deceleration) &&
           write_and_wait_u16(kControlWord, 0x00, 0x0006) &&
           write_and_wait_u16(kControlWord, 0x00, 0x0007) &&
           write_and_wait_u16(kControlWord, 0x00, 0x000F);
  }

  std::optional<CanEvent> poll_once()
  {
    if (socket_fd_ < 0) {
      return std::nullopt;
    }

    can_frame frame {};
    const ssize_t n = read(socket_fd_, &frame, sizeof(frame));
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "CAN read failed: %s", std::strerror(errno));
      }
      return std::nullopt;
    }
    if (n != sizeof(frame)) {
      return std::nullopt;
    }

    const canid_t cob_id = frame.can_id & CAN_SFF_MASK;
    if (cob_id == static_cast<canid_t>(0x700 + node_id_)) {
      heartbeat_state_ = frame.can_dlc > 0 ? frame.data[0] : 0;
      heartbeat_seen_ = true;
      CanEvent event;
      event.type = EventType::Heartbeat;
      event.heartbeat_state = heartbeat_state_;
      return event;
    }
    if (cob_id != static_cast<canid_t>(0x580 + node_id_) || frame.can_dlc < 8) {
      RCLCPP_DEBUG(logger_, "Ignoring CAN frame %s", frame_to_string(frame).c_str());
      return std::nullopt;
    }
    if (frame.data[0] == 0x80) {
      const uint16_t index = frame.data[1] | (frame.data[2] << 8);
      const uint8_t subindex = frame.data[3];
      const uint32_t abort_code = frame.data[4] | (frame.data[5] << 8) |
        (frame.data[6] << 16) | (frame.data[7] << 24);
      RCLCPP_ERROR(logger_, "SDO abort 0x%04X:%02X code=0x%08X", index, subindex, abort_code);
      CanEvent event;
      event.type = EventType::SdoResponse;
      event.sdo = SdoResponse{index, subindex, frame.data[0], abort_code, 4};
      return event;
    }

    const uint16_t index = frame.data[1] | (frame.data[2] << 8);
    const uint8_t subindex = frame.data[3];
    const uint32_t raw = frame.data[4] | (frame.data[5] << 8) |
      (frame.data[6] << 16) | (frame.data[7] << 24);
    const uint8_t size = response_size(frame.data[0]);
    CanEvent event;
    event.type = EventType::SdoResponse;
    event.sdo = SdoResponse{index, subindex, frame.data[0], raw, size};
    return event;
  }

  void set_clock(rclcpp::Clock::SharedPtr clock) { clock_ = clock; }

  std::optional<std::pair<double, double>> decode_actual_rpm(const SdoResponse & response) const
  {
    if (response.index != kActualVelocity || response.subindex != 0x03) {
      return std::nullopt;
    }
    const int16_t low_raw = static_cast<int16_t>(response.raw & 0xFFFF);
    const int16_t high_raw = static_cast<int16_t>((response.raw >> 16) & 0xFFFF);
    return std::make_pair(
      static_cast<double>(low_raw) / actual_velocity_unit_per_rpm_,
      static_cast<double>(high_raw) / actual_velocity_unit_per_rpm_);
  }

  std::optional<uint32_t> decode_fault(const SdoResponse & response) const
  {
    if (response.index != kFaultCode || response.subindex != 0x00) {
      return std::nullopt;
    }
    return response.raw;
  }

  bool heartbeat_seen() const { return heartbeat_seen_; }
  uint8_t heartbeat_state() const { return heartbeat_state_; }

private:
  bool send_frame(const can_frame & frame)
  {
    if (socket_fd_ < 0) {
      return false;
    }
    const ssize_t n = write(socket_fd_, &frame, sizeof(frame));
    if (n != sizeof(frame)) {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 2000, "CAN write failed: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  bool send_sdo_write(uint16_t index, uint8_t subindex, uint8_t command, uint32_t value)
  {
    can_frame frame {};
    frame.can_id = 0x600 + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = command;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = (value >> 16) & 0xFF;
    frame.data[7] = (value >> 24) & 0xFF;
    return send_frame(frame);
  }

  bool write_and_wait_u8(uint16_t index, uint8_t subindex, uint8_t value)
  {
    return write_u8(index, subindex, value) && wait_for_ack(index, subindex);
  }

  bool write_and_wait_u16(uint16_t index, uint8_t subindex, uint16_t value)
  {
    return write_u16(index, subindex, value) && wait_for_ack(index, subindex);
  }

  bool write_and_wait_i32(uint16_t index, uint8_t subindex, int32_t value)
  {
    return write_i32(index, subindex, value) && wait_for_ack(index, subindex);
  }

  bool wait_for_ack(uint16_t index, uint8_t subindex)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count() < sdo_timeout_ms_)
    {
      auto response = poll_once();
      if (response && response->type == EventType::SdoResponse &&
        response->sdo.index == index && response->sdo.subindex == subindex)
      {
        return response->sdo.command == 0x60;
      }
      rclcpp::sleep_for(2ms);
    }
    RCLCPP_ERROR(logger_, "Timeout waiting SDO ack 0x%04X:%02X", index, subindex);
    return false;
  }

  int32_t rpm_to_target_units(double rpm) const
  {
    const double scaled = std::round(rpm * target_velocity_unit_per_rpm_);
    return static_cast<int32_t>(std::clamp(
      scaled, static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max())));
  }

  uint8_t response_size(uint8_t command) const
  {
    if (command == 0x4F) {
      return 1;
    }
    if (command == 0x4B) {
      return 2;
    }
    if (command == 0x43) {
      return 4;
    }
    if (command == 0x60) {
      return 0;
    }
    return 4;
  }

  std::string interface_name_;
  uint8_t node_id_;
  int sdo_timeout_ms_;
  double target_velocity_unit_per_rpm_;
  double actual_velocity_unit_per_rpm_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  int socket_fd_ = -1;
  bool heartbeat_seen_ = false;
  uint8_t heartbeat_state_ = 0;
};

class OdometryIntegrator
{
public:
  OdometryIntegrator(const std::string & odom_frame, const std::string & base_frame)
  : odom_frame_(odom_frame), base_frame_(base_frame)
  {
  }

  nav_msgs::msg::Odometry update(const rclcpp::Time & stamp, double vx, double wz)
  {
    if (last_time_.nanoseconds() == 0) {
      last_time_ = stamp;
    }
    const double dt = std::max(0.0, (stamp - last_time_).seconds());
    last_time_ = stamp;

    x_ += vx * std::cos(yaw_) * dt;
    y_ += vx * std::sin(yaw_) * dt;
    yaw_ += wz * dt;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = wz;

    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 1e6;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 0.01;
    return odom;
  }

private:
  std::string odom_frame_;
  std::string base_frame_;
  rclcpp::Time last_time_;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

class Zlac8015DCanopenController : public rclcpp::Node
{
public:
  Zlac8015DCanopenController()
  : Node("zlac8015d_canopen_controller")
  {
    declare_parameters();
    load_parameters();

    kinematics_ = std::make_unique<DifferentialDriveKinematics>(
      wheel_radius_, wheel_track_, max_linear_speed_, max_angular_speed_);
    channel_mapping_ = std::make_unique<ZlacChannelMapping>(
      low_channel_is_left_, low_channel_direction_, high_channel_direction_);
    odom_integrator_ = std::make_unique<OdometryIntegrator>(odom_frame_, base_frame_);
    client_ = std::make_unique<Zlac8015DCanopenClient>(
      can_interface_, static_cast<uint8_t>(node_id_), sdo_timeout_ms_,
      target_velocity_unit_per_rpm_, actual_velocity_unit_per_rpm_, get_logger());
    client_->set_clock(get_clock());

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&Zlac8015DCanopenController::cmd_vel_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Zlac8015DCanopenController::scan_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("zlac8015d/status", 10);
    fault_pub_ = create_publisher<std_msgs::msg::String>("zlac8015d/fault", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (use_pdo_) {
      RCLCPP_WARN(get_logger(), "use_pdo is reserved for a later version; this node uses SDO only.");
    }

    online_ = client_->open_socket();
    if (online_) {
      online_ = client_->initialize(fault_reset_on_start_, profile_acceleration_, profile_deceleration_);
    }
    if (!online_) {
      RCLCPP_ERROR(get_logger(),
        "ZLAC8015D backend is offline. Check %s and run scripts/setup_zlac_can.sh.",
        can_interface_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "ZLAC8015D CANopen backend started on %s node_id=%d",
        can_interface_.c_str(), node_id_);
    }

    last_cmd_time_ = now();
    create_timers();
  }

  ~Zlac8015DCanopenController() override
  {
    if (stop_on_exit_ && client_ && client_->is_open()) {
      client_->write_target_velocity(0.0, 0.0);
      rclcpp::sleep_for(50ms);
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<bool>("require_fresh_scan", false);
    declare_parameter<double>("scan_timeout_sec", 0.3);
    declare_parameter<int>("node_id", 1);
    declare_parameter<double>("wheel_radius", 0.076);
    declare_parameter<double>("wheel_diameter", 0.0);
    declare_parameter<double>("wheel_track", 0.25);
    declare_parameter<double>("max_linear_speed", 0.6);
    declare_parameter<double>("max_angular_speed", 1.2);
    declare_parameter<double>("max_linear_velocity", 0.0);
    declare_parameter<double>("max_angular_velocity", 0.0);
    declare_parameter<bool>("low_channel_is_left", false);
    declare_parameter<double>("low_channel_direction", -1.0);
    declare_parameter<double>("high_channel_direction", 1.0);
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<bool>("fault_reset_on_start", false);
    declare_parameter<bool>("stop_on_exit", true);
    declare_parameter<bool>("use_pdo", false);
    declare_parameter<double>("control_rate_hz", 50.0);
    declare_parameter<double>("feedback_rate_hz", 30.0);
    declare_parameter<double>("fault_check_rate_hz", 2.0);
    declare_parameter<double>("can_poll_rate_hz", 200.0);
    declare_parameter<double>("status_rate_hz", 1.0);
    declare_parameter<double>("cmd_timeout_sec", 0.5);
    declare_parameter<int>("sdo_timeout_ms", 200);
    declare_parameter<double>("target_velocity_unit_per_rpm", 1.0);
    declare_parameter<double>("actual_velocity_unit_per_rpm", 10.0);
    declare_parameter<int>("profile_acceleration", 300);
    declare_parameter<int>("profile_deceleration", 300);
    declare_parameter<int>("acceleration_rpm_per_sec", 0);
    declare_parameter<int>("deceleration_rpm_per_sec", 0);
    declare_parameter<std::string>("protocol_docs_dir", "/home/nvidia/ros2_DL/官方通信协议");
  }

  void load_parameters()
  {
    get_parameter("can_interface", can_interface_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("scan_topic", scan_topic_);
    get_parameter("require_fresh_scan", require_fresh_scan_);
    get_parameter("scan_timeout_sec", scan_timeout_sec_);
    get_parameter("node_id", node_id_);
    get_parameter("wheel_radius", wheel_radius_);
    if (!has_parameter_override("wheel_radius")) {
      double wheel_diameter = 0.0;
      get_parameter("wheel_diameter", wheel_diameter);
      if (wheel_diameter > 0.0) {
        wheel_radius_ = wheel_diameter * 0.5;
      }
    }
    get_parameter("wheel_track", wheel_track_);
    get_parameter("max_linear_speed", max_linear_speed_);
    if (!has_parameter_override("max_linear_speed")) {
      double max_linear_velocity = 0.0;
      get_parameter("max_linear_velocity", max_linear_velocity);
      if (max_linear_velocity > 0.0) {
        max_linear_speed_ = max_linear_velocity;
      }
    }
    get_parameter("max_angular_speed", max_angular_speed_);
    if (!has_parameter_override("max_angular_speed")) {
      double max_angular_velocity = 0.0;
      get_parameter("max_angular_velocity", max_angular_velocity);
      if (max_angular_velocity > 0.0) {
        max_angular_speed_ = max_angular_velocity;
      }
    }
    get_parameter("low_channel_is_left", low_channel_is_left_);
    get_parameter("low_channel_direction", low_channel_direction_);
    get_parameter("high_channel_direction", high_channel_direction_);
    normalize_channel_direction("low_channel_direction", low_channel_direction_);
    normalize_channel_direction("high_channel_direction", high_channel_direction_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("publish_tf", publish_tf_);
    get_parameter("fault_reset_on_start", fault_reset_on_start_);
    get_parameter("stop_on_exit", stop_on_exit_);
    get_parameter("use_pdo", use_pdo_);
    get_parameter("control_rate_hz", control_rate_hz_);
    get_parameter("feedback_rate_hz", feedback_rate_hz_);
    get_parameter("fault_check_rate_hz", fault_check_rate_hz_);
    get_parameter("can_poll_rate_hz", can_poll_rate_hz_);
    get_parameter("status_rate_hz", status_rate_hz_);
    get_parameter("cmd_timeout_sec", cmd_timeout_sec_);
    get_parameter("sdo_timeout_ms", sdo_timeout_ms_);
    get_parameter("target_velocity_unit_per_rpm", target_velocity_unit_per_rpm_);
    get_parameter("actual_velocity_unit_per_rpm", actual_velocity_unit_per_rpm_);
    get_parameter("profile_acceleration", profile_acceleration_);
    if (!has_parameter_override("profile_acceleration")) {
      int acceleration_rpm_per_sec = 0;
      get_parameter("acceleration_rpm_per_sec", acceleration_rpm_per_sec);
      if (acceleration_rpm_per_sec > 0) {
        profile_acceleration_ = acceleration_rpm_per_sec;
      }
    }
    get_parameter("profile_deceleration", profile_deceleration_);
    if (!has_parameter_override("profile_deceleration")) {
      int deceleration_rpm_per_sec = 0;
      get_parameter("deceleration_rpm_per_sec", deceleration_rpm_per_sec);
      if (deceleration_rpm_per_sec > 0) {
        profile_deceleration_ = deceleration_rpm_per_sec;
      }
    }
    get_parameter("protocol_docs_dir", protocol_docs_dir_);
  }

  bool has_parameter_override(const std::string & name)
  {
    const auto overrides = get_node_parameters_interface()->get_parameter_overrides();
    return overrides.find(name) != overrides.end();
  }

  void normalize_channel_direction(const char * name, double & value)
  {
    const double requested = value;
    value = ZlacChannelMapping::normalize_direction(value);
    if (requested != value) {
      RCLCPP_WARN(get_logger(), "%s must be +1.0 or -1.0; normalized %.3f to %.1f",
        name, requested, value);
    }
  }

  void create_timers()
  {
    can_poll_timer_ = create_wall_timer(period_from_hz(can_poll_rate_hz_),
      std::bind(&Zlac8015DCanopenController::poll_can, this));
    feedback_timer_ = create_wall_timer(period_from_hz(feedback_rate_hz_),
      std::bind(&Zlac8015DCanopenController::request_feedback, this));
    fault_timer_ = create_wall_timer(period_from_hz(fault_check_rate_hz_),
      std::bind(&Zlac8015DCanopenController::request_fault, this));
    watchdog_timer_ = create_wall_timer(period_from_hz(control_rate_hz_),
      std::bind(&Zlac8015DCanopenController::watchdog, this));
    status_timer_ = create_wall_timer(period_from_hz(status_rate_hz_),
      std::bind(&Zlac8015DCanopenController::publish_status, this));
  }

  std::chrono::milliseconds period_from_hz(double hz) const
  {
    const double safe_hz = std::max(0.1, hz);
    return std::chrono::milliseconds(static_cast<int>(std::round(1000.0 / safe_hz)));
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    timed_out_ = false;
    if (!online_) {
      return;
    }
    const bool nonzero = std::abs(msg->linear.x) > 1e-6 || std::abs(msg->angular.z) > 1e-6;
    if (require_fresh_scan_ && nonzero && !scan_is_fresh()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting non-zero %s: %s is stale or missing", cmd_vel_topic_.c_str(),
        scan_topic_.c_str());
      client_->write_target_velocity(0.0, 0.0);
      return;
    }
    const auto logical_rpm =
      kinematics_->twist_to_wheel_rpm(msg->linear.x, msg->angular.z);
    const auto channels = channel_mapping_->logical_to_channels(logical_rpm);
    client_->write_target_velocity(channels.low, channels.high);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_received_sec_ = now().seconds();
    last_scan_stamp_sec_ = rclcpp::Time(msg->header.stamp).seconds();
  }

  bool scan_is_fresh() const
  {
    const double now_sec = now().seconds();
    const double stamp_age = now_sec - last_scan_stamp_sec_;
    return last_scan_received_sec_ > 0.0 && last_scan_stamp_sec_ > 0.0 &&
           now_sec - last_scan_received_sec_ <= scan_timeout_sec_ &&
           stamp_age >= -0.1 && stamp_age <= scan_timeout_sec_;
  }

  void poll_can()
  {
    if (!online_) {
      return;
    }

    for (int i = 0; i < 32; ++i) {
      auto event = client_->poll_once();
      if (!event) {
        break;
      }

      if (event->type == Zlac8015DCanopenClient::EventType::Heartbeat) {
        heartbeat_state_ = event->heartbeat_state;
        last_heartbeat_time_ = now();
        continue;
      }

      const auto & response = event->sdo;
      if (auto rpm = client_->decode_actual_rpm(response)) {
        low_channel_actual_rpm_ = rpm->first;
        high_channel_actual_rpm_ = rpm->second;
        const auto logical_rpm = channel_mapping_->channels_to_logical(
          ChannelRpm{low_channel_actual_rpm_, high_channel_actual_rpm_});
        left_actual_rpm_ = logical_rpm.left;
        right_actual_rpm_ = logical_rpm.right;
        last_feedback_time_ = now();
        publish_odom();
      }

      if (auto fault = client_->decode_fault(response)) {
        handle_fault(*fault);
      }
    }
  }

  void request_feedback()
  {
    if (!online_) {
      return;
    }
    client_->request_read(kActualVelocity, 0x03);
  }

  void request_fault()
  {
    if (!online_) {
      return;
    }
    client_->request_read(kStatusWord, 0x00);
    client_->request_read(kFaultCode, 0x00);
  }

  void watchdog()
  {
    if (!online_) {
      return;
    }
    const double age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_sec_) {
      if (!timed_out_) {
        RCLCPP_WARN(
          get_logger(), "%s timeout %.3fs; sending zero target velocity",
          cmd_vel_topic_.c_str(), age);
        timed_out_ = true;
      }
      client_->write_target_velocity(0.0, 0.0);
    }
  }

  void publish_odom()
  {
    const auto [vx, wz] =
      kinematics_->wheel_rpm_to_twist(LogicalWheelRpm{left_actual_rpm_, right_actual_rpm_});
    auto odom = odom_integrator_->update(now(), vx, wz);
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = odom.child_frame_id;
      tf.transform.translation.x = odom.pose.pose.position.x;
      tf.transform.translation.y = odom.pose.pose.position.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void handle_fault(uint32_t fault)
  {
    if (fault == 0) {
      return;
    }
    client_->write_target_velocity(0.0, 0.0);
    const auto logical_faults = channel_mapping_->channels_to_logical(ChannelFaults{
      static_cast<uint16_t>(fault & 0xFFFF),
      static_cast<uint16_t>((fault >> 16) & 0xFFFF)});
    std::ostringstream msg;
    msg << "raw=0x" << std::hex << std::uppercase
        << std::setw(8) << std::setfill('0') << fault
        << " left=0x" << std::setw(4) << logical_faults.left
        << "(" << fault_text(logical_faults.left) << ")"
        << " right=0x" << std::setw(4) << logical_faults.right
        << "(" << fault_text(logical_faults.right) << ")";
    std_msgs::msg::String out;
    out.data = msg.str();
    fault_pub_->publish(out);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "ZLAC8015D fault: %s", out.data.c_str());
  }

  void publish_status()
  {
    const auto stamp = now();
    std::string state = "offline";
    double heartbeat_age = -1.0;
    double feedback_age = -1.0;

    if (online_) {
      if (last_heartbeat_time_.nanoseconds() == 0) {
        state = "stale/offline";
      } else {
        heartbeat_age = (stamp - last_heartbeat_time_).seconds();
        if (heartbeat_age > 2.0) {
          state = "stale/offline";
        } else if (last_feedback_time_.nanoseconds() == 0) {
          state = "feedback_timeout";
        } else {
          feedback_age = (stamp - last_feedback_time_).seconds();
          state = feedback_age > 1.0 ? "feedback_timeout" : "online";
        }
      }
    }

    std::ostringstream msg;
    msg << state << " heartbeat_seen=" << (last_heartbeat_time_.nanoseconds() == 0 ? "false" : "true")
        << " heartbeat_state=0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(heartbeat_state_)
        << std::dec << " heartbeat_age_sec=" << heartbeat_age
        << " feedback_age_sec=" << feedback_age
        << std::dec << " left_rpm=" << left_actual_rpm_ << " right_rpm=" << right_actual_rpm_
        << " low_channel_rpm=" << low_channel_actual_rpm_
        << " high_channel_rpm=" << high_channel_actual_rpm_
        << " timed_out=" << (timed_out_ ? "true" : "false");
    std_msgs::msg::String out;
    out.data = msg.str();
    status_pub_->publish(out);
  }

  std::string can_interface_;
  std::string cmd_vel_topic_ = "cmd_vel";
  std::string scan_topic_ = "/scan";
  bool require_fresh_scan_ = false;
  double scan_timeout_sec_ = 0.3;
  int node_id_ = 1;
  double wheel_radius_ = 0.076;
  double wheel_track_ = 0.25;
  double max_linear_speed_ = 0.6;
  double max_angular_speed_ = 1.2;
  bool low_channel_is_left_ = false;
  double low_channel_direction_ = -1.0;
  double high_channel_direction_ = 1.0;
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_footprint";
  bool publish_tf_ = false;
  bool fault_reset_on_start_ = false;
  bool stop_on_exit_ = true;
  bool use_pdo_ = false;
  double control_rate_hz_ = 50.0;
  double feedback_rate_hz_ = 30.0;
  double fault_check_rate_hz_ = 2.0;
  double can_poll_rate_hz_ = 200.0;
  double status_rate_hz_ = 1.0;
  double cmd_timeout_sec_ = 0.5;
  double last_scan_received_sec_ = 0.0;
  double last_scan_stamp_sec_ = 0.0;
  int sdo_timeout_ms_ = 200;
  double target_velocity_unit_per_rpm_ = 1.0;
  double actual_velocity_unit_per_rpm_ = 10.0;
  int profile_acceleration_ = 300;
  int profile_deceleration_ = 300;
  std::string protocol_docs_dir_ = "/home/nvidia/ros2_DL/官方通信协议";

  bool online_ = false;
  bool timed_out_ = false;
  double left_actual_rpm_ = 0.0;
  double right_actual_rpm_ = 0.0;
  double low_channel_actual_rpm_ = 0.0;
  double high_channel_actual_rpm_ = 0.0;
  uint8_t heartbeat_state_ = 0;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_heartbeat_time_;
  rclcpp::Time last_feedback_time_;

  std::unique_ptr<DifferentialDriveKinematics> kinematics_;
  std::unique_ptr<ZlacChannelMapping> channel_mapping_;
  std::unique_ptr<Zlac8015DCanopenClient> client_;
  std::unique_ptr<OdometryIntegrator> odom_integrator_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr can_poll_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Zlac8015DCanopenController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
