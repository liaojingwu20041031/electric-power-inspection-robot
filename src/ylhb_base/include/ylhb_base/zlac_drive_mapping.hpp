#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace ylhb_base
{
struct LogicalWheelRpm
{
  double left = 0.0;
  double right = 0.0;
};

struct ChannelRpm
{
  double low = 0.0;
  double high = 0.0;
};

struct ChannelFaults
{
  uint16_t low = 0;
  uint16_t high = 0;
};

struct LogicalFaults
{
  uint16_t left = 0;
  uint16_t right = 0;
};

class DifferentialDriveKinematics
{
public:
  DifferentialDriveKinematics(
    double wheel_radius, double wheel_track, double max_linear_speed,
    double max_angular_speed)
  : wheel_radius_(wheel_radius),
    wheel_track_(wheel_track),
    max_linear_speed_(max_linear_speed),
    max_angular_speed_(max_angular_speed)
  {
  }

  LogicalWheelRpm twist_to_wheel_rpm(double linear_x, double angular_z) const
  {
    const double vx = std::clamp(linear_x, -max_linear_speed_, max_linear_speed_);
    const double wz = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);
    const double left_mps = vx - wz * wheel_track_ * 0.5;
    const double right_mps = vx + wz * wheel_track_ * 0.5;
    const double meters_per_rev = 2.0 * M_PI * wheel_radius_;
    return {left_mps / meters_per_rev * 60.0, right_mps / meters_per_rev * 60.0};
  }

  std::pair<double, double> wheel_rpm_to_twist(const LogicalWheelRpm & rpm) const
  {
    const double meters_per_rev = 2.0 * M_PI * wheel_radius_;
    const double left_mps = rpm.left / 60.0 * meters_per_rev;
    const double right_mps = rpm.right / 60.0 * meters_per_rev;
    return {(left_mps + right_mps) * 0.5, (right_mps - left_mps) / wheel_track_};
  }

private:
  double wheel_radius_;
  double wheel_track_;
  double max_linear_speed_;
  double max_angular_speed_;
};

// Converts between the ROS base_link longitudinal axis and the motor driver's
// calibrated longitudinal axis.  Some assembled chassis have both drive wheels
// mounted opposite to the driver's positive convention.  Keeping this separate
// from the left/right channel mapping preserves the verified yaw direction.
class LongitudinalAxisMapping
{
public:
  explicit LongitudinalAxisMapping(double direction)
  : direction_(normalize_direction(direction))
  {
  }

  static double normalize_direction(double value)
  {
    return value < 0.0 ? -1.0 : 1.0;
  }

  std::pair<double, double> command_to_internal(double linear_x, double angular_z) const
  {
    return {direction_ * linear_x, angular_z};
  }

  std::pair<double, double> feedback_to_external(double linear_x, double angular_z) const
  {
    return {direction_ * linear_x, angular_z};
  }

private:
  double direction_;
};

class ZlacChannelMapping
{
public:
  ZlacChannelMapping(
    bool low_channel_is_left, double low_channel_direction, double high_channel_direction)
  : low_channel_is_left_(low_channel_is_left),
    low_channel_direction_(normalize_direction(low_channel_direction)),
    high_channel_direction_(normalize_direction(high_channel_direction))
  {
  }

  static double normalize_direction(double value)
  {
    return value < 0.0 ? -1.0 : 1.0;
  }

  ChannelRpm logical_to_channels(const LogicalWheelRpm & logical) const
  {
    const double low_logical = low_channel_is_left_ ? logical.left : logical.right;
    const double high_logical = low_channel_is_left_ ? logical.right : logical.left;
    return {low_channel_direction_ * low_logical, high_channel_direction_ * high_logical};
  }

  LogicalWheelRpm channels_to_logical(const ChannelRpm & channels) const
  {
    const double low_logical = low_channel_direction_ * channels.low;
    const double high_logical = high_channel_direction_ * channels.high;
    if (low_channel_is_left_) {
      return {low_logical, high_logical};
    }
    return {high_logical, low_logical};
  }

  LogicalFaults channels_to_logical(const ChannelFaults & channels) const
  {
    if (low_channel_is_left_) {
      return {channels.low, channels.high};
    }
    return {channels.high, channels.low};
  }

private:
  bool low_channel_is_left_;
  double low_channel_direction_;
  double high_channel_direction_;
};
}  // namespace ylhb_base
