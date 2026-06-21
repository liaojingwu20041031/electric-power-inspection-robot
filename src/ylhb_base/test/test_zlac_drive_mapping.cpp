#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <string>

#include "ylhb_base/zlac_drive_mapping.hpp"

namespace
{
using ylhb_base::ChannelFaults;
using ylhb_base::ChannelRpm;
using ylhb_base::DifferentialDriveKinematics;
using ylhb_base::LogicalFaults;
using ylhb_base::LogicalWheelRpm;
using ylhb_base::ZlacChannelMapping;

TEST(DifferentialDriveKinematicsTest, PositiveLinearVelocityProducesPositiveLogicalWheelRpm)
{
  const DifferentialDriveKinematics kinematics(0.1, 0.4, 1.0, 2.0);

  const auto rpm = kinematics.twist_to_wheel_rpm(0.5, 0.0);

  EXPECT_GT(rpm.left, 0.0);
  EXPECT_DOUBLE_EQ(rpm.left, rpm.right);
}

TEST(DifferentialDriveKinematicsTest, PositiveAngularVelocityKeepsRosLeftTurnSemantics)
{
  const DifferentialDriveKinematics kinematics(0.1, 0.4, 1.0, 2.0);

  const auto rpm = kinematics.twist_to_wheel_rpm(0.0, 1.0);

  EXPECT_LT(rpm.left, 0.0);
  EXPECT_GT(rpm.right, 0.0);
}

TEST(ZlacChannelMappingTest, FieldMappingPreservesRosTurnDirection)
{
  const ZlacChannelMapping mapping(true, 1.0, -1.0);

  const auto forward = mapping.logical_to_channels(LogicalWheelRpm{100.0, 100.0});
  const auto left_turn = mapping.logical_to_channels(LogicalWheelRpm{-100.0, 100.0});

  EXPECT_DOUBLE_EQ(forward.low, 100.0);
  EXPECT_DOUBLE_EQ(forward.high, -100.0);
  EXPECT_DOUBLE_EQ(left_turn.low, -100.0);
  EXPECT_DOUBLE_EQ(left_turn.high, -100.0);
}

TEST(ZlacChannelMappingTest, PreviousHighChannelPolarityReproducesAxisSwapSymptoms)
{
  const ZlacChannelMapping previous_mapping(false, -1.0, -1.0);
  const ZlacChannelMapping physical_calibration(false, -1.0, 1.0);

  const auto actual_forward = physical_calibration.channels_to_logical(
    previous_mapping.logical_to_channels(LogicalWheelRpm{100.0, 100.0}));
  const auto actual_reverse = physical_calibration.channels_to_logical(
    previous_mapping.logical_to_channels(LogicalWheelRpm{-100.0, -100.0}));
  const auto actual_left = physical_calibration.channels_to_logical(
    previous_mapping.logical_to_channels(LogicalWheelRpm{-100.0, 100.0}));
  const auto actual_right = physical_calibration.channels_to_logical(
    previous_mapping.logical_to_channels(LogicalWheelRpm{100.0, -100.0}));

  EXPECT_DOUBLE_EQ(actual_forward.left, -100.0);
  EXPECT_DOUBLE_EQ(actual_forward.right, 100.0);
  EXPECT_DOUBLE_EQ(actual_reverse.left, 100.0);
  EXPECT_DOUBLE_EQ(actual_reverse.right, -100.0);
  EXPECT_DOUBLE_EQ(actual_left.left, 100.0);
  EXPECT_DOUBLE_EQ(actual_left.right, 100.0);
  EXPECT_DOUBLE_EQ(actual_right.left, -100.0);
  EXPECT_DOUBLE_EQ(actual_right.right, -100.0);
}

TEST(ZlacChannelMappingTest, BaseKinematicsConfigUsesCalibratedFieldMapping)
{
  std::ifstream config_stream(BASE_KINEMATICS_CONFIG_PATH);
  ASSERT_TRUE(config_stream.is_open());
  const std::string config(
    (std::istreambuf_iterator<char>(config_stream)), std::istreambuf_iterator<char>());

  EXPECT_NE(config.find("low_channel_is_left: true"), std::string::npos);
  EXPECT_NE(config.find("low_channel_direction: 1.0"), std::string::npos);
  EXPECT_NE(config.find("high_channel_direction: -1.0"), std::string::npos);
}

TEST(ZlacChannelMappingTest, CommandAndFeedbackMappingsRoundTrip)
{
  const ZlacChannelMapping mapping(true, 1.0, -1.0);
  const LogicalWheelRpm expected{-42.5, 17.25};

  const auto actual = mapping.channels_to_logical(mapping.logical_to_channels(expected));

  EXPECT_DOUBLE_EQ(actual.left, expected.left);
  EXPECT_DOUBLE_EQ(actual.right, expected.right);
}

TEST(ZlacChannelMappingTest, LowChannelCanMapToEitherLogicalSide)
{
  const LogicalWheelRpm logical{10.0, 20.0};

  const auto low_is_left =
    ZlacChannelMapping(true, 1.0, 1.0).logical_to_channels(logical);
  const auto low_is_right =
    ZlacChannelMapping(false, 1.0, 1.0).logical_to_channels(logical);

  EXPECT_DOUBLE_EQ(low_is_left.low, 10.0);
  EXPECT_DOUBLE_EQ(low_is_left.high, 20.0);
  EXPECT_DOUBLE_EQ(low_is_right.low, 20.0);
  EXPECT_DOUBLE_EQ(low_is_right.high, 10.0);
}

TEST(ZlacChannelMappingTest, DirectionValuesNormalizeToExplicitPolarity)
{
  EXPECT_DOUBLE_EQ(ZlacChannelMapping::normalize_direction(2.0), 1.0);
  EXPECT_DOUBLE_EQ(ZlacChannelMapping::normalize_direction(-0.01), -1.0);
  EXPECT_DOUBLE_EQ(ZlacChannelMapping::normalize_direction(0.0), 1.0);
}

TEST(ZlacChannelMappingTest, FaultWordsUseTheSameLogicalSideMapping)
{
  const ChannelFaults channels{0x0004, 0x0400};

  const LogicalFaults low_is_left =
    ZlacChannelMapping(true, 1.0, 1.0).channels_to_logical(channels);
  const LogicalFaults low_is_right =
    ZlacChannelMapping(false, 1.0, 1.0).channels_to_logical(channels);

  EXPECT_EQ(low_is_left.left, 0x0004);
  EXPECT_EQ(low_is_left.right, 0x0400);
  EXPECT_EQ(low_is_right.left, 0x0400);
  EXPECT_EQ(low_is_right.right, 0x0004);
}
}  // namespace
