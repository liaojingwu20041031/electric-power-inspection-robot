#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>

#include "ylhb_base/wtrtk980_nmea.hpp"

namespace
{

std::string with_checksum(const std::string & payload)
{
  unsigned char checksum = 0;
  for (const char character : payload) {
    checksum ^= static_cast<unsigned char>(character);
  }

  constexpr char hex[] = "0123456789ABCDEF";
  std::string sentence = "$" + payload + "*";
  sentence.push_back(hex[(checksum >> 4) & 0x0F]);
  sentence.push_back(hex[checksum & 0x0F]);
  return sentence;
}

}  // namespace

TEST(Wtrtk980Nmea, ParsesRtkFixedGngga)
{
  const auto sentence = with_checksum(
    "GNGGA,123519,4807.038,N,01131.000,E,4,12,0.7,545.4,M,46.9,M,1.2,0042");

  const auto result = ylhb_base::parse_gga(sentence);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->latitude, 48.1173, 1e-7);
  EXPECT_NEAR(result->longitude, 11.5166666667, 1e-7);
  EXPECT_DOUBLE_EQ(result->altitude, 545.4);
  EXPECT_EQ(result->quality, 4);
  EXPECT_EQ(result->satellites, 12);
  EXPECT_DOUBLE_EQ(result->hdop, 0.7);
  ASSERT_TRUE(result->differential_age.has_value());
  EXPECT_DOUBLE_EQ(*result->differential_age, 1.2);
  EXPECT_EQ(result->base_station_id, "0042");
  EXPECT_EQ(ylhb_base::quality_text(result->quality), "RTK fixed");
}

TEST(Wtrtk980Nmea, ParsesRtkFloatGpggaWithSouthernWesternCoordinates)
{
  const auto sentence = with_checksum(
    "GPGGA,010203,3456.789,S,12345.678,W,5,09,1.1,10.5,M,0.0,M,,");

  const auto result = ylhb_base::parse_gga(sentence);

  ASSERT_TRUE(result.has_value());
  EXPECT_LT(result->latitude, 0.0);
  EXPECT_LT(result->longitude, 0.0);
  EXPECT_EQ(result->quality, 5);
  EXPECT_FALSE(result->differential_age.has_value());
  EXPECT_TRUE(result->base_station_id.empty());
  EXPECT_EQ(ylhb_base::quality_text(result->quality), "RTK float");
}

TEST(Wtrtk980Nmea, RejectsInvalidChecksumAndUnsupportedSentence)
{
  EXPECT_FALSE(
    ylhb_base::parse_gga(
      "$GNGGA,123519,4807.038,N,01131.000,E,4,12,0.7,545.4,M,46.9,M,1.2,0042*00")
    .has_value());
  EXPECT_FALSE(
    ylhb_base::parse_gga(with_checksum("GNRMC,123519,A,4807.038,N,01131.000,E"))
    .has_value());
}

TEST(Wtrtk980Nmea, PreservesEmptyNoFixGga)
{
  const auto sentence = with_checksum("GNGGA,123519,,,,,0,00,99.9,,,,,,,");

  const auto result = ylhb_base::parse_gga(sentence);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->quality, 0);
  EXPECT_FALSE(std::isfinite(result->latitude));
  EXPECT_FALSE(std::isfinite(result->longitude));
  EXPECT_EQ(ylhb_base::quality_text(result->quality), "no fix");
}

TEST(Wtrtk980Nmea, DetectsStaleInputAtTimeout)
{
  using namespace std::chrono_literals;
  EXPECT_FALSE(ylhb_base::is_stale(10s, 12s, 3s));
  EXPECT_TRUE(ylhb_base::is_stale(10s, 13s, 3s));
}
