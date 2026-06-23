#pragma once

#include <chrono>
#include <limits>
#include <optional>
#include <string>

namespace ylhb_base
{

struct GgaData
{
  double latitude{std::numeric_limits<double>::quiet_NaN()};
  double longitude{std::numeric_limits<double>::quiet_NaN()};
  double altitude{std::numeric_limits<double>::quiet_NaN()};
  int quality{0};
  int satellites{0};
  double hdop{std::numeric_limits<double>::quiet_NaN()};
  std::optional<double> differential_age;
  std::string base_station_id;
};

std::optional<GgaData> parse_gga(const std::string & sentence);
std::string quality_text(int quality);

template<typename Time, typename Duration>
bool is_stale(Time last_update, Time now, Duration timeout)
{
  return now - last_update >= timeout;
}

}  // namespace ylhb_base
