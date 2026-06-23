#include "ylhb_base/wtrtk980_nmea.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ylhb_base
{
namespace
{

std::vector<std::string> split_fields(const std::string & text)
{
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= text.size()) {
    const auto comma = text.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(text.substr(start));
      break;
    }
    fields.push_back(text.substr(start, comma - start));
    start = comma + 1;
  }
  return fields;
}

int hex_value(char character)
{
  if (character >= '0' && character <= '9') {
    return character - '0';
  }
  if (character >= 'A' && character <= 'F') {
    return character - 'A' + 10;
  }
  if (character >= 'a' && character <= 'f') {
    return character - 'a' + 10;
  }
  return -1;
}

bool checksum_valid(const std::string & sentence, std::size_t star)
{
  if (star + 2 >= sentence.size()) {
    return false;
  }
  const int high = hex_value(sentence[star + 1]);
  const int low = hex_value(sentence[star + 2]);
  if (high < 0 || low < 0) {
    return false;
  }

  unsigned char checksum = 0;
  for (std::size_t index = 1; index < star; ++index) {
    checksum ^= static_cast<unsigned char>(sentence[index]);
  }
  return checksum == static_cast<unsigned char>((high << 4) | low);
}

double parse_coordinate(
  const std::string & value, const std::string & hemisphere, bool latitude)
{
  if (value.empty() || hemisphere.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double raw = std::stod(value);
  const double degrees = std::floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  const double maximum_degrees = latitude ? 90.0 : 180.0;
  if (degrees > maximum_degrees || minutes >= 60.0) {
    throw std::out_of_range("invalid NMEA coordinate");
  }

  double decimal = degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    decimal = -decimal;
  } else if (hemisphere != "N" && hemisphere != "E") {
    throw std::invalid_argument("invalid NMEA hemisphere");
  }
  return decimal;
}

double optional_double_or_nan(const std::string & value)
{
  return value.empty() ? std::numeric_limits<double>::quiet_NaN() : std::stod(value);
}

}  // namespace

std::optional<GgaData> parse_gga(const std::string & raw_sentence)
{
  std::string sentence = raw_sentence;
  while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n')) {
    sentence.pop_back();
  }

  if (sentence.empty() || sentence.front() != '$') {
    return std::nullopt;
  }
  const auto star = sentence.find('*');
  if (star == std::string::npos || !checksum_valid(sentence, star)) {
    return std::nullopt;
  }

  const auto fields = split_fields(sentence.substr(1, star - 1));
  if (fields.size() < 15 || (fields[0] != "GNGGA" && fields[0] != "GPGGA")) {
    return std::nullopt;
  }

  try {
    GgaData data;
    data.quality = fields[6].empty() ? 0 : std::stoi(fields[6]);
    data.satellites = fields[7].empty() ? 0 : std::stoi(fields[7]);
    data.hdop = optional_double_or_nan(fields[8]);
    data.altitude = optional_double_or_nan(fields[9]);
    data.differential_age =
      fields[13].empty() ? std::nullopt : std::optional<double>(std::stod(fields[13]));
    data.base_station_id = fields[14];

    if (!fields[2].empty() || !fields[4].empty()) {
      data.latitude = parse_coordinate(fields[2], fields[3], true);
      data.longitude = parse_coordinate(fields[4], fields[5], false);
    }
    return data;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::string quality_text(int quality)
{
  switch (quality) {
    case 0:
      return "no fix";
    case 1:
      return "single point";
    case 2:
      return "DGPS";
    case 4:
      return "RTK fixed";
    case 5:
      return "RTK float";
    default:
      return "quality " + std::to_string(quality);
  }
}

}  // namespace ylhb_base
