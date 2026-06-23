#include "ylhb_base/wtrtk980_nmea.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace ylhb_base
{
namespace
{

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return 0;
  }
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

class Wtrtk980NmeaNode : public rclcpp::Node
{
public:
  Wtrtk980NmeaNode()
  : Node("wtrtk980_nmea_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/rtk_4g");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "gps_link");
    stale_timeout_ = declare_parameter<double>("stale_timeout", 3.0);

    fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    sentence_publisher_ = create_publisher<nmea_msgs::msg::Sentence>(
      "/gps/nmea_sentence", 10);
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/gps/rtk_status", 10);

    read_timer_ = create_wall_timer(50ms, std::bind(&Wtrtk980NmeaNode::read_serial, this));
    status_timer_ = create_wall_timer(1s, std::bind(&Wtrtk980NmeaNode::publish_stale_status, this));
    RCLCPP_INFO(
      get_logger(), "WTRTK980 NMEA reader configured for %s at %d baud",
      port_.c_str(), baud_);
  }

  ~Wtrtk980NmeaNode() override
  {
    close_serial();
  }

private:
  void close_serial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    input_buffer_.clear();
  }

  bool open_serial()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_open_attempt_) {
      return false;
    }
    next_open_attempt_ = now + 1s;

    const speed_t speed = baud_constant(baud_);
    if (speed == 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unsupported RTK baud rate: %d", baud_);
      return false;
    }

    serial_fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot open RTK port %s: %s",
        port_.c_str(), std::strerror(errno));
      return false;
    }

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      RCLCPP_WARN(get_logger(), "Cannot read RTK serial settings: %s", std::strerror(errno));
      close_serial();
      return false;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      RCLCPP_WARN(get_logger(), "Cannot configure RTK serial port: %s", std::strerror(errno));
      close_serial();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to RTK port %s", port_.c_str());
    return true;
  }

  void read_serial()
  {
    if (serial_fd_ < 0 && !open_serial()) {
      return;
    }

    char chunk[512];
    while (true) {
      const ssize_t count = read(serial_fd_, chunk, sizeof(chunk));
      if (count > 0) {
        input_buffer_.append(chunk, static_cast<std::size_t>(count));
        consume_lines();
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (count < 0) {
        RCLCPP_WARN(get_logger(), "RTK serial connection lost: %s", std::strerror(errno));
        close_serial();
      } else if (count == 0) {
        RCLCPP_WARN(get_logger(), "RTK serial connection closed");
        close_serial();
      }
      break;
    }
  }

  void consume_lines()
  {
    std::size_t newline = 0;
    while ((newline = input_buffer_.find('\n')) != std::string::npos) {
      std::string sentence = input_buffer_.substr(0, newline);
      input_buffer_.erase(0, newline + 1);
      if (!sentence.empty() && sentence.back() == '\r') {
        sentence.pop_back();
      }
      if (!sentence.empty()) {
        publish_sentence(sentence);
      }
    }
    if (input_buffer_.size() > 4096) {
      input_buffer_.clear();
    }
  }

  void publish_sentence(const std::string & sentence)
  {
    const auto gga = parse_gga(sentence);
    if (!gga) {
      return;
    }

    const auto stamp = now();
    nmea_msgs::msg::Sentence raw_message;
    raw_message.header.stamp = stamp;
    raw_message.header.frame_id = frame_id_;
    raw_message.sentence = sentence;
    sentence_publisher_->publish(raw_message);

    sensor_msgs::msg::NavSatFix fix;
    fix.header = raw_message.header;
    fix.latitude = gga->latitude;
    fix.longitude = gga->longitude;
    fix.altitude = gga->altitude;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = gga->quality == 0 ?
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    if (std::isfinite(gga->hdop)) {
      const double variance = gga->hdop * gga->hdop;
      fix.position_covariance[0] = variance;
      fix.position_covariance[4] = variance;
      fix.position_covariance[8] = variance * 4.0;
      fix.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    }
    fix_publisher_->publish(fix);

    last_gga_time_ = std::chrono::steady_clock::now();
    has_received_gga_ = true;
    last_gga_ = *gga;
    publish_status(*gga, false);
  }

  void publish_status(const GgaData & data, bool stale)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "WTRTK980 RTK";
    status.hardware_id = port_;
    status.level = stale ?
      diagnostic_msgs::msg::DiagnosticStatus::STALE :
      (data.quality == 0 ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK);
    status.message = stale ? "NMEA GGA input stale" : quality_text(data.quality);
    status.values = {
      key_value("quality", std::to_string(data.quality)),
      key_value("quality_text", quality_text(data.quality)),
      key_value("satellites", std::to_string(data.satellites)),
      key_value("hdop", std::isfinite(data.hdop) ? std::to_string(data.hdop) : ""),
      key_value(
        "differential_age",
        data.differential_age ? std::to_string(*data.differential_age) : ""),
      key_value("base_station_id", data.base_station_id),
    };
    array.status.push_back(status);
    status_publisher_->publish(array);
  }

  void publish_stale_status()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(stale_timeout_);
    if (!has_received_gga_ || is_stale(last_gga_time_, now_steady, timeout)) {
      publish_status(last_gga_, true);
    }
  }

  std::string port_;
  int baud_{115200};
  std::string frame_id_;
  double stale_timeout_{3.0};
  int serial_fd_{-1};
  std::string input_buffer_;
  bool has_received_gga_{false};
  GgaData last_gga_;
  std::chrono::steady_clock::time_point last_gga_time_{};
  std::chrono::steady_clock::time_point next_open_attempt_{};
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr sentence_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
};

}  // namespace ylhb_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ylhb_base::Wtrtk980NmeaNode>());
  rclcpp::shutdown();
  return 0;
}
