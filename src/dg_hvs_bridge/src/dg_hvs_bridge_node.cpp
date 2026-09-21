#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <shimetapi/codec/mipi_raw8_codec.h>
#include <shimetapi/core/event_cd.h>
#include <shimetapi/core/frame.h>
#include <shimetapi/hv/camera.h>
#include <shimetapi/hv/device_config.h>
#include <shimetapi/io/event_reader.h>
#include <shimetapi/io/event_writer.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef DG_HVS_HAS_HBN_BACKEND
#include "dg_hvs_hbn_backend.h"
#endif

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

template<typename T>
void append_scalar(std::vector<uint8_t> & data, size_t offset, const T & value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string default_record_directory()
{
  const char * home = std::getenv("HOME");
  if (home != nullptr && *home != '\0') {
    return (fs::path(home) / "dg202611_rdk_records").string();
  }
  return "dg202611_rdk_records";
}

std::string pixel_format_text(Shimeta::PixelFormat format)
{
  switch (format) {
    case Shimeta::PixelFormat::BayerRG8: return "BayerRG8";
    case Shimeta::PixelFormat::RGB888: return "RGB888";
    case Shimeta::PixelFormat::Gray8: return "Gray8";
    case Shimeta::PixelFormat::RAW8: return "RAW8";
    case Shimeta::PixelFormat::RAW10: return "RAW10";
    case Shimeta::PixelFormat::NV12: return "NV12";
  }
  return "UNKNOWN";
}

std::string session_name()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm calendar{};
  localtime_r(&time, &calendar);
  std::ostringstream stream;
  stream << std::put_time(&calendar, "%Y%m%d_%H%M%S") << "_apx003cc";
  return stream.str();
}

}  // namespace

class DgHvsBridge final : public rclcpp::Node
{
public:
  DgHvsBridge()
  : Node("dg_hvs_bridge_node")
  {
    declare_parameter<std::string>("device_node", "");
    declare_parameter<std::string>("backend", "toolkit");
    declare_parameter<int>("hbn_config_index", 1);
    declare_parameter<int>("hbn_settle_value", -1);
    declare_parameter<int>("sensor_index", 0);
    declare_parameter<int>("i2c_bus", 1);
    declare_parameter<int>("evs_fps", 240);
    declare_parameter<int>("buffer_count", 8);
    declare_parameter<std::string>("frame_id", "event_camera_optical_frame");
    declare_parameter<std::string>("camera_name", "apx003cc_hvs");
    declare_parameter<int>("event_width", 768);
    declare_parameter<int>("event_height", 608);
    declare_parameter<int>("aps_width", 768);
    declare_parameter<int>("aps_height", 608);
    declare_parameter<double>("aps_expected_fps", 30.0);
    declare_parameter<double>("pair_tolerance_ms", 33.34);
    declare_parameter<double>("stream_timeout_sec", 0.50);
    declare_parameter<double>("static_event_rate_threshold", 500.0);
    declare_parameter<double>("event_publish_rate_hz", 20.0);
    declare_parameter<int>("max_events_per_message", 20000);
    declare_parameter<double>("preview_publish_rate_hz", 10.0);
    declare_parameter<int>("jpeg_quality", 80);
    declare_parameter<double>("event_decay", 0.90);
    declare_parameter<int>("denoise_cell_px", 4);
    declare_parameter<int>("denoise_min_events_per_cell", 2);
    declare_parameter<int>("max_events_per_second", 1000000);
    declare_parameter<bool>("publish_features", false);
    declare_parameter<int>("feature_cell_px", 12);
    declare_parameter<int>("feature_min_events_per_cell", 5);
    declare_parameter<int>("feature_min_unique_pixels", 3);
    declare_parameter<int>("feature_max_features", 512);
    declare_parameter<double>("feature_match_distance_px", 12.0);
    declare_parameter<bool>("afk_filter_enabled", false);
    declare_parameter<bool>("recording_enabled", false);
    declare_parameter<std::string>("record_directory", default_record_directory());
    declare_parameter<bool>("record_csv_enabled", true);
    declare_parameter<int>("record_aps_every_n", 3);
    declare_parameter<double>("camera_fx", 0.0);
    declare_parameter<double>("camera_fy", 0.0);
    declare_parameter<double>("camera_cx", 0.0);
    declare_parameter<double>("camera_cy", 0.0);
    declare_parameter<std::string>("distortion_model", "plumb_bob");
    declare_parameter<std::vector<double>>(
      "distortion_coefficients", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    read_parameters();
    validate_parameters();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
    events_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dg/event_camera/events", sensor_qos);
    aps_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/dg/event_camera/aps/image_raw", sensor_qos);
    aps_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/dg/event_camera/aps/image_raw/compressed", sensor_qos);
    fused_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/dg/event_camera/fused/image", sensor_qos);
    fused_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/dg/event_camera/fused/image/compressed", sensor_qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/dg/event_camera/motion_mask", sensor_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/dg/event_camera/camera_info", rclcpp::QoS(1).transient_local());
    quality_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/dg/event_camera/quality", 10);
    if (publish_features_) {
      features_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/dg/event_camera/features", sensor_qos);
      feature_quality_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/dg/event_camera/feature_quality", 10);
    }

    quality_timer_ = create_wall_timer(1s, std::bind(&DgHvsBridge::publish_quality, this));
    open_recording();
    start_camera();
    try {
      capture_thread_ = std::thread([this]() noexcept {
          try {
            capture_loop();
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "EVS capture thread stopped: %s", error.what());
            running_.store(false);
          } catch (...) {
            RCLCPP_ERROR(get_logger(), "EVS capture thread stopped with an unknown error");
            running_.store(false);
          }
        });
#ifdef DG_HVS_HAS_HBN_BACKEND
      if (use_hbn_backend_) {
        aps_capture_thread_ = std::thread([this]() noexcept {
            try {
              capture_hbn_aps_loop();
            } catch (const std::exception & error) {
              RCLCPP_ERROR(get_logger(), "APS capture thread stopped: %s", error.what());
              running_.store(false);
            } catch (...) {
              RCLCPP_ERROR(get_logger(), "APS capture thread stopped with an unknown error");
              running_.store(false);
            }
          });
      }
#endif
    } catch (...) {
      running_.store(false);
      if (capture_thread_.joinable()) {
        capture_thread_.join();
      }
#ifdef DG_HVS_HAS_HBN_BACKEND
      if (hbn_handle_ != nullptr) {
        dg_hvs_hbn_close(hbn_handle_);
        hbn_handle_ = nullptr;
      } else {
        camera_.StopStream();
        camera_.Destroy();
      }
#else
      camera_.StopStream();
      camera_.Destroy();
#endif
      close_recording();
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "APX003CC MIPI-HVS bridge started: sensor_index=%d i2c_bus=%d evs_fps=%d; "
      "recording=%s; publishes_cmd_vel=false",
      sensor_index_, i2c_bus_, evs_fps_, bool_text(recording_enabled_).c_str());
  }

  ~DgHvsBridge() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
#ifdef DG_HVS_HAS_HBN_BACKEND
    if (aps_capture_thread_.joinable()) {
      aps_capture_thread_.join();
    }
#endif
#ifdef DG_HVS_HAS_HBN_BACKEND
    if (use_hbn_backend_) {
      dg_hvs_hbn_close(hbn_handle_);
      hbn_handle_ = nullptr;
    } else {
      camera_.StopStream();
      camera_.Destroy();
    }
#else
    camera_.StopStream();
    camera_.Destroy();
#endif
    close_recording();
  }

private:
  void read_parameters()
  {
    device_node_ = get_parameter("device_node").as_string();
    backend_ = get_parameter("backend").as_string();
    hbn_config_index_ = get_parameter("hbn_config_index").as_int();
    hbn_settle_value_ = get_parameter("hbn_settle_value").as_int();
    sensor_index_ = get_parameter("sensor_index").as_int();
    i2c_bus_ = get_parameter("i2c_bus").as_int();
    evs_fps_ = get_parameter("evs_fps").as_int();
    buffer_count_ = get_parameter("buffer_count").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    camera_name_ = get_parameter("camera_name").as_string();
    event_width_ = get_parameter("event_width").as_int();
    event_height_ = get_parameter("event_height").as_int();
    aps_width_ = get_parameter("aps_width").as_int();
    aps_height_ = get_parameter("aps_height").as_int();
    aps_expected_fps_ = get_parameter("aps_expected_fps").as_double();
    pair_tolerance_ms_ = get_parameter("pair_tolerance_ms").as_double();
    stream_timeout_sec_ = get_parameter("stream_timeout_sec").as_double();
    static_event_rate_threshold_ = get_parameter("static_event_rate_threshold").as_double();
    event_publish_rate_hz_ = get_parameter("event_publish_rate_hz").as_double();
    max_events_per_message_ = get_parameter("max_events_per_message").as_int();
    preview_publish_rate_hz_ = get_parameter("preview_publish_rate_hz").as_double();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    event_decay_ = get_parameter("event_decay").as_double();
    denoise_cell_px_ = get_parameter("denoise_cell_px").as_int();
    denoise_min_events_per_cell_ = get_parameter("denoise_min_events_per_cell").as_int();
    max_events_per_second_ = get_parameter("max_events_per_second").as_int();
    publish_features_ = get_parameter("publish_features").as_bool();
    feature_cell_px_ = get_parameter("feature_cell_px").as_int();
    feature_min_events_per_cell_ = get_parameter("feature_min_events_per_cell").as_int();
    feature_min_unique_pixels_ = get_parameter("feature_min_unique_pixels").as_int();
    feature_max_features_ = get_parameter("feature_max_features").as_int();
    feature_match_distance_px_ = get_parameter("feature_match_distance_px").as_double();
    afk_filter_enabled_ = get_parameter("afk_filter_enabled").as_bool();
    recording_enabled_ = get_parameter("recording_enabled").as_bool();
    record_directory_ = get_parameter("record_directory").as_string();
    if (record_directory_.empty()) {
      record_directory_ = default_record_directory();
    }
    record_csv_enabled_ = get_parameter("record_csv_enabled").as_bool();
    record_aps_every_n_ = get_parameter("record_aps_every_n").as_int();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    distortion_model_ = get_parameter("distortion_model").as_string();
    distortion_coefficients_ = get_parameter("distortion_coefficients").as_double_array();
  }

  void validate_parameters() const
  {
    const std::vector<int> supported_fps{120, 240, 300, 500, 750, 1000};
    if (sensor_index_ < 0 || i2c_bus_ < 0 || i2c_bus_ > 255 ||
      (evs_fps_ != 0 && std::find(supported_fps.begin(), supported_fps.end(), evs_fps_) == supported_fps.end()))
    {
      throw std::invalid_argument("invalid sensor_index, i2c_bus or EVS FPS");
    }
    if (event_width_ <= 0 || event_height_ <= 0 || aps_width_ <= 0 || aps_height_ <= 0 ||
      buffer_count_ <= 0 ||
      aps_expected_fps_ <= 0.0 || pair_tolerance_ms_ <= 0.0 || stream_timeout_sec_ <= 0.0 ||
      event_publish_rate_hz_ <= 0.0 || preview_publish_rate_hz_ <= 0.0 ||
      max_events_per_message_ <= 0 || max_events_per_second_ <= 0 ||
      feature_cell_px_ <= 0 || feature_min_events_per_cell_ <= 0 || feature_max_features_ <= 0 ||
      feature_min_unique_pixels_ <= 0 || !std::isfinite(feature_match_distance_px_) ||
      feature_match_distance_px_ <= 0.0 ||
      denoise_cell_px_ <= 0 || denoise_min_events_per_cell_ <= 0 ||
      record_aps_every_n_ <= 0 || jpeg_quality_ < 1 || jpeg_quality_ > 100 ||
      event_decay_ < 0.0 || event_decay_ > 1.0)
    {
      throw std::invalid_argument("invalid HVS stream, rate, denoise or recording parameter");
    }
    if (backend_ != "toolkit" && backend_ != "hbn") {
      throw std::invalid_argument("backend must be toolkit or hbn");
    }
    if (backend_ == "hbn" && hbn_config_index_ < 0) {
      throw std::invalid_argument("hbn_config_index must be non-negative");
    }
    if (frame_id_.empty() || camera_name_.empty()) {
      throw std::invalid_argument("frame_id and camera_name must be non-empty");
    }
    if (afk_filter_enabled_) {
      throw std::invalid_argument(
              "afk_filter_enabled is not exposed by the current official Toolkit API; keep it false");
    }
  }

  void start_camera()
  {
#ifdef DG_HVS_HAS_HBN_BACKEND
    if (backend_ == "hbn") {
      const int result = dg_hvs_hbn_open(
        &hbn_handle_, hbn_config_index_, sensor_index_, i2c_bus_,
        hbn_settle_value_, buffer_count_);
      if (result != 0 || hbn_handle_ == nullptr) {
        throw std::runtime_error("official RDK HBN backend failed to start");
      }
      use_hbn_backend_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Using official RDK HBN dual-VC backend: config_index=%d; requested_sensor_index=%d requested_i2c_bus=%d; actual pair is resolved by vendor config",
        hbn_config_index_, sensor_index_, i2c_bus_);
      return;
    }
#else
    if (backend_ == "hbn") {
      throw std::runtime_error(
              "hbn backend requested, but the official RDK HVS sample tree was unavailable at build time");
    }
#endif
    Shimeta::hv::DeviceConfig config;
    config.backend = Shimeta::hv::Backend::MipiHvs;
    config.device_node = device_node_;
    config.sensor_index = sensor_index_;
    config.i2c_bus = static_cast<uint8_t>(i2c_bus_);
    config.evs_fps = static_cast<uint16_t>(evs_fps_);
    config.buffer_count = buffer_count_;
    if (!camera_.Init(config)) {
      throw std::runtime_error("ShiMetaPi Camera::Init failed");
    }
    if (!camera_.StartStream()) {
      camera_.Destroy();
      throw std::runtime_error("ShiMetaPi MIPI-HVS StartStream failed");
    }
  }

  void open_recording()
  {
    if (!recording_enabled_) {
      return;
    }
    recording_session_dir_ = fs::path(record_directory_) / session_name();
    fs::create_directories(recording_session_dir_ / "aps");
    fs::create_directories(recording_session_dir_ / "fused");
    raw8_file_.open(recording_session_dir_ / "events.raw8", std::ios::binary | std::ios::out);
    if (!raw8_file_) {
      throw std::runtime_error("cannot open RAW8 recording file");
    }
    if (!evt2_writer_.open(
        (recording_session_dir_ / "events.evt2").string(), event_width_, event_height_,
        Shimeta::io::RawFormat::Evt2))
    {
      throw std::runtime_error("cannot open EVT2 recording file");
    }
    if (record_csv_enabled_) {
      csv_file_.open(recording_session_dir_ / "events.csv", std::ios::out);
      if (!csv_file_) {
        throw std::runtime_error("cannot open event CSV recording file");
      }
      // Keep the recorded CSV compatible with the documented EVT2 export and
      // auxiliary_sensor_data parser.  The timestamp remains in microseconds;
      // only the canonical column order/name is important for replay.
      csv_file_ << "x,y,polarity,timestamp\n";
    }
    std::ofstream config_file(recording_session_dir_ / "capture_config.txt");
    config_file << "camera_name=" << camera_name_ << "\n"
                << "frame_id=" << frame_id_ << "\n"
                << "sensor_index=" << sensor_index_ << "\n"
                << "i2c_bus=" << i2c_bus_ << "\n"
                << "evs_fps=" << evs_fps_ << "\n"
                << "publish_features=" << bool_text(publish_features_) << "\n"
                << "feature_cell_px=" << feature_cell_px_ << "\n"
                << "feature_min_events_per_cell=" << feature_min_events_per_cell_ << "\n"
                << "feature_min_unique_pixels=" << feature_min_unique_pixels_ << "\n"
                << "feature_match_distance_px=" << feature_match_distance_px_ << "\n"
                << "event_geometry=" << event_width_ << "x" << event_height_ << "\n"
                << "aps_fallback_geometry=" << aps_width_ << "x" << aps_height_ << "\n"
                << "pair_tolerance_ms=" << pair_tolerance_ms_ << "\n"
                << "afk_filter_enabled=false\n"
                << "publishes_cmd_vel=false\n";
  }

  void close_recording()
  {
    if (!recording_enabled_) {
      return;
    }
    evt2_writer_.close();
    raw8_file_.close();
    csv_file_.close();
    std::ofstream summary(recording_session_dir_ / "capture_summary.txt");
    summary << "evs_packets=" << evs_packet_count_.load() << "\n"
            << "decoded_events=" << decoded_event_count_.load() << "\n"
            << "aps_packets=" << aps_packet_count_.load() << "\n"
            << "aps_frames=" << aps_frame_count_.load() << "\n"
            << "fused_frames=" << fused_frame_count_.load() << "\n"
            << "skipped_fused_frames=" << skipped_fused_frame_count_.load() << "\n"
            << "aps_decode_failures=" << unsupported_aps_format_count_.load() << "\n"
            << "get_frame_timeouts=" << get_frame_timeout_count_.load() << "\n"
            << "dropped_event_messages=" << dropped_event_messages_.load() << "\n"
            << "last_state=" << stream_state_ << "\n";
  }

  void capture_loop()
  {
#ifdef DG_HVS_HAS_HBN_BACKEND
    if (use_hbn_backend_) {
      capture_hbn_loop();
      return;
    }
#endif
    const uint8_t * last_evs_pointer = nullptr;
    const uint8_t * last_aps_pointer = nullptr;
    while (running_.load() && rclcpp::ok()) {
      Shimeta::Frame frame;
      if (!camera_.GetFrame(frame, 100)) {
        get_frame_timeout_count_.fetch_add(1);
        continue;
      }
      const auto receipt_time = now();
      const auto steady_now = std::chrono::steady_clock::now();
      const bool new_evs = frame.evs.data != nullptr && frame.evs.size > 0 &&
        frame.evs.data != last_evs_pointer;
      const bool new_aps = is_new_aps_frame(frame, last_aps_pointer);

      std::vector<Shimeta::EventCD> events;
      if (new_evs) {
        last_evs_pointer = frame.evs.data;
        decoder_.Decode(frame.evs.data, frame.evs.size, events);
        denoise_and_limit(events);
        const auto extracted = Shimeta::codec::extractEvsTimestamp(frame.evs.data, frame.evs.size);
        if (extracted.valid) {
          last_evs_sensor_us_.store(extracted.processed_timestamp);
        } else if (!events.empty()) {
          last_evs_sensor_us_.store(static_cast<uint64_t>(std::max<int64_t>(0, events.front().t)));
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_evs_steady_ = steady_now;
        }
        evs_packet_count_.fetch_add(1);
        decoded_event_count_.fetch_add(events.size());
        interval_event_count_.fetch_add(events.size());
        update_event_canvas(events);
        publish_events_if_due(events, receipt_time, steady_now);
        record_events(frame, events);
      }

      if (new_aps) {
        last_aps_pointer = frame.aps.data;
        last_aps_frame_id_ = frame.frame_id;
        last_aps_ts_ns_ = frame.ts.aps_ts_ns;
        aps_packet_count_.fetch_add(1);
        cv::Mat gray = aps_to_gray(frame);
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_aps_width_ = frame.width > 0 ? frame.width : aps_width_;
          last_aps_height_ = frame.height > 0 ? frame.height : aps_height_;
          last_aps_format_ = pixel_format_text(frame.format);
          last_aps_bytes_ = frame.aps.size;
        }
        if (!gray.empty()) {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_aps_steady_ = steady_now;
          }
          aps_frame_count_.fetch_add(1);
          if (frame.ts.aps_ts_ns > 0) {
            last_aps_sensor_ns_.store(static_cast<uint64_t>(frame.ts.aps_ts_ns));
          } else if (frame.aps_evs_ts.valid) {
            last_aps_sensor_ns_.store(frame.aps_evs_ts.processed_timestamp * 1000ULL);
          }
          update_pairing(frame);
          publish_aps_and_preview_if_due(gray, receipt_time, steady_now);
        }
      }
    }
  }

#ifdef DG_HVS_HAS_HBN_BACKEND
  void capture_hbn_loop()
  {
    while (running_.load() && rclcpp::ok()) {
      dg_hvs_hbn_frame_t raw{};
      const int result = dg_hvs_hbn_get_evs_frame(hbn_handle_, 100, &raw);
      if (result != 0) {
        evs_get_frame_error_count_.fetch_add(1);
        last_evs_get_frame_error_.store(result);
        continue;
      }
      const auto receipt_time = now();
      const auto steady_now = std::chrono::steady_clock::now();

      Shimeta::Frame frame;
      frame.evs.data = raw.evs_data;
      frame.evs.size = raw.evs_size;
      frame.ts.evs_ts_ns = static_cast<int64_t>(raw.evs_timestamp_ns);

      last_evs_hbn_sensor_ns_.store(raw.evs_timestamp_ns);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_evs_steady_ = steady_now;
        if (raw.evs_timestamp_ns > 0) {
          recent_evs_hbn_timestamps_[recent_evs_hbn_write_index_] = raw.evs_timestamp_ns;
          recent_evs_hbn_write_index_ =
            (recent_evs_hbn_write_index_ + 1U) % recent_evs_hbn_timestamps_.size();
        }
      }

      std::vector<Shimeta::EventCD> events;
      decoder_.Decode(frame.evs.data, frame.evs.size, events);
      denoise_and_limit(events);
      const auto extracted = Shimeta::codec::extractEvsTimestamp(frame.evs.data, frame.evs.size);
      if (extracted.valid) {
        last_evs_sensor_us_.store(extracted.processed_timestamp);
      } else if (!events.empty()) {
        last_evs_sensor_us_.store(static_cast<uint64_t>(std::max<int64_t>(0, events.front().t)));
      }
      evs_packet_count_.fetch_add(1);
      decoded_event_count_.fetch_add(events.size());
      interval_event_count_.fetch_add(events.size());
      update_event_canvas(events);
      publish_events_if_due(events, receipt_time, steady_now);
      record_events(frame, events);
    }
  }

  void capture_hbn_aps_loop()
  {
    while (running_.load() && rclcpp::ok()) {
      dg_hvs_hbn_frame_t raw{};
      const int result = dg_hvs_hbn_get_aps_frame(hbn_handle_, 100, &raw);
      if (result != 0) {
        aps_get_frame_error_count_.fetch_add(1);
        last_aps_get_frame_error_.store(result);
        continue;
      }
      const auto receipt_time = now();
      const auto steady_now = std::chrono::steady_clock::now();

      Shimeta::Frame frame;
      frame.aps.data = raw.aps_data;
      frame.aps.size = raw.aps_size;
      frame.width = raw.aps_width;
      frame.height = raw.aps_height;
      frame.frame_id = static_cast<int>(raw.aps_frame_id);
      if (raw.aps_format != DG_HVS_HBN_PIXEL_FORMAT_NV12) {
        unsupported_aps_format_count_.fetch_add(1);
        continue;
      }
      frame.format = Shimeta::PixelFormat::NV12;
      frame.ts.aps_ts_ns = static_cast<int64_t>(raw.aps_timestamp_ns);

      last_aps_frame_id_ = frame.frame_id;
      last_aps_ts_ns_ = frame.ts.aps_ts_ns;
      aps_packet_count_.fetch_add(1);
      cv::Mat gray = aps_to_gray(frame);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_aps_width_ = raw.aps_width;
        last_aps_height_ = raw.aps_height;
        last_aps_format_ = "NV12";
        last_aps_bytes_ = raw.aps_size;
      }
      if (!gray.empty()) {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_aps_steady_ = steady_now;
        }
        aps_frame_count_.fetch_add(1);
        last_aps_sensor_ns_.store(raw.aps_timestamp_ns);
        update_hbn_pairing(raw.aps_timestamp_ns);
        publish_aps_and_preview_if_due(gray, receipt_time, steady_now);
      }
    }
  }
#endif

  void denoise_and_limit(std::vector<Shimeta::EventCD> & events)
  {
    if (events.empty()) {
      return;
    }
    const int cells_x = (event_width_ + denoise_cell_px_ - 1) / denoise_cell_px_;
    const int cells_y = (event_height_ + denoise_cell_px_ - 1) / denoise_cell_px_;
    std::vector<uint16_t> counts(static_cast<size_t>(cells_x * cells_y), 0);
    for (const auto & event : events) {
      if (event.x < event_width_ && event.y < event_height_) {
        auto & count = counts[(event.y / denoise_cell_px_) * cells_x + event.x / denoise_cell_px_];
        if (count < std::numeric_limits<uint16_t>::max()) {
          ++count;
        }
      }
    }
    events.erase(
      std::remove_if(
        events.begin(), events.end(), [&](const auto & event) {
          if (event.x >= event_width_ || event.y >= event_height_) {
            return true;
          }
          return counts[(event.y / denoise_cell_px_) * cells_x + event.x / denoise_cell_px_] <
                 denoise_min_events_per_cell_;
        }),
      events.end());
    const auto per_packet_limit = static_cast<size_t>(std::max(
        1.0, static_cast<double>(max_events_per_second_) / std::max(1, evs_fps_)));
    if (events.size() > per_packet_limit) {
      events.resize(per_packet_limit);
      rate_limited_event_count_.fetch_add(1);
    }
  }

  void update_event_canvas(const std::vector<Shimeta::EventCD> & events)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (event_canvas_.empty()) {
      event_canvas_ = cv::Mat::zeros(event_height_, event_width_, CV_32FC2);
    }
    event_canvas_ *= event_decay_;
    for (const auto & event : events) {
      if (event.x >= event_width_ || event.y >= event_height_) {
        continue;
      }
      auto & pixel = event_canvas_.at<cv::Vec2f>(event.y, event.x);
      pixel[event.polarity ? 1 : 0] = 1.0F;
    }
  }

  cv::Mat aps_to_gray(const Shimeta::Frame & frame) const
  {
    const int width = frame.width > 0 ? frame.width : aps_width_;
    const int height = frame.height > 0 ? frame.height : aps_height_;
    if (width <= 0 || height <= 0 || frame.aps.data == nullptr) {
      unsupported_aps_format_count_.fetch_add(1);
      return {};
    }
    const size_t pixels = static_cast<size_t>(width) * height;
    if ((frame.format == Shimeta::PixelFormat::Gray8 ||
      frame.format == Shimeta::PixelFormat::RAW8) && frame.aps.size >= pixels)
    {
      cv::Mat view(height, width, CV_8UC1, const_cast<uint8_t *>(frame.aps.data));
      return view.clone();
    }
    if (frame.format == Shimeta::PixelFormat::NV12 && frame.aps.size >= pixels * 3U / 2U) {
      cv::Mat nv12(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t *>(frame.aps.data));
      cv::Mat gray;
      cv::cvtColor(nv12, gray, cv::COLOR_YUV2GRAY_NV12);
      return gray;
    }
    if (frame.format == Shimeta::PixelFormat::BayerRG8 && frame.aps.size >= pixels) {
      cv::Mat bayer(height, width, CV_8UC1, const_cast<uint8_t *>(frame.aps.data));
      cv::Mat gray;
      cv::cvtColor(bayer, gray, cv::COLOR_BayerRG2GRAY);
      return gray;
    }
    if (frame.format == Shimeta::PixelFormat::RGB888 && frame.aps.size >= pixels * 3U) {
      cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(frame.aps.data));
      cv::Mat gray;
      cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
      return gray;
    }
    // Some X5 VIN paths expose unpacked RAW10 as 16-bit little-endian samples.
    // Keep this conservative: only accept the unambiguous 2-byte-per-pixel form.
    if (frame.format == Shimeta::PixelFormat::RAW10 && frame.aps.size >= pixels * 2U) {
      cv::Mat raw16(height, width, CV_16UC1, const_cast<uint8_t *>(frame.aps.data));
      cv::Mat gray;
      raw16.convertTo(gray, CV_8UC1, 1.0 / 4.0);
      return gray;
    }
    unsupported_aps_format_count_.fetch_add(1);
    return {};
  }

  bool is_new_aps_frame(const Shimeta::Frame & frame, const uint8_t * last_pointer) const
  {
    if (frame.aps.data == nullptr || frame.aps.size == 0) {
      return false;
    }
    if (frame.ts.aps_ts_ns > 0 && last_aps_ts_ns_ > 0) {
      return frame.ts.aps_ts_ns != last_aps_ts_ns_;
    }
    if (frame.aps.data != last_pointer) {
      return true;
    }
    // Frame IDs are a useful fallback when the pool reuses the same slab and
    // the platform does not expose an APS timestamp.
    return frame.frame_id != 0 && frame.frame_id != last_aps_frame_id_;
  }

  void update_pairing(const Shimeta::Frame & frame)
  {
    double skew_ms = std::numeric_limits<double>::quiet_NaN();
    if (frame.ts.aps_ts_ns > 0 && frame.ts.evs_ts_ns > 0) {
      skew_ms = std::abs(static_cast<double>(frame.ts.aps_ts_ns - frame.ts.evs_ts_ns)) / 1e6;
    } else if (frame.aps_evs_ts.valid && last_evs_sensor_us_.load() > 0) {
      const auto aps_us = frame.aps_evs_ts.processed_timestamp;
      const auto evs_us = last_evs_sensor_us_.load();
      skew_ms = static_cast<double>(aps_us > evs_us ? aps_us - evs_us : evs_us - aps_us) / 1000.0;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    pair_skew_ms_ = skew_ms;
    pair_valid_ = std::isfinite(skew_ms) && skew_ms <= pair_tolerance_ms_;
  }

  void update_hbn_pairing(uint64_t aps_timestamp_ns)
  {
    double best_skew_ms = std::numeric_limits<double>::quiet_NaN();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (aps_timestamp_ns > 0) {
      uint64_t best_skew_ns = std::numeric_limits<uint64_t>::max();
      for (const uint64_t evs_timestamp_ns : recent_evs_hbn_timestamps_) {
        if (evs_timestamp_ns == 0) {
          continue;
        }
        const uint64_t skew_ns = aps_timestamp_ns > evs_timestamp_ns ?
          aps_timestamp_ns - evs_timestamp_ns : evs_timestamp_ns - aps_timestamp_ns;
        best_skew_ns = std::min(best_skew_ns, skew_ns);
      }
      if (best_skew_ns != std::numeric_limits<uint64_t>::max()) {
        best_skew_ms = static_cast<double>(best_skew_ns) / 1e6;
      }
    }
    pair_skew_ms_ = best_skew_ms;
    pair_valid_ = std::isfinite(best_skew_ms) && best_skew_ms <= pair_tolerance_ms_;
  }

  struct FeaturePoint
  {
    double x{0.0};
    double y{0.0};
  };

  struct FeatureQualitySnapshot
  {
    size_t event_count{0};
    size_t unique_pixel_count{0};
    std::vector<FeaturePoint> points;
    std::string repeatability_status{"NO_PREVIOUS_PACKET"};
    double repeatability{std::numeric_limits<double>::quiet_NaN()};
  };

  static size_t one_to_one_match_count(
    const std::vector<FeaturePoint> & previous,
    const std::vector<FeaturePoint> & current,
    double match_distance_px)
  {
    const double limit_squared = match_distance_px * match_distance_px;
    std::vector<std::vector<size_t>> adjacency(previous.size());
    for (size_t previous_index = 0; previous_index < previous.size(); ++previous_index) {
      std::vector<std::pair<double, size_t>> matches;
      for (size_t current_index = 0; current_index < current.size(); ++current_index) {
        const double dx = previous[previous_index].x - current[current_index].x;
        const double dy = previous[previous_index].y - current[current_index].y;
        const double distance_squared = dx * dx + dy * dy;
        if (distance_squared <= limit_squared) {
          matches.emplace_back(distance_squared, current_index);
        }
      }
      std::stable_sort(matches.begin(), matches.end(), [](const auto & lhs, const auto & rhs) {
        if (lhs.first != rhs.first) {
          return lhs.first < rhs.first;
        }
        return lhs.second < rhs.second;
      });
      adjacency[previous_index].reserve(matches.size());
      for (const auto & match : matches) {
        adjacency[previous_index].push_back(match.second);
      }
    }

    std::vector<int> matched_current(current.size(), -1);
    std::function<bool(size_t, std::vector<bool> &)> augment =
      [&](size_t previous_index, std::vector<bool> & seen) {
        for (const size_t current_index : adjacency[previous_index]) {
          if (seen[current_index]) {
            continue;
          }
          seen[current_index] = true;
          const int displaced = matched_current[current_index];
          if (displaced < 0 || augment(static_cast<size_t>(displaced), seen)) {
            matched_current[current_index] = static_cast<int>(previous_index);
            return true;
          }
        }
        return false;
      };

    size_t matches = 0;
    for (size_t previous_index = 0; previous_index < previous.size(); ++previous_index) {
      std::vector<bool> seen(current.size(), false);
      if (augment(previous_index, seen)) {
        ++matches;
      }
    }
    return matches;
  }

  sensor_msgs::msg::PointCloud2 make_feature_cloud(
    const std::vector<Shimeta::EventCD> & events, const rclcpp::Time & stamp,
    FeatureQualitySnapshot & snapshot)
  {
    struct Accumulator {
      uint32_t count{0};
      uint32_t positive{0};
      double sum_x{0.0};
      double sum_y{0.0};
      std::unordered_set<uint64_t> unique_pixels;
    };
    const int cells_x = (event_width_ + feature_cell_px_ - 1) / feature_cell_px_;
    const int cells_y = (event_height_ + feature_cell_px_ - 1) / feature_cell_px_;
    std::vector<Accumulator> cells(static_cast<size_t>(cells_x * cells_y));
    for (const auto & event : events) {
      if (event.x >= event_width_ || event.y >= event_height_) {
        continue;
      }
      auto & cell = cells[(event.y / feature_cell_px_) * cells_x + event.x / feature_cell_px_];
      ++cell.count;
      cell.positive += event.polarity ? 1U : 0U;
      cell.sum_x += event.x;
      cell.sum_y += event.y;
      const uint64_t pixel = (static_cast<uint64_t>(event.y) << 32U) |
        static_cast<uint64_t>(event.x);
      cell.unique_pixels.insert(pixel);
    }
    struct Candidate { int index; const Accumulator * value; };
    std::vector<Candidate> candidates;
    for (int index = 0; index < static_cast<int>(cells.size()); ++index) {
      if (cells[index].count >= static_cast<uint32_t>(feature_min_events_per_cell_) &&
        cells[index].unique_pixels.size() >= static_cast<size_t>(feature_min_unique_pixels_))
      {
        candidates.push_back({index, &cells[index]});
      }
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate & lhs, const Candidate & rhs) {
      if (lhs.value->count != rhs.value->count) return lhs.value->count > rhs.value->count;
      if (lhs.value->unique_pixels.size() != rhs.value->unique_pixels.size()) {
        return lhs.value->unique_pixels.size() > rhs.value->unique_pixels.size();
      }
      return lhs.index < rhs.index;
    });
    if (candidates.size() > static_cast<size_t>(feature_max_features_)) {
      candidates.resize(static_cast<size_t>(feature_max_features_));
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(candidates.size());
    cloud.fields.resize(4);
    cloud.fields[0].name = "pixel_x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "pixel_y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "event_count";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;
    cloud.fields[3].name = "positive_fraction";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;
    cloud.is_bigendian = false;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);
    snapshot.event_count = events.size();
    snapshot.unique_pixel_count = 0;
    snapshot.points.clear();
    snapshot.points.reserve(candidates.size());
    for (size_t index = 0; index < candidates.size(); ++index) {
      const auto & value = *candidates[index].value;
      const float x = static_cast<float>(value.sum_x / value.count);
      const float y = static_cast<float>(value.sum_y / value.count);
      const float count = static_cast<float>(value.count);
      const float positive_fraction = static_cast<float>(value.positive) / value.count;
      const size_t offset = index * cloud.point_step;
      append_scalar(cloud.data, offset, x);
      append_scalar(cloud.data, offset + 4, y);
      append_scalar(cloud.data, offset + 8, count);
      append_scalar(cloud.data, offset + 12, positive_fraction);
      snapshot.points.push_back({x, y});
      snapshot.unique_pixel_count += value.unique_pixels.size();
    }
    if (!feature_previous_packet_seen_) {
      snapshot.repeatability_status = "NO_PREVIOUS_PACKET";
    } else if (previous_feature_points_.empty() || snapshot.points.empty()) {
      snapshot.repeatability_status = "ZERO_FEATURE_PAIR";
    } else {
      const size_t matches = one_to_one_match_count(
        previous_feature_points_, snapshot.points, feature_match_distance_px_);
      snapshot.repeatability = 2.0 * static_cast<double>(matches) /
        static_cast<double>(previous_feature_points_.size() + snapshot.points.size());
      snapshot.repeatability_status = "COMPARED";
    }
    previous_feature_points_ = snapshot.points;
    feature_previous_packet_seen_ = true;
    feature_packet_count_.fetch_add(1);
    feature_count_.store(candidates.size());
    return cloud;
  }

  void publish_feature_quality(
    const FeatureQualitySnapshot & snapshot, const rclcpp::Time & stamp)
  {
    if (!feature_quality_pub_) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    array.header.frame_id = frame_id_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dg/event_camera/feature_quality";
    status.hardware_id = camera_name_;
    status.message = snapshot.repeatability_status;
    status.level = snapshot.repeatability_status == "COMPARED" && !snapshot.points.empty() ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    const auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    const bool quality_good = snapshot.repeatability_status == "COMPARED" &&
      std::isfinite(snapshot.repeatability) && !snapshot.points.empty();
    add("state", quality_good ? "GOOD" : "DEGRADED");
    add("quality", quality_good ? "GOOD" : "DEGRADED");
    add("repeatability_status", snapshot.repeatability_status);
    add("repeatability", std::isfinite(snapshot.repeatability) ?
      std::to_string(snapshot.repeatability) : "nan");
    add("feature_count", std::to_string(snapshot.points.size()));
    add("event_count", std::to_string(snapshot.event_count));
    add("unique_pixel_count", std::to_string(snapshot.unique_pixel_count));
    add("feature_match_distance_px", std::to_string(feature_match_distance_px_));
    add("feature_packets", std::to_string(feature_packet_count_.load()));
    add("frame_id", frame_id_);
    add("observation_only", "true");
    add("publishes_cmd_vel", "false");
    array.status.push_back(std::move(status));
    feature_quality_pub_->publish(array);
  }

  void publish_events_if_due(
    const std::vector<Shimeta::EventCD> & events, const rclcpp::Time & stamp,
    std::chrono::steady_clock::time_point steady_now)
  {
    if (events.empty()) {
      return;
    }
    const auto period = std::chrono::duration<double>(1.0 / event_publish_rate_hz_);
    if (last_event_publish_ != std::chrono::steady_clock::time_point{} &&
      steady_now - last_event_publish_ < period)
    {
      dropped_event_messages_.fetch_add(1);
      return;
    }
    last_event_publish_ = steady_now;
    const size_t output_count = std::min(events.size(), static_cast<size_t>(max_events_per_message_));
    const size_t stride = std::max<size_t>(1, events.size() / output_count);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(output_count);
    cloud.fields.resize(4);
    cloud.fields[0].name = "pixel_x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "pixel_y";
    cloud.fields[1].offset = 2;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "polarity";
    cloud.fields[2].offset = 4;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[2].count = 1;
    cloud.fields[3].name = "time_offset_us";
    cloud.fields[3].offset = 8;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    cloud.fields[3].count = 1;
    cloud.is_bigendian = false;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);
    const int64_t first_time = events.front().t;
    for (size_t output_index = 0; output_index < output_count; ++output_index) {
      const auto & event = events[std::min(events.size() - 1, output_index * stride)];
      const size_t offset = output_index * cloud.point_step;
      const uint8_t polarity = event.polarity ? 1U : 0U;
      const int64_t delta = std::max<int64_t>(0, event.t - first_time);
      const uint32_t time_offset = static_cast<uint32_t>(
        std::min<int64_t>(delta, std::numeric_limits<uint32_t>::max()));
      append_scalar(cloud.data, offset, event.x);
      append_scalar(cloud.data, offset + 2, event.y);
      append_scalar(cloud.data, offset + 4, polarity);
      append_scalar(cloud.data, offset + 8, time_offset);
    }
    events_pub_->publish(cloud);
    if (features_pub_) {
      FeatureQualitySnapshot snapshot;
      features_pub_->publish(make_feature_cloud(events, stamp, snapshot));
      publish_feature_quality(snapshot, stamp);
    }
    published_event_count_.fetch_add(output_count);
  }

  void publish_aps_and_preview_if_due(
    const cv::Mat & gray, const rclcpp::Time & stamp,
    std::chrono::steady_clock::time_point steady_now)
  {
    const auto period = std::chrono::duration<double>(1.0 / preview_publish_rate_hz_);
    if (last_preview_publish_ != std::chrono::steady_clock::time_point{} &&
      steady_now - last_preview_publish_ < period)
    {
      record_aps_if_due(gray, cv::Mat{});
      return;
    }
    last_preview_publish_ = steady_now;
    cv::Mat resized_gray;
    if (gray.cols != event_width_ || gray.rows != event_height_) {
      cv::resize(gray, resized_gray, cv::Size(event_width_, event_height_), 0, 0, cv::INTER_AREA);
    } else {
      resized_gray = gray;
    }

    cv::Mat fused;
    cv::Mat mask;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      cv::cvtColor(resized_gray, fused, cv::COLOR_GRAY2BGR);
      mask = cv::Mat::zeros(event_height_, event_width_, CV_8UC1);
      if (!event_canvas_.empty()) {
        for (int y = 0; y < event_height_; ++y) {
          for (int x = 0; x < event_width_; ++x) {
            const auto value = event_canvas_.at<cv::Vec2f>(y, x);
            if (value[0] > 0.08F || value[1] > 0.08F) {
              mask.at<uint8_t>(y, x) = 255;
              auto & pixel = fused.at<cv::Vec3b>(y, x);
              if (value[1] >= value[0]) {
                pixel = cv::Vec3b(32, 32, 255);
              } else {
                pixel = cv::Vec3b(0, 165, 255);
              }
            }
          }
        }
      }
    }

    aps_pub_->publish(make_image(resized_gray, stamp, "mono8"));
    mask_pub_->publish(make_image(mask, stamp, "mono8"));
    aps_compressed_pub_->publish(make_compressed(resized_gray, stamp));

    bool pair_valid;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pair_valid = pair_valid_;
    }
    if (pair_valid) {
      fused_pub_->publish(make_image(fused, stamp, "bgr8"));
      fused_compressed_pub_->publish(make_compressed(fused, stamp));
      fused_frame_count_.fetch_add(1);
    } else {
      skipped_fused_frame_count_.fetch_add(1);
    }
    publish_camera_info(stamp, resized_gray.cols, resized_gray.rows);
    record_aps_if_due(resized_gray, pair_valid ? fused : cv::Mat{});
  }

  sensor_msgs::msg::Image make_image(
    const cv::Mat & image, const rclcpp::Time & stamp, const std::string & encoding) const
  {
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.height = static_cast<uint32_t>(image.rows);
    message.width = static_cast<uint32_t>(image.cols);
    message.encoding = encoding;
    message.is_bigendian = false;
    message.step = static_cast<uint32_t>(image.cols * image.elemSize());
    message.data.assign(image.data, image.data + image.total() * image.elemSize());
    return message;
  }

  sensor_msgs::msg::CompressedImage make_compressed(
    const cv::Mat & image, const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.format = "jpeg";
    cv::imencode(".jpg", image, message.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    return message;
  }

  void publish_camera_info(const rclcpp::Time & stamp, int width, int height)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.height = static_cast<uint32_t>(height);
    info.width = static_cast<uint32_t>(width);
    info.distortion_model = distortion_model_;
    info.d = distortion_coefficients_;
    info.k = {camera_fx_, 0.0, camera_cx_, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {camera_fx_, 0.0, camera_cx_, 0.0, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(info);
  }

  void record_events(const Shimeta::Frame & frame, const std::vector<Shimeta::EventCD> & events)
  {
    if (!recording_enabled_) {
      return;
    }
    raw8_file_.write(reinterpret_cast<const char *>(frame.evs.data), frame.evs.size);
    evt2_writer_.writeEvents(events);
    if (record_csv_enabled_) {
      for (const auto & event : events) {
        csv_file_ << event.x << ',' << event.y << ','
                  << (event.polarity ? 1 : 0) << ',' << event.t << '\n';
      }
    }
  }

  void record_aps_if_due(const cv::Mat & gray, const cv::Mat & fused)
  {
    if (!recording_enabled_) {
      return;
    }
    const uint64_t frame_number = aps_frame_count_.load();
    if (frame_number == 0 || frame_number % static_cast<uint64_t>(record_aps_every_n_) != 0) {
      return;
    }
    std::ostringstream name;
    name << std::setw(8) << std::setfill('0') << frame_number;
    cv::imwrite((recording_session_dir_ / "aps" / (name.str() + ".pgm")).string(), gray);
    if (!fused.empty()) {
      cv::imwrite(
        (recording_session_dir_ / "fused" / (name.str() + ".jpg")).string(), fused,
        {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    }
  }

  void publish_quality()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_evs_steady;
    std::chrono::steady_clock::time_point last_aps_steady;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_evs_steady = last_evs_steady_;
      last_aps_steady = last_aps_steady_;
    }
    const auto event_age = last_evs_steady == std::chrono::steady_clock::time_point{} ?
      std::numeric_limits<double>::infinity() :
      std::chrono::duration<double>(steady_now - last_evs_steady).count();
    const auto aps_age = last_aps_steady == std::chrono::steady_clock::time_point{} ?
      std::numeric_limits<double>::infinity() :
      std::chrono::duration<double>(steady_now - last_aps_steady).count();
    const bool event_fresh = event_age <= stream_timeout_sec_;
    const bool aps_fresh = aps_age <= stream_timeout_sec_;
    const uint64_t interval_events = interval_event_count_.exchange(0);
    const auto quality_elapsed = quality_last_steady_ == std::chrono::steady_clock::time_point{} ?
      1.0 : std::chrono::duration<double>(steady_now - quality_last_steady_).count();
    quality_last_steady_ = steady_now;
    event_rate_hz_ = static_cast<double>(interval_events) / std::max(quality_elapsed, 1e-6);
    const auto aps_frames = aps_frame_count_.load();
    const auto aps_frames_at_last_quality = aps_frames_at_last_quality_.exchange(aps_frames);
    aps_rate_hz_ = static_cast<double>(aps_frames - aps_frames_at_last_quality) /
      std::max(quality_elapsed, 1e-6);

    bool pair_valid;
    double pair_skew;
    int aps_width;
    int aps_height;
    size_t aps_bytes;
    std::string aps_format;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pair_valid = pair_valid_;
      pair_skew = pair_skew_ms_;
      aps_width = last_aps_width_;
      aps_height = last_aps_height_;
      aps_format = last_aps_format_;
      aps_bytes = last_aps_bytes_;
    }
    if (!event_fresh && !aps_fresh) {
      stream_state_ = "FAULT";
    } else if (!event_fresh || !aps_fresh || !pair_valid) {
      stream_state_ = "DEGRADED";
    } else if (event_rate_hz_ < static_event_rate_threshold_) {
      stream_state_ = "STATIC_IDLE";
    } else {
      stream_state_ = "GOOD";
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.header.frame_id = frame_id_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dg/event_camera/quality";
    status.hardware_id = camera_name_;
    status.message = stream_state_;
    status.level = stream_state_ == "FAULT" ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (stream_state_ == "DEGRADED" ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK);
    const auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    add("state", stream_state_);
    add("evs_fresh", bool_text(event_fresh));
    add("aps_fresh", bool_text(aps_fresh));
    add("pair_valid", bool_text(pair_valid));
    add("pair_skew_ms", std::isfinite(pair_skew) ? std::to_string(pair_skew) : "nan");
    add("pair_tolerance_ms", std::to_string(pair_tolerance_ms_));
    add("event_rate_per_sec", std::to_string(event_rate_hz_));
    const auto evs_packets = evs_packet_count_.load();
    const auto evs_packets_at_last_quality = evs_packets_at_last_quality_.exchange(evs_packets);
    add("event_packets_per_sec", std::to_string(
        static_cast<double>(evs_packets - evs_packets_at_last_quality) /
        std::max(quality_elapsed, 1e-6)));
    add("aps_rate_per_sec", std::to_string(aps_rate_hz_));
    add("static_event_rate_threshold", std::to_string(static_event_rate_threshold_));
    add("evs_packets", std::to_string(evs_packet_count_.load()));
    add("decoded_events", std::to_string(decoded_event_count_.load()));
    add("published_events", std::to_string(published_event_count_.load()));
    add("aps_frames", std::to_string(aps_frame_count_.load()));
    add("aps_packets", std::to_string(aps_packet_count_.load()));
    add("aps_width", std::to_string(aps_width));
    add("aps_height", std::to_string(aps_height));
    add("aps_format", aps_format);
    add("aps_bytes", std::to_string(aps_bytes));
    add("aps_decode_failures", std::to_string(unsupported_aps_format_count_.load()));
    add("get_frame_timeouts", std::to_string(get_frame_timeout_count_.load()));
    add("evs_get_frame_errors", std::to_string(evs_get_frame_error_count_.load()));
    add("aps_get_frame_errors", std::to_string(aps_get_frame_error_count_.load()));
    add("last_evs_get_frame_error", std::to_string(last_evs_get_frame_error_.load()));
    add("last_aps_get_frame_error", std::to_string(last_aps_get_frame_error_.load()));
    add("fused_frames", std::to_string(fused_frame_count_.load()));
    add("skipped_fused_frames", std::to_string(skipped_fused_frame_count_.load()));
    add("dropped_event_messages", std::to_string(dropped_event_messages_.load()));
    add("rate_limited_packets", std::to_string(rate_limited_event_count_.load()));
    add("features_enabled", bool_text(publish_features_));
    add("feature_count", std::to_string(feature_count_.load()));
    add("feature_min_unique_pixels", std::to_string(feature_min_unique_pixels_));
    add("feature_match_distance_px", std::to_string(feature_match_distance_px_));
    add("feature_packets", std::to_string(feature_packet_count_.load()));
    add("unsupported_aps_frames", std::to_string(unsupported_aps_format_count_.load()));
    add("recording_enabled", bool_text(recording_enabled_));
    add("afk_filter_enabled", "false");
    add("clock_basis", "ros_receipt_time_with_sensor_pair_diagnostics");
    add("publishes_cmd_vel", "false");
    array.status.push_back(std::move(status));
    quality_pub_->publish(array);
  }

  Shimeta::hv::Camera camera_;
  Shimeta::codec::MipiRaw8Decoder decoder_;
  Shimeta::io::EventWriter evt2_writer_;
  std::atomic<bool> running_{true};
  std::thread capture_thread_;
  rclcpp::TimerBase::SharedPtr quality_timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr events_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr aps_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr fused_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr quality_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr feature_quality_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr features_pub_;

  std::string device_node_;
  std::string backend_;
  int hbn_config_index_{};
  int hbn_settle_value_{};
  int sensor_index_{};
  int i2c_bus_{};
  int evs_fps_{};
  int buffer_count_{};
  std::string frame_id_;
  std::string camera_name_;
  int event_width_{};
  int event_height_{};
  int aps_width_{};
  int aps_height_{};
  double aps_expected_fps_{};
  double pair_tolerance_ms_{};
  double stream_timeout_sec_{};
  double static_event_rate_threshold_{};
  double event_publish_rate_hz_{};
  int max_events_per_message_{};
  double preview_publish_rate_hz_{};
  int jpeg_quality_{};
  double event_decay_{};
  int denoise_cell_px_{};
  int denoise_min_events_per_cell_{};
  int max_events_per_second_{};
  bool publish_features_{false};
  int feature_cell_px_{};
  int feature_min_events_per_cell_{};
  int feature_min_unique_pixels_{};
  int feature_max_features_{};
  double feature_match_distance_px_{};
  bool afk_filter_enabled_{};
  bool recording_enabled_{};
  std::string record_directory_;
  bool record_csv_enabled_{};
  int record_aps_every_n_{};
  double camera_fx_{};
  double camera_fy_{};
  double camera_cx_{};
  double camera_cy_{};
  std::string distortion_model_;
#ifdef DG_HVS_HAS_HBN_BACKEND
  dg_hvs_hbn_handle_t * hbn_handle_{nullptr};
  bool use_hbn_backend_{false};
  std::thread aps_capture_thread_;
#endif
  std::vector<double> distortion_coefficients_;

  std::mutex image_mutex_;
  cv::Mat event_canvas_;
  std::mutex state_mutex_;
  bool pair_valid_{false};
  double pair_skew_ms_{std::numeric_limits<double>::quiet_NaN()};
  std::string stream_state_{"FAULT"};
  double event_rate_hz_{0.0};
  double aps_rate_hz_{0.0};
  std::chrono::steady_clock::time_point quality_last_steady_{};
  std::chrono::steady_clock::time_point last_evs_steady_{};
  std::chrono::steady_clock::time_point last_aps_steady_{};
  std::chrono::steady_clock::time_point last_event_publish_{};
  std::chrono::steady_clock::time_point last_preview_publish_{};

  std::atomic<uint64_t> last_evs_sensor_us_{0};
  std::atomic<uint64_t> last_evs_hbn_sensor_ns_{0};
  std::array<uint64_t, 32> recent_evs_hbn_timestamps_{};
  size_t recent_evs_hbn_write_index_{0};
  std::atomic<uint64_t> last_aps_sensor_ns_{0};
  std::atomic<uint64_t> aps_packet_count_{0};
  std::atomic<uint64_t> get_frame_timeout_count_{0};
  std::atomic<uint64_t> evs_get_frame_error_count_{0};
  std::atomic<uint64_t> aps_get_frame_error_count_{0};
  std::atomic<int> last_evs_get_frame_error_{0};
  std::atomic<int> last_aps_get_frame_error_{0};
  std::atomic<uint64_t> aps_frames_at_last_quality_{0};
  std::atomic<uint64_t> evs_packets_at_last_quality_{0};
  std::atomic<uint64_t> evs_packet_count_{0};
  std::atomic<uint64_t> decoded_event_count_{0};
  std::atomic<uint64_t> interval_event_count_{0};
  std::atomic<uint64_t> published_event_count_{0};
  std::atomic<uint64_t> feature_count_{0};
  std::atomic<uint64_t> aps_frame_count_{0};
  std::atomic<uint64_t> fused_frame_count_{0};
  std::atomic<uint64_t> skipped_fused_frame_count_{0};
  std::atomic<uint64_t> dropped_event_messages_{0};
  std::atomic<uint64_t> rate_limited_event_count_{0};
  std::atomic<uint64_t> feature_packet_count_{0};
  mutable std::atomic<uint64_t> unsupported_aps_format_count_{0};
  std::vector<FeaturePoint> previous_feature_points_;
  bool feature_previous_packet_seen_{false};
  int last_aps_frame_id_{0};
  int64_t last_aps_ts_ns_{0};
  int last_aps_width_{0};
  int last_aps_height_{0};
  size_t last_aps_bytes_{0};
  std::string last_aps_format_{"UNKNOWN"};

  fs::path recording_session_dir_;
  std::ofstream raw8_file_;
  std::ofstream csv_file_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<DgHvsBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "dg_hvs_bridge fatal: " << error.what() << std::endl;
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
