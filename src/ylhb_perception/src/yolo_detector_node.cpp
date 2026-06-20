#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

using Clock = std::chrono::steady_clock;

class TrtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      RCLCPP_WARN(rclcpp::get_logger("yolo_detector_node.trt"), "%s", msg);
    }
  }
};

struct CudaDeleter
{
  void operator()(void * ptr) const noexcept
  {
    if (ptr != nullptr) {
      cudaFree(ptr);
    }
  }
};

using DevicePtr = std::unique_ptr<void, CudaDeleter>;

struct CudaHostDeleter
{
  void operator()(void * ptr) const noexcept
  {
    if (ptr != nullptr) {
      cudaFreeHost(ptr);
    }
  }
};

template<typename T>
using HostPtr = std::unique_ptr<T, CudaHostDeleter>;

struct Detection
{
  int class_id{-1};
  std::string class_name;
  float confidence{0.0F};
  cv::Rect2f box;
};

struct LetterboxInfo
{
  float scale{1.0F};
  float pad_x{0.0F};
  float pad_y{0.0F};
};

inline void cuda_check(cudaError_t status, const char * what)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(status));
  }
}

int64_t volume(const nvinfer1::Dims & dims)
{
  int64_t v = 1;
  for (int32_t i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) {
      return -1;
    }
    v *= dims.d[i];
  }
  return v;
}

std::string json_escape(const std::string & s)
{
  std::ostringstream out;
  for (char c : s) {
    switch (c) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        } else {
          out << c;
        }
    }
  }
  return out.str();
}

float iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float inter = (a & b).area();
  const float uni = a.area() + b.area() - inter;
  return uni > 0.0F ? inter / uni : 0.0F;
}

}  // namespace

class YoloDetectorNode final : public rclcpp::Node
{
public:
  YoloDetectorNode()
  : Node("yolo_detector_node")
  {
    declare_parameter<std::string>("image_topic", "/zed/zed_node/rgb/color/rect/image");
    declare_parameter<std::string>("detections_topic", "/perception/detections");
    declare_parameter<std::string>("debug_image_topic", "/perception/debug_image");
    declare_parameter<std::string>(
      "model_path", "/home/nvidia/ros2_DL/src/ylhb_perception/models/yolo26.engine");
    declare_parameter<std::string>("backend", "tensorrt");
    declare_parameter<double>("confidence_threshold", 0.35);
    declare_parameter<double>("iou_threshold", 0.45);
    declare_parameter<bool>("publish_debug_image", false);
    declare_parameter<bool>("show_debug_window", false);
    declare_parameter<std::string>("debug_window_name", "YOLO26 TensorRT Debug");
    declare_parameter<std::string>("device", "cuda:0");
    declare_parameter<int>("imgsz", 640);
    declare_parameter<int>("max_det", 100);
    declare_parameter<bool>("half", true);
    declare_parameter<double>("log_interval_sec", 2.0);
    declare_parameter<bool>("require_tensorrt", true);
    declare_parameter<double>("debug_image_max_hz", 5.0);
    declare_parameter<double>("debug_window_max_hz", 15.0);
    declare_parameter<std::vector<std::string>>("class_names", std::vector<std::string>{});

    image_topic_ = get_parameter("image_topic").as_string();
    detections_topic_ = get_parameter("detections_topic").as_string();
    debug_image_topic_ = get_parameter("debug_image_topic").as_string();
    model_path_ = get_parameter("model_path").as_string();
    backend_ = get_parameter("backend").as_string();
    actual_backend_ = detect_backend(model_path_);
    conf_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
    show_debug_window_ = get_parameter("show_debug_window").as_bool();
    debug_window_name_ = get_parameter("debug_window_name").as_string();
    imgsz_ = get_parameter("imgsz").as_int();
    max_det_ = get_parameter("max_det").as_int();
    log_interval_sec_ = std::max(0.1, get_parameter("log_interval_sec").as_double());
    require_tensorrt_ = get_parameter("require_tensorrt").as_bool();
    const double debug_image_max_hz = get_parameter("debug_image_max_hz").as_double();
    debug_image_min_interval_sec_ = debug_image_max_hz > 0.0 ?
      1.0 / debug_image_max_hz :
      0.0;
    const double debug_window_max_hz = get_parameter("debug_window_max_hz").as_double();
    debug_window_min_interval_sec_ = debug_window_max_hz > 0.0 ?
      1.0 / debug_window_max_hz :
      0.0;
    const auto class_names_param = get_parameter("class_names");
    if (class_names_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      class_names_ = class_names_param.as_string_array();
    } else {
      class_names_.clear();
    }

    if (backend_ != actual_backend_) {
      RCLCPP_WARN(
        get_logger(), "Backend parameter '%s' does not match model file '%s'; actual backend is '%s'",
        backend_.c_str(), model_path_.c_str(), actual_backend_.c_str());
    }
    if (require_tensorrt_ && actual_backend_ != "tensorrt") {
      throw std::runtime_error("require_tensorrt is true but model_path is not a .engine file");
    }

    load_engine();

    if (show_debug_window_) {
      cv::namedWindow(debug_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(debug_window_name_, 960, 540);
    }

    detections_pub_ = create_publisher<std_msgs::msg::String>(detections_topic_, 1);
    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::QoS(1).reliable());
    }
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&YoloDetectorNode::image_callback, this, std::placeholders::_1));

    stats_last_log_time_ = Clock::now();
    RCLCPP_INFO(
      get_logger(),
      "C++ TensorRT YOLO detector started: image=%s, detections=%s, debug_image=%s, "
      "publish_debug_image=%s, show_debug_window=%s, backend=%s, model=%s, conf=%.2f, iou=%.2f, imgsz=%d, "
      "engine_input=%dx%d",
      image_topic_.c_str(), detections_topic_.c_str(), debug_image_topic_.c_str(),
      publish_debug_image_ ? "true" : "false", show_debug_window_ ? "true" : "false",
      actual_backend_.c_str(), model_path_.c_str(),
      conf_threshold_, iou_threshold_, imgsz_, input_w_, input_h_);
  }

  ~YoloDetectorNode() override
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    if (show_debug_window_) {
      cv::destroyWindow(debug_window_name_);
    }
  }

private:
  std::string detect_backend(const std::string & path) const
  {
    const auto dot = path.find_last_of('.');
    const std::string suffix = dot == std::string::npos ? "" : path.substr(dot);
    if (suffix == ".engine") {
      return "tensorrt";
    }
    if (suffix == ".onnx") {
      return "onnxruntime";
    }
    if (suffix == ".pt") {
      return "pytorch";
    }
    return backend_;
  }

  void load_engine()
  {
    std::ifstream file(model_path_, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Model file does not exist: " + model_path_);
    }
    file.seekg(0, std::ios::end);
    const auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(static_cast<size_t>(size));
    file.read(engine_data.data(), size);

    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
    if (!engine_) {
      throw std::runtime_error("Failed to deserialize TensorRT engine: " + model_path_);
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("Failed to create TensorRT execution context");
    }

    discover_tensors();
    allocate_buffers();
    cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate");
  }

  void discover_tensors()
  {
    const int32_t nb_io = engine_->getNbIOTensors();
    for (int32_t i = 0; i < nb_io; ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
      } else if (output_name_.empty()) {
        output_name_ = name;
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT engine must have one input and one output tensor");
    }

    input_dims_ = engine_->getTensorShape(input_name_.c_str());
    if (input_dims_.nbDims != 4) {
      throw std::runtime_error("Expected 4D YOLO input tensor");
    }

    for (int32_t i = 0; i < input_dims_.nbDims; ++i) {
      if (input_dims_.d[i] < 0) {
        if (i == 0) {
          input_dims_.d[i] = 1;
        } else if (i == 2 || i == 3) {
          input_dims_.d[i] = imgsz_;
        }
      }
    }
    if (!context_->setInputShape(input_name_.c_str(), input_dims_)) {
      throw std::runtime_error("Failed to set TensorRT input shape");
    }

    input_is_nchw_ = input_dims_.d[1] == 3;
    if (!input_is_nchw_ && input_dims_.d[3] != 3) {
      throw std::runtime_error("YOLO input tensor must be NCHW or NHWC with 3 channels");
    }
    input_h_ = static_cast<int>(input_is_nchw_ ? input_dims_.d[2] : input_dims_.d[1]);
    input_w_ = static_cast<int>(input_is_nchw_ ? input_dims_.d[3] : input_dims_.d[2]);
    input_type_ = engine_->getTensorDataType(input_name_.c_str());
    output_type_ = engine_->getTensorDataType(output_name_.c_str());
    output_dims_ = context_->getTensorShape(output_name_.c_str());
    output_count_ = volume(output_dims_);
    if (output_count_ <= 0) {
      throw std::runtime_error("Invalid TensorRT output tensor shape");
    }
    if (imgsz_ != input_w_ || imgsz_ != input_h_) {
      RCLCPP_WARN(
        get_logger(), "Launch imgsz=%d, but engine input is %dx%d; engine shape wins.",
        imgsz_, input_w_, input_h_);
    }
  }

  void allocate_buffers()
  {
    const int64_t input_count = volume(input_dims_);
    const size_t input_bytes =
      static_cast<size_t>(input_count) *
      (input_type_ == nvinfer1::DataType::kHALF ? sizeof(__half) : sizeof(float));
    const size_t output_bytes =
      static_cast<size_t>(output_count_) *
      (output_type_ == nvinfer1::DataType::kHALF ? sizeof(__half) : sizeof(float));

    void * input_device = nullptr;
    void * output_device = nullptr;
    cuda_check(cudaMalloc(&input_device, input_bytes), "cudaMalloc input");
    cuda_check(cudaMalloc(&output_device, output_bytes), "cudaMalloc output");
    input_device_.reset(input_device);
    output_device_.reset(output_device);
    input_count_ = static_cast<size_t>(input_count);
    output_count_size_ = static_cast<size_t>(output_count_);

    void * input_host = nullptr;
    void * output_host = nullptr;
    cuda_check(cudaHostAlloc(&input_host, input_bytes, cudaHostAllocDefault), "cudaHostAlloc input");
    cuda_check(cudaHostAlloc(&output_host, output_bytes, cudaHostAllocDefault), "cudaHostAlloc output");
    if (input_type_ == nvinfer1::DataType::kHALF) {
      input_host_half_.reset(static_cast<__half *>(input_host));
    } else {
      input_host_float_.reset(static_cast<float *>(input_host));
    }
    if (output_type_ == nvinfer1::DataType::kHALF) {
      output_host_half_.reset(static_cast<__half *>(output_host));
      output_host_float_scratch_.resize(output_count_size_);
    } else {
      output_host_float_.reset(static_cast<float *>(output_host));
    }

    if (!context_->setTensorAddress(input_name_.c_str(), input_device_.get()) ||
      !context_->setTensorAddress(output_name_.c_str(), output_device_.get()))
    {
      throw std::runtime_error("Failed to set TensorRT tensor addresses");
    }
  }

  LetterboxInfo preprocess(const cv::Mat & bgr)
  {
    const float scale = std::min(static_cast<float>(input_w_) / bgr.cols, static_cast<float>(input_h_) / bgr.rows);
    const int new_w = static_cast<int>(std::round(bgr.cols * scale));
    const int new_h = static_cast<int>(std::round(bgr.rows * scale));
    const int pad_x = (input_w_ - new_w) / 2;
    const int pad_y = (input_h_ - new_h) / 2;

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(new_w, new_h), 0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat canvas(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));
    cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);

    if (input_is_nchw_) {
      const int area = input_h_ * input_w_;
      for (int y = 0; y < input_h_; ++y) {
        const auto * row = canvas.ptr<cv::Vec3b>(y);
        for (int x = 0; x < input_w_; ++x) {
          const int idx = y * input_w_ + x;
          set_input_value(static_cast<size_t>(idx), row[x][0] / 255.0F);
          set_input_value(static_cast<size_t>(area + idx), row[x][1] / 255.0F);
          set_input_value(static_cast<size_t>(2 * area + idx), row[x][2] / 255.0F);
        }
      }
    } else {
      size_t idx = 0;
      for (int y = 0; y < input_h_; ++y) {
        const auto * row = canvas.ptr<cv::Vec3b>(y);
        for (int x = 0; x < input_w_; ++x) {
          set_input_value(idx++, row[x][0] / 255.0F);
          set_input_value(idx++, row[x][1] / 255.0F);
          set_input_value(idx++, row[x][2] / 255.0F);
        }
      }
    }

    return LetterboxInfo{scale, static_cast<float>(pad_x), static_cast<float>(pad_y)};
  }

  void set_input_value(size_t index, float value)
  {
    if (input_type_ == nvinfer1::DataType::kHALF) {
      input_host_half_.get()[index] = __float2half(value);
    } else {
      input_host_float_.get()[index] = value;
    }
  }

  const float * output_float_data()
  {
    if (output_type_ != nvinfer1::DataType::kHALF) {
      return output_host_float_.get();
    }
    for (size_t i = 0; i < output_count_size_; ++i) {
      output_host_float_scratch_[i] = __half2float(output_host_half_.get()[i]);
    }
    return output_host_float_scratch_.data();
  }

  void infer()
  {
    const void * input_host = input_type_ == nvinfer1::DataType::kHALF ?
      static_cast<const void *>(input_host_half_.get()) :
      static_cast<const void *>(input_host_float_.get());
    const size_t input_bytes =
      input_count_ * (input_type_ == nvinfer1::DataType::kHALF ? sizeof(__half) : sizeof(float));
    const size_t output_bytes =
      output_count_size_ * (output_type_ == nvinfer1::DataType::kHALF ? sizeof(__half) : sizeof(float));
    void * output_host = output_type_ == nvinfer1::DataType::kHALF ?
      static_cast<void *>(output_host_half_.get()) :
      static_cast<void *>(output_host_float_.get());

    cuda_check(cudaMemcpyAsync(input_device_.get(), input_host, input_bytes, cudaMemcpyHostToDevice, stream_), "H2D");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }
    cuda_check(cudaMemcpyAsync(output_host, output_device_.get(), output_bytes, cudaMemcpyDeviceToHost, stream_), "D2H");
    cuda_check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
  }

  std::vector<Detection> parse_output(const float * out, const LetterboxInfo & lb, int image_w, int image_h)
  {
    std::vector<Detection> proposals;
    if (is_nms_output()) {
      parse_nms_output(out, lb, image_w, image_h, proposals);
    } else {
      parse_raw_output(out, lb, image_w, image_h, proposals);
    }
    std::sort(proposals.begin(), proposals.end(), [](const Detection & a, const Detection & b) {
      return a.confidence > b.confidence;
    });

    std::vector<Detection> kept;
    kept.reserve(std::min<int>(max_det_, proposals.size()));
    for (const auto & det : proposals) {
      bool suppress = false;
      for (const auto & existing : kept) {
        if (det.class_id == existing.class_id && iou(det.box, existing.box) > iou_threshold_) {
          suppress = true;
          break;
        }
      }
      if (!suppress) {
        kept.push_back(det);
        if (static_cast<int>(kept.size()) >= max_det_) {
          break;
        }
      }
    }
    return kept;
  }

  bool is_nms_output() const
  {
    if (output_dims_.nbDims < 2) {
      return false;
    }
    const int64_t last = output_dims_.d[output_dims_.nbDims - 1];
    return last == 6 || last == 7;
  }

  void parse_nms_output(
    const float * out, const LetterboxInfo & lb, int image_w, int image_h,
    std::vector<Detection> & proposals)
  {
    const int attrs = static_cast<int>(output_dims_.d[output_dims_.nbDims - 1]);
    const int count = static_cast<int>(output_count_size_ / static_cast<size_t>(attrs));
    for (int i = 0; i < count; ++i) {
      const float * p = out + i * attrs;
      const float conf = p[4];
      if (conf < conf_threshold_) {
        continue;
      }
      const int cls = static_cast<int>(std::round(p[5]));
      add_detection(p[0], p[1], p[2], p[3], conf, cls, lb, image_w, image_h, true, proposals);
    }
  }

  void parse_raw_output(
    const float * out, const LetterboxInfo & lb, int image_w, int image_h,
    std::vector<Detection> & proposals)
  {
    if (output_dims_.nbDims < 3) {
      return;
    }
    int64_t dim_a = output_dims_.d[output_dims_.nbDims - 2];
    int64_t dim_b = output_dims_.d[output_dims_.nbDims - 1];
    bool attrs_first = dim_a < dim_b;
    int attrs = static_cast<int>(attrs_first ? dim_a : dim_b);
    int count = static_cast<int>(attrs_first ? dim_b : dim_a);
    if (attrs < 6) {
      return;
    }

    for (int i = 0; i < count; ++i) {
      float cx = 0.0F;
      float cy = 0.0F;
      float w = 0.0F;
      float h = 0.0F;
      int best_cls = -1;
      float best_score = 0.0F;

      if (attrs_first) {
        cx = out[0 * count + i];
        cy = out[1 * count + i];
        w = out[2 * count + i];
        h = out[3 * count + i];
        for (int c = 4; c < attrs; ++c) {
          const float score = out[c * count + i];
          if (score > best_score) {
            best_score = score;
            best_cls = c - 4;
          }
        }
      } else {
        const float * p = out + i * attrs;
        cx = p[0];
        cy = p[1];
        w = p[2];
        h = p[3];
        for (int c = 4; c < attrs; ++c) {
          if (p[c] > best_score) {
            best_score = p[c];
            best_cls = c - 4;
          }
        }
      }

      if (best_score >= conf_threshold_) {
        add_detection(cx, cy, w, h, best_score, best_cls, lb, image_w, image_h, false, proposals);
      }
    }
  }

  void add_detection(
    float a, float b, float c, float d, float conf, int cls, const LetterboxInfo & lb, int image_w, int image_h,
    bool xyxy, std::vector<Detection> & proposals)
  {
    float x1 = xyxy ? a : a - c * 0.5F;
    float y1 = xyxy ? b : b - d * 0.5F;
    float x2 = xyxy ? c : a + c * 0.5F;
    float y2 = xyxy ? d : b + d * 0.5F;

    x1 = (x1 - lb.pad_x) / lb.scale;
    y1 = (y1 - lb.pad_y) / lb.scale;
    x2 = (x2 - lb.pad_x) / lb.scale;
    y2 = (y2 - lb.pad_y) / lb.scale;
    x1 = std::clamp(x1, 0.0F, static_cast<float>(image_w - 1));
    y1 = std::clamp(y1, 0.0F, static_cast<float>(image_h - 1));
    x2 = std::clamp(x2, 0.0F, static_cast<float>(image_w - 1));
    y2 = std::clamp(y2, 0.0F, static_cast<float>(image_h - 1));
    if (x2 <= x1 || y2 <= y1) {
      return;
    }

    Detection det;
    det.class_id = cls;
    det.class_name = class_name(cls);
    det.confidence = conf;
    det.box = cv::Rect2f(cv::Point2f(x1, y1), cv::Point2f(x2, y2));
    proposals.push_back(std::move(det));
  }

  std::string class_name(int class_id) const
  {
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size()) && !class_names_[class_id].empty()) {
      return class_names_[class_id];
    }
    return std::to_string(class_id);
  }

  std::string build_json(const sensor_msgs::msg::Image::SharedPtr & msg, const std::vector<Detection> & detections) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\"header\":{\"stamp\":{\"sec\":" << msg->header.stamp.sec <<
      ",\"nanosec\":" << msg->header.stamp.nanosec << "},\"frame_id\":\"" <<
      json_escape(msg->header.frame_id) << "\"},\"backend\":\"" << json_escape(backend_) <<
      "\",\"actual_backend\":\"" << json_escape(actual_backend_) << "\",\"model_path\":\"" <<
      json_escape(model_path_) << "\",\"detections\":[";
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto & d = detections[i];
      const float x1 = d.box.x;
      const float y1 = d.box.y;
      const float x2 = d.box.x + d.box.width;
      const float y2 = d.box.y + d.box.height;
      if (i > 0) {
        out << ',';
      }
      out << "{\"class_id\":" << d.class_id << ",\"class_name\":\"" << json_escape(d.class_name) <<
        "\",\"confidence\":" << d.confidence << ",\"bbox_xyxy\":[" << x1 << ',' << y1 << ',' <<
        x2 << ',' << y2 << "],\"bbox_center\":[" << (x1 + x2) * 0.5F << ',' <<
        (y1 + y2) * 0.5F << "],\"bbox_size\":[" << d.box.width << ',' << d.box.height << "]}";
    }
    out << "]}";
    return out.str();
  }

  void draw_debug(cv::Mat & image, const std::vector<Detection> & detections)
  {
    for (const auto & det : detections) {
      cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);
      std::ostringstream label;
      label << det.class_name << ' ' << std::fixed << std::setprecision(2) << det.confidence;
      int baseline = 0;
      const auto size = cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      const int y = std::max(0, static_cast<int>(det.box.y) - size.height - baseline);
      cv::rectangle(
        image, cv::Rect(static_cast<int>(det.box.x), y, size.width, size.height + baseline),
        cv::Scalar(0, 255, 0), cv::FILLED);
      cv::putText(
        image, label.str(), cv::Point(static_cast<int>(det.box.x), y + size.height),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto callback_start = Clock::now();
    ++frame_count_;
    ++stats_frame_count_;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to convert image: %s", e.what());
      return;
    }

    std::vector<Detection> detections;
    double preprocess_ms = 0.0;
    double inference_ms = 0.0;
    double postprocess_ms = 0.0;

    try {
      const auto pre_start = Clock::now();
      const auto lb = preprocess(cv_ptr->image);
      preprocess_ms = elapsed_ms(pre_start);

      const auto infer_start = Clock::now();
      infer();
      inference_ms = elapsed_ms(infer_start);

      const auto post_start = Clock::now();
      detections = parse_output(output_float_data(), lb, cv_ptr->image.cols, cv_ptr->image.rows);
      postprocess_ms = elapsed_ms(post_start);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "YOLO TensorRT inference failed: %s", e.what());
    }

    std_msgs::msg::String out;
    out.data = build_json(msg, detections);
    detections_pub_->publish(out);

    const auto now = Clock::now();
    const bool should_publish_debug =
      publish_debug_image_ &&
      (!has_published_debug_image_ ||
      std::chrono::duration<double>(now - last_debug_publish_time_).count() >= debug_image_min_interval_sec_);
    const bool should_show_debug =
      show_debug_window_ &&
      (!has_shown_debug_window_ ||
      std::chrono::duration<double>(now - last_debug_window_time_).count() >= debug_window_min_interval_sec_);
    if (should_publish_debug || should_show_debug) {
      cv::Mat debug_image = cv_ptr->image.clone();
      draw_debug(debug_image, detections);
      if (should_publish_debug) {
        if (debug_pub_) {
          auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
          debug_pub_->publish(*debug_msg);
        }
        last_debug_publish_time_ = now;
        has_published_debug_image_ = true;
        ++stats_debug_image_count_;
      }
      if (should_show_debug) {
        cv::imshow(debug_window_name_, debug_image);
        cv::waitKey(1);
        last_debug_window_time_ = now;
        has_shown_debug_window_ = true;
        ++stats_debug_window_count_;
      }
    }

    stats_predict_ms_total_ += preprocess_ms + inference_ms + postprocess_ms;
    stats_trt_ms_total_ += inference_ms;
    ++stats_predict_count_;
    maybe_log_runtime_stats(
      *msg, detections, preprocess_ms, inference_ms, postprocess_ms,
      elapsed_ms(callback_start));
  }

  double elapsed_ms(Clock::time_point start) const
  {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  }

  void maybe_log_runtime_stats(
    const sensor_msgs::msg::Image & msg, const std::vector<Detection> & detections,
    double preprocess_ms, double inference_ms, double postprocess_ms, double callback_ms)
  {
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - stats_last_log_time_).count();
    if (elapsed < log_interval_sec_) {
      return;
    }

    const double fps = elapsed > 0.0 ? stats_frame_count_ / elapsed : 0.0;
    const double avg_predict_ms = stats_predict_count_ > 0 ? stats_predict_ms_total_ / stats_predict_count_ : 0.0;
    const double avg_trt_ms = stats_predict_count_ > 0 ? stats_trt_ms_total_ / stats_predict_count_ : 0.0;
    std::ostringstream summary;
    for (size_t i = 0; i < std::min<size_t>(5, detections.size()); ++i) {
      if (i > 0) {
        summary << ", ";
      }
      summary << detections[i].class_name << ':' << std::fixed << std::setprecision(2) << detections[i].confidence;
    }
    if (summary.str().empty()) {
      summary << "no detections";
    }

    RCLCPP_INFO(
      get_logger(),
      "帧率=%.1f FPS, 帧号=%zu, 图像=%ux%u, 目标=%zu (%s), 总预测=%.1fms, "
      "平均预测=%.1fms, TRT=%.1fms, 平均TRT=%.1fms, 预处理=%.1fms, 后处理=%.1fms, "
      "总耗时=%.1fms, ROS调试图=%.1f FPS, 窗口=%.1f FPS, 后端=%s",
      fps, frame_count_, msg.width, msg.height, detections.size(), summary.str().c_str(),
      preprocess_ms + inference_ms + postprocess_ms, avg_predict_ms, inference_ms, avg_trt_ms,
      preprocess_ms, postprocess_ms, callback_ms,
      elapsed > 0.0 ? stats_debug_image_count_ / elapsed : 0.0,
      elapsed > 0.0 ? stats_debug_window_count_ / elapsed : 0.0,
      actual_backend_.c_str());

    stats_last_log_time_ = now;
    stats_frame_count_ = 0;
    stats_predict_ms_total_ = 0.0;
    stats_trt_ms_total_ = 0.0;
    stats_predict_count_ = 0;
    stats_debug_image_count_ = 0;
    stats_debug_window_count_ = 0;
  }

  TrtLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_{nullptr};

  std::string image_topic_;
  std::string detections_topic_;
  std::string debug_image_topic_;
  std::string model_path_;
  std::string backend_;
  std::string actual_backend_;
  std::string input_name_;
  std::string output_name_;
  std::vector<std::string> class_names_;

  float conf_threshold_{0.35F};
  float iou_threshold_{0.45F};
  bool publish_debug_image_{false};
  bool show_debug_window_{false};
  bool require_tensorrt_{true};
  std::string debug_window_name_{"YOLO26 TensorRT Debug"};
  int imgsz_{640};
  int max_det_{100};
  int input_h_{640};
  int input_w_{640};
  bool input_is_nchw_{true};
  nvinfer1::DataType input_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  int64_t output_count_{0};
  size_t input_count_{0};
  size_t output_count_size_{0};

  DevicePtr input_device_{nullptr};
  DevicePtr output_device_{nullptr};
  HostPtr<float> input_host_float_{nullptr};
  HostPtr<__half> input_host_half_{nullptr};
  HostPtr<float> output_host_float_{nullptr};
  HostPtr<__half> output_host_half_{nullptr};
  std::vector<float> output_host_float_scratch_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  double log_interval_sec_{2.0};
  double debug_image_min_interval_sec_{0.2};
  double debug_window_min_interval_sec_{1.0 / 15.0};
  size_t frame_count_{0};
  Clock::time_point stats_last_log_time_;
  Clock::time_point last_debug_publish_time_{};
  Clock::time_point last_debug_window_time_{};
  bool has_published_debug_image_{false};
  bool has_shown_debug_window_{false};
  int stats_frame_count_{0};
  double stats_predict_ms_total_{0.0};
  double stats_trt_ms_total_{0.0};
  int stats_predict_count_{0};
  int stats_debug_image_count_{0};
  int stats_debug_window_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YoloDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_detector_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
