#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fcntl.h>
#include <limits>
#include <linux/videodev2.h>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using namespace std::chrono_literals;
using CB =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
struct Control {
  uint32_t id;
  int scale;
};
static const std::map<std::string, Control> controls = {
    {"auto_exposure", {V4L2_CID_EXPOSURE_AUTO, 1}},
    {"exposure_time_us", {V4L2_CID_EXPOSURE_ABSOLUTE, 100}},
    {"auto_white_balance", {V4L2_CID_AUTO_WHITE_BALANCE, 1}},
    {"white_balance_temperature", {V4L2_CID_WHITE_BALANCE_TEMPERATURE, 1}},
    {"brightness", {V4L2_CID_BRIGHTNESS, 1}},
    {"contrast", {V4L2_CID_CONTRAST, 1}},
    {"saturation", {V4L2_CID_SATURATION, 1}},
    {"hue", {V4L2_CID_HUE, 1}},
    {"gamma", {V4L2_CID_GAMMA, 1}},
    {"gain", {V4L2_CID_GAIN, 1}},
    {"sharpness", {V4L2_CID_SHARPNESS, 1}},
    {"backlight_compensation", {V4L2_CID_BACKLIGHT_COMPENSATION, 1}},
    {"power_line_frequency", {V4L2_CID_POWER_LINE_FREQUENCY, 1}}};
static std::map<std::string, std::string> parameter_descriptions() {
  std::map<std::string, std::string> result;
  result["autostart"] =
      "true 自动配置采集；false 等待 lifecycle configure / activate";
  result["device"] = "V4L2设备路径；也可使用/dev/videoX，推荐固定链接";
  result["pixel_format"] =
      "MJPG / YUYV；当前设备 "
      "MJPG支持1080p/720p/640x480均20fps；YUYV对应5/10/20fps";
  result["width"] = "当前设备可选1920/1280/640，与height、fps按上述组合使用";
  result["height"] = "对应1080/720/480；不支持组合报错，不自动降低规格";
  result["fps"] = "目标帧率，浮点数，实际速率见/diagnostics";
  result["camera_name"] = "相机逻辑名称";
  result["frame_id"] = "消息坐标系，不发布TF";
  result["camera_info_url"] = "空=未标定；file:///绝对路径或package://包名/"
                              "路径；标准标定YAML，尺寸须匹配";
  result["publish_raw"] = "true/false；bgr8，有订阅者才解码";
  result["publish_compressed"] = "true/false；MJPG直接转发，YUYV额外编码";
  result["jpeg_quality"] = "1～100，仅用于YUYV转JPEG";
  result["qos.reliability"] = "best_effort / reliable";
  result["qos.depth"] = "正整数，队列深度";
  result["capture.timeout_ms"] = "正整数，无帧超时，单位毫秒";
  result["reconnect.enabled"] = "true/false，运行中掉线是否重连";
  result["reconnect.interval_sec"] = "大于0的浮点数，重连间隔秒";
  result["controls.apply"] = "此开关需要cleanup后修改。";
  result["controls.auto_exposure"] = "true=自动模式3；false=手动模式1";
  result["controls.exposure_time_us"] =
      "1000～62600微秒，步长100；自动时保存为待用值；手动时受帧间隔限制，20fps≤"
      "50000";
  result["controls.auto_white_balance"] = "true 自动；false 使用手动色温";
  result["controls.white_balance_temperature"] =
      "2800～6500 K，步长1；自动时保存为待用值";
  result["controls.brightness"] = "0～255";
  result["controls.contrast"] = "0～95";
  result["controls.saturation"] = "0～100";
  result["controls.hue"] = "-2000～2000，原始值";
  result["controls.gamma"] = "64～300，原始值";
  result["controls.gain"] = "1～8，原始值";
  result["controls.sharpness"] = "1～7";
  result["controls.backlight_compensation"] = "0～2";
  result["controls.power_line_frequency"] = "0=关闭；50=50Hz；60=60Hz";
  return result;
}
class Camera : public rclcpp_lifecycle::LifecycleNode {
  using LP = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>;
  struct Buffer {
    void *ptr;
    size_t length;
  };
  uint32_t stride_ = 0;
  int fd_ = -1;
  bool active_ = false, streaming_ = false;
  std::vector<Buffer> buffers_;
  std::map<std::string, int> settings_;
  std::shared_ptr<LP> raw_;
  rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      info_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostic_timer_;
  OnSetParametersCallbackHandle::SharedPtr callback_;
  sensor_msgs::msg::CameraInfo calibration_;
  std::chrono::steady_clock::time_point last_ =
                                            std::chrono::steady_clock::now(),
                                        retry_ = last_, stats_ = last_;
  uint64_t frames_ = 0, bad_ = 0, reconnects_ = 0, interval_frames_ = 0;
  std::string status_ = "unconfigured";
  template <class T> void declare(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = parameter_descriptions()[name];
    declare_parameter(name, value, d);
  }
  template <class T> T p(const std::string &name) {
    return get_parameter(name).get_value<T>();
  }
  void io(unsigned long request, void *arg) {
    int result;
    do {
      result = ioctl(fd_, request, arg);
    } while (result < 0 && errno == EINTR);
    if (result < 0)
      throw std::runtime_error(std::string("V4L2: ") + strerror(errno));
  }
  void close_device() {
    if (fd_ >= 0 && streaming_) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    streaming_ = false;
    for (auto &b : buffers_) {
      munmap(b.ptr, b.length);
    }
    buffers_.clear();
    if (fd_ >= 0)
      close(fd_);
    fd_ = -1;
  }
  void validate_settings(const std::map<std::string, int> &values) {
    if (!values.at("auto_exposure") &&
        values.at("exposure_time_us") > 1000000.0 / p<double>("fps"))
      throw std::runtime_error("exposure exceeds requested frame interval");
    for (auto &entry : values) {
      auto spec = controls.at(entry.first);
      int v = entry.second;

      if (entry.first == "auto_exposure")
        v = v ? 3 : 1;
      if (entry.first == "power_line_frequency") {
        if (v != 0 && v != 50 && v != 60)
          throw std::runtime_error("power_line_frequency must be 0, 50 or 60");
        v = v == 50 ? 1 : v == 60 ? 2 : 0;
      }
      if (v % spec.scale) {
        throw std::runtime_error(entry.first + ": invalid step");
      }
      v /= spec.scale;
      v4l2_queryctrl q{};
      q.id = spec.id;
      io(VIDIOC_QUERYCTRL, &q);
      if (q.flags & V4L2_CTRL_FLAG_DISABLED)
        throw std::runtime_error(entry.first + ": disabled");
      if (v < q.minimum || v > q.maximum ||
          (q.step && (v - q.minimum) % q.step))
        throw std::runtime_error(entry.first + ": outside device range");
    }
  }
  void write_settings(const std::map<std::string, int> &values) {
    if (!p<bool>("controls.apply"))
      return;
    validate_settings(values);
    std::vector<std::pair<uint32_t, int>> undo;
    try {
      std::vector<std::string> order = {"auto_exposure", "auto_white_balance"};
      for (auto &v : values)
        if (v.first != "auto_exposure" && v.first != "auto_white_balance")
          order.push_back(v.first);
      for (auto &name : order) {
        auto it = values.find(name);
        if (it == values.end())
          continue;
        if ((name == "exposure_time_us" && values.at("auto_exposure")) ||
            (name == "white_balance_temperature" &&
             values.at("auto_white_balance")))
          continue;
        int value = it->second;
        auto spec = controls.at(name);
        if (name == "auto_exposure")
          value = value ? 3 : 1;
        if (name == "power_line_frequency") {
          value = value == 50 ? 1 : value == 60 ? 2 : 0;
        }
        value /= spec.scale;
        v4l2_control c{};
        c.id = spec.id;
        io(VIDIOC_G_CTRL, &c);
        undo.emplace_back(c.id, c.value);
        c.value = value;
        io(VIDIOC_S_CTRL, &c);
        io(VIDIOC_G_CTRL, &c);
        if (c.value != value)
          throw std::runtime_error(name + ": device readback differs");
      }
    } catch (...) {
      for (auto it = undo.rbegin(); it != undo.rend(); ++it) {
        v4l2_control c{};
        c.id = it->first;
        c.value = it->second;
        ioctl(fd_, VIDIOC_S_CTRL, &c);
      }
      throw;
    }
  }
  void open_device() {
    try {
      fd_ = open(p<std::string>("device").c_str(), O_RDWR | O_NONBLOCK);
      if (fd_ < 0)
        throw std::runtime_error(strerror(errno));
      v4l2_capability cap{};
      io(VIDIOC_QUERYCAP, &cap);
      auto flags = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps
                                                           : cap.capabilities;
      if (!(flags & V4L2_CAP_VIDEO_CAPTURE) || !(flags & V4L2_CAP_STREAMING))
        throw std::runtime_error("not a streaming capture device");
      auto format = p<std::string>("pixel_format");
      if (format != "MJPG" && format != "YUYV")
        throw std::runtime_error("pixel_format must be MJPG or YUYV");
      v4l2_format f{};
      f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      f.fmt.pix.width = p<int>("width");
      f.fmt.pix.height = p<int>("height");
      f.fmt.pix.pixelformat =
          format == "MJPG" ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
      f.fmt.pix.field = V4L2_FIELD_ANY;
      auto requested = f.fmt.pix;
      io(VIDIOC_S_FMT, &f);
      stride_ = f.fmt.pix.bytesperline;
      if (format == "YUYV" && !stride_)
        stride_ = requested.width * 2;
      if (f.fmt.pix.width != requested.width ||
          f.fmt.pix.height != requested.height ||
          f.fmt.pix.pixelformat != requested.pixelformat)
        throw std::runtime_error("device rejected requested image mode");
      v4l2_streamparm rate{};
      rate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      rate.parm.capture.timeperframe.numerator = 1000;
      rate.parm.capture.timeperframe.denominator =
          std::lround(p<double>("fps") * 1000);
      io(VIDIOC_S_PARM, &rate);
      auto t = rate.parm.capture.timeperframe;
      if (!t.numerator || std::abs(double(t.denominator) / t.numerator -
                                   p<double>("fps")) > 0.01)
        throw std::runtime_error("device rejected requested fps");
      write_settings(settings_);
      v4l2_requestbuffers req{};
      req.count = 4;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_MMAP;
      io(VIDIOC_REQBUFS, &req);
      if (req.count < 2)
        throw std::runtime_error("insufficient capture buffers");
      for (unsigned i = 0; i < req.count; i++) {
        v4l2_buffer b{};
        b.type = req.type;
        b.memory = req.memory;
        b.index = i;
        io(VIDIOC_QUERYBUF, &b);
        void *ptr = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                         fd_, b.m.offset);
        if (ptr == MAP_FAILED)
          throw std::runtime_error("mmap failed");
        buffers_.push_back({ptr, b.length});
      }
      status_ = "configured";
    } catch (...) {
      close_device();
      throw;
    }
  }
  void stream() {
    for (unsigned i = 0; i < buffers_.size(); i++) {
      v4l2_buffer b{};
      b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = i;
      io(VIDIOC_QBUF, &b);
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    io(VIDIOC_STREAMON, &type);
    streaming_ = true;
    last_ = std::chrono::steady_clock::now();
    status_ = "streaming";
  }
  void load_calibration() {
    calibration_ = sensor_msgs::msg::CameraInfo();
    calibration_.width = p<int>("width");
    calibration_.height = p<int>("height");
    auto url = p<std::string>("camera_info_url");
    if (url.empty())
      return;
    if (url.rfind("package://", 0) == 0) {
      auto slash = url.find('/', 10);
      if (slash == std::string::npos)
        throw std::runtime_error("invalid package URL");
      url = ament_index_cpp::get_package_share_directory(
                url.substr(10, slash - 10)) +
            url.substr(slash);
    } else if (url.rfind("file://", 0) == 0)
      url = url.substr(7);
    else
      throw std::runtime_error(
          "camera_info_url requires file:// or package://");
    auto y = YAML::LoadFile(url);
    if (y["image_width"].as<int>() != p<int>("width") ||
        y["image_height"].as<int>() != p<int>("height"))
      throw std::runtime_error("calibration resolution differs");
    calibration_.distortion_model = y["distortion_model"].as<std::string>();
    calibration_.d =
        y["distortion_coefficients"]["data"].as<std::vector<double>>();
    auto copy = [&](const char *key, auto &dest) {
      auto values = y[key]["data"].as<std::vector<double>>();
      if (values.size() != dest.size())
        throw std::runtime_error("invalid calibration matrix");
      std::copy(values.begin(), values.end(), dest.begin());
    };
    copy("camera_matrix", calibration_.k);
    copy("rectification_matrix", calibration_.r);
    copy("projection_matrix", calibration_.p);
  }
  void tick() {
    if (!active_) {
      return;
    }
    auto now = std::chrono::steady_clock::now();
    if (fd_ < 0) {
      if (!p<bool>("reconnect.enabled") || now < retry_)
        return;
      retry_ =
          now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(
                        p<double>("reconnect.interval_sec")));
      try {
        open_device();
        stream();
        ++reconnects_;
      } catch (const std::exception &e) {
        status_ = e.what();
      }
      return;
    }
    try {
      v4l2_buffer b{};
      b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      if (ioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
        if (errno == EAGAIN) {
          if (now - last_ >
              std::chrono::milliseconds(p<int>("capture.timeout_ms")))
            throw std::runtime_error("capture timeout");
          return;
        }
        if (errno == EINTR)
          return;
        throw std::runtime_error(strerror(errno));
      }
      if (b.index >= buffers_.size())
        throw std::runtime_error("invalid buffer index");
      auto &buf = buffers_[b.index];
      last_ = now;
      auto stamp = this->now();
      if (b.bytesused > buf.length || !b.bytesused ||
          (b.flags & V4L2_BUF_FLAG_ERROR)) {
        ++bad_;
        io(VIDIOC_QBUF, &b);
        return;
      }
      try {
        cv::Mat image;
        bool mjpg = p<std::string>("pixel_format") == "MJPG";
        bool want_raw =
            p<bool>("publish_raw") && raw_->get_subscription_count() > 0;
        bool want_jpeg = p<bool>("publish_compressed") &&
                         jpeg_->get_subscription_count() > 0;
        if (want_raw || (want_jpeg && !mjpg)) {
          if (mjpg) {
            cv::Mat bytes(1, b.bytesused, CV_8UC1, buf.ptr);
            image = cv::imdecode(bytes, cv::IMREAD_COLOR);
            if (image.empty() || image.cols != p<int>("width") ||
                image.rows != p<int>("height"))
              throw std::runtime_error("invalid JPEG frame");
          } else {
            size_t minimum =
                size_t(stride_) * (p<int>("height") - 1) + p<int>("width") * 2;
            if (b.bytesused < minimum)
              throw std::runtime_error("short YUYV frame");
            cv::Mat yuyv(p<int>("height"), p<int>("width"), CV_8UC2, buf.ptr,
                         stride_);
            cv::cvtColor(yuyv, image, cv::COLOR_YUV2BGR_YUY2);
          }
        }
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = p<std::string>("frame_id");
        if (want_raw) {
          sensor_msgs::msg::Image msg;
          msg.header = header;
          msg.width = image.cols;
          msg.height = image.rows;
          msg.encoding = "bgr8";
          msg.step = image.cols * 3;
          msg.data.assign(image.data, image.data + image.total() * 3);
          raw_->publish(msg);
        }
        if (want_jpeg) {
          sensor_msgs::msg::CompressedImage msg;
          msg.header = header;
          msg.format = "bgr8; jpeg compressed bgr8";
          if (mjpg)
            msg.data.assign(static_cast<uint8_t *>(buf.ptr),
                            static_cast<uint8_t *>(buf.ptr) + b.bytesused);
          else
            cv::imencode(".jpg", image, msg.data,
                         {cv::IMWRITE_JPEG_QUALITY, p<int>("jpeg_quality")});
          jpeg_->publish(msg);
        }
        calibration_.header = header;
        info_->publish(calibration_);
        ++frames_;
        ++interval_frames_;
      } catch (const std::exception &e) {
        ++bad_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Frame dropped: %s", e.what());
      }
      io(VIDIOC_QBUF, &b);
    } catch (const std::exception &e) {
      status_ = e.what();
      RCLCPP_ERROR(get_logger(), "Capture stopped: %s", e.what());
      close_device();
      retry_ = now;
    }
  }
  void diagnostics() {
    auto now = std::chrono::steady_clock::now();
    double seconds = std::chrono::duration<double>(now - stats_).count();
    double fps = interval_frames_ / seconds;
    interval_frames_ = 0;
    stats_ = now;
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = get_node_base_interface()->get_fully_qualified_name();
    s.hardware_id =
        p<std::string>("camera_name") + ":" + p<std::string>("device");
    s.level = fd_ < 0 && status_ != "unconfigured" && status_ != "inactive" &&
                      status_ != "shutdown"
                  ? 2
                  : 0;
    s.message = status_;
    auto add = [&](std::string key, std::string value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      s.values.push_back(kv);
    };
    add("capture_fps", std::to_string(fps));
    add("frames", std::to_string(frames_));
    add("bad_frames", std::to_string(bad_));
    add("reconnects", std::to_string(reconnects_));
    add("calibrated", calibration_.k[0] != 0 ? "true" : "false");
    if (fd_ >= 0 && p<bool>("controls.apply")) {
      for (auto &entry : controls) {
        v4l2_control c{};
        c.id = entry.second.id;
        if (ioctl(fd_, VIDIOC_G_CTRL, &c) == 0)
          add("device_readback." + entry.first,
              std::to_string(c.value * entry.second.scale));
      }
    }

    for (auto &entry : settings_) {
      add("requested." + entry.first, std::to_string(entry.second));
    }
    msg.status.push_back(s);
    diagnostics_->publish(msg);
  }

public:
  Camera() : LifecycleNode("endoscope") {
    declare("autostart", true);
    declare(
        "device",
        std::string("/dev/v4l/by-id/usb-DP-251024-J_HD_WebCam-video-index0"));
    declare("pixel_format", std::string("MJPG"));
    declare("width", 1920);
    declare("height", 1080);
    declare("fps", 20.0);
    declare("camera_name", std::string("endoscope"));
    declare("frame_id", std::string("endoscope_optical_frame"));
    declare("camera_info_url", std::string(""));
    declare("publish_raw", true);
    declare("publish_compressed", true);
    declare("jpeg_quality", 90);
    declare("qos.reliability", std::string("best_effort"));
    declare("qos.depth", 3);
    declare("capture.timeout_ms", 2000);
    declare("reconnect.enabled", true);
    declare("reconnect.interval_sec", 2.0);
    declare("controls.apply", true);
    declare("controls.auto_exposure", true);
    declare("controls.auto_white_balance", true);
    for (auto &e :
         std::map<std::string, int>{{"exposure_time_us", 15600},
                                    {"white_balance_temperature", 4600},
                                    {"brightness", 128},
                                    {"contrast", 6},
                                    {"saturation", 52},
                                    {"hue", 0},
                                    {"gamma", 100},
                                    {"gain", 1},
                                    {"sharpness", 2},
                                    {"backlight_compensation", 0},
                                    {"power_line_frequency", 50}})
      declare("controls." + e.first, e.second);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    diagnostic_timer_ = create_wall_timer(1s, [this] { diagnostics(); });
    timer_ = create_wall_timer(1ms, [this] { tick(); });
    callback_ = add_on_set_parameters_callback([this](const std::vector<
                                                      rclcpp::Parameter>
                                                          &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      try {
        auto pending = settings_;
        for (auto &v : parameters) {
          auto name = v.get_name();
          if (has_parameter(name) &&
              v.get_type() != get_parameter(name).get_type())
            throw std::runtime_error("parameter type mismatch: " + name);
          if (name.rfind("controls.", 0) == 0 && name != "controls.apply") {
            if (!p<bool>("controls.apply"))
              throw std::runtime_error(
                  "controls.apply is disabled; cleanup and enable it first");
            auto key = name.substr(9);
            if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
                (v.as_int() < std::numeric_limits<int>::min() ||
                 v.as_int() > std::numeric_limits<int>::max()))
              throw std::runtime_error(
                  "control integer outside supported representation");
            pending.at(key) =
                v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL
                    ? v.as_bool()
                    : v.as_int();
          } else if (name != "use_sim_time" && get_current_state().id() != 1)
            throw std::runtime_error(
                "non-control parameters require cleanup first");
        }
        if (fd_ >= 0) {
          write_settings(pending);
        }
        settings_ = pending;
        result.successful = true;
      } catch (const std::exception &e) {
        result.reason = e.what();
      }
      return result;
    });
    for (auto &e : controls) {
      auto v = get_parameter("controls." + e.first);
      settings_[e.first] = v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL
                               ? v.as_bool()
                               : v.as_int();
    }
  }
  ~Camera() { close_device(); }
  CB on_configure(const rclcpp_lifecycle::State &) override {
    try {
      if (p<int>("width") <= 0 || p<int>("height") <= 0 ||
          !std::isfinite(p<double>("fps")) || p<double>("fps") <= 0 ||
          p<double>("fps") > 1000 || p<int>("qos.depth") <= 0 ||
          p<int>("capture.timeout_ms") <= 0 ||
          !std::isfinite(p<double>("reconnect.interval_sec")) ||
          p<double>("reconnect.interval_sec") <= 0 ||
          p<int>("jpeg_quality") < 1 || p<int>("jpeg_quality") > 100)
        throw std::runtime_error("invalid capture/software parameter");
      auto reliability = p<std::string>("qos.reliability");
      if (reliability != "best_effort" && reliability != "reliable")
        throw std::runtime_error("invalid QoS reliability");
      load_calibration();
      open_device();
      rclcpp::QoS qos(p<int>("qos.depth"));
      if (reliability == "best_effort")
        qos.best_effort();
      raw_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
      jpeg_ = create_publisher<sensor_msgs::msg::CompressedImage>(
          "~/image_raw/compressed", qos);
      info_ =
          create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);
      return CB::SUCCESS;
    } catch (const std::exception &e) {
      status_ = e.what();
      close_device();
      RCLCPP_ERROR(get_logger(), "Configure failed: %s", e.what());
      return CB::FAILURE;
    }
  }
  CB on_activate(const rclcpp_lifecycle::State &) override {
    try {
      if (fd_ < 0)
        open_device();
      stream();
      raw_->on_activate();
      jpeg_->on_activate();
      info_->on_activate();
      active_ = true;
      return CB::SUCCESS;
    } catch (const std::exception &e) {
      status_ = e.what();
      close_device();
      return CB::FAILURE;
    }
  }
  CB on_deactivate(const rclcpp_lifecycle::State &) override {
    active_ = false;
    close_device();
    raw_->on_deactivate();
    jpeg_->on_deactivate();
    info_->on_deactivate();
    status_ = "inactive";
    return CB::SUCCESS;
  }
  CB on_cleanup(const rclcpp_lifecycle::State &) override {
    active_ = false;
    close_device();
    raw_.reset();
    jpeg_.reset();
    info_.reset();
    status_ = "unconfigured";
    return CB::SUCCESS;
  }
  CB on_shutdown(const rclcpp_lifecycle::State &) override {
    active_ = false;
    close_device();
    status_ = "shutdown";
    return CB::SUCCESS;
  }
};
int main(int argc, char **argv) {
  // DDS must be configured before rclcpp initializes its context. The package
  // environment hook configures subscribers as well; this fallback covers
  // invoking the executable without sourcing the package hook.
  const auto profile =
      ament_index_cpp::get_package_share_directory("endoscope_device") +
      "/config/fastdds_local.xml";
  if (!std::getenv("RMW_IMPLEMENTATION"))
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
  if (!std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE"))
    setenv("FASTRTPS_DEFAULT_PROFILES_FILE", profile.c_str(), 1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Camera>();
  if (node->get_parameter("autostart").as_bool()) {
    node->configure();
    if (node->get_current_state().id() == 2)
      node->activate();
  }
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
