#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <boost/asio.hpp>

#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <array>

using boost::asio::ip::udp;

namespace
{
static inline std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  cur.reserve(s.size());
  for (char c : s) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

static inline std::string strip_checksum_and_dollar(std::string s)
{
  // remove leading '$'
  if (!s.empty() && s.front() == '$') s.erase(s.begin());
  // cut at '*'
  auto pos = s.find('*');
  if (pos != std::string::npos) s.resize(pos);
  // trim CR/LF
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
  return s;
}

static inline double safe_atof(const std::string & s, double fallback = NAN)
{
  if (s.empty()) return fallback;
  char * end = nullptr;
  double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) return fallback;
  return v;
}

static inline bool is_gpchc_header(const std::string & head)
{
  // 兼容 "GPCHC" 或 "$GPCHC"（此处已 strip 掉 '$'，所以一般是 "GPCHC"）
  return (head == "GPCHC") || (head.size() >= 5 && head.find("GPCHC") != std::string::npos);
}

struct ChcData
{
  double heading_deg{NAN};
  double pitch_deg{NAN};
  double roll_deg{NAN};

  double ang_vel_x{NAN};
  double ang_vel_y{NAN};
  double ang_vel_z{NAN};

  double lin_acc_x{NAN};
  double lin_acc_y{NAN};
  double lin_acc_z{NAN};
};

static inline std::optional<ChcData> parse_chc(const std::vector<std::string> & f)
{
  // 需要至少到索引 11
  if (f.size() < 12) return std::nullopt;
  if (!is_gpchc_header(f[0])) return std::nullopt;

  ChcData d;
  // 按你 parser.py 的 CHC map（注意：0 是 "GPCHC"，所以从 1 开始是 gps_week）
  d.heading_deg = safe_atof(f[3]);
  d.pitch_deg   = safe_atof(f[4]);
  d.roll_deg    = safe_atof(f[5]);

  d.ang_vel_x = safe_atof(f[6]);
  d.ang_vel_y = safe_atof(f[7]);
  d.ang_vel_z = safe_atof(f[8]);

  d.lin_acc_x = safe_atof(f[9]);
  d.lin_acc_y = safe_atof(f[10]);
  d.lin_acc_z = safe_atof(f[11]);

  return d;
}

// roll/pitch/yaw (rad) -> quaternion (x,y,z,w)
static inline void rpy_to_quat(double roll, double pitch, double yaw,
                              double & qx, double & qy, double & qz, double & qw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
  qw = cr * cp * cy + sr * sp * sy;
}
}  // namespace

class NmeaUdpImuDriverNode final : public rclcpp::Node
{
public:
  explicit NmeaUdpImuDriverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("nmea_udp_imu_driver", options),
    io_(),
    socket_(io_)
  {
    // --- params ---
    bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
    port_ = declare_parameter<int>("port", 10110);
    frame_id_ = declare_parameter<std::string>("frame_id", "gps");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu");
    exit_on_bind_fail_ = declare_parameter<bool>("exit_on_bind_fail", true);

    // 与你们 py 对齐：常量重力
    gravity_ = declare_parameter<double>("gravity", 9.80665);

    // --- pub ---
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS{1});

    // --- open UDP ---
    if (!open_udp()) {
      if (exit_on_bind_fail_) {
        RCLCPP_FATAL(get_logger(), "Failed to bind UDP %s:%d, exiting.", bind_address_.c_str(), port_);
        rclcpp::shutdown();
        return;
      }
    }

    // --- timer poll ---
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() { poll_once(); });

    RCLCPP_INFO(get_logger(), "Listening UDP %s:%d -> publish Imu on [%s]",
      bind_address_.c_str(), port_, imu_topic_.c_str());
  }

private:
  bool open_udp()
  {
    boost::system::error_code ec;
    udp::endpoint ep(boost::asio::ip::make_address(bind_address_, ec), static_cast<unsigned short>(port_));
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Invalid bind_address [%s]: %s", bind_address_.c_str(), ec.message().c_str());
      return false;
    }

    socket_.open(udp::v4(), ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "socket open failed: %s", ec.message().c_str());
      return false;
    }

    socket_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    (void)ec;

    socket_.bind(ep, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "UDP bind %s:%d failed: %s",
        bind_address_.c_str(), port_, ec.message().c_str());
      return false;
    }

    socket_.non_blocking(true, ec);
    (void)ec;
    return true;
  }

  void poll_once()
  {
    if (!rclcpp::ok()) return;

    boost::system::error_code ec;
    udp::endpoint sender;

    for (int i = 0; i < 32; ++i) {
      std::array<char, 2048> buf{};
      size_t n = socket_.receive_from(boost::asio::buffer(buf), sender, 0, ec);
      if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
        break;
      }
      if (ec) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "UDP receive error: %s", ec.message().c_str());
        break;
      }

      std::string payload(buf.data(), n);
      handle_payload(payload);
    }
  }

  void handle_payload(const std::string & payload)
  {
    std::istringstream iss(payload);
    std::string line;
    while (std::getline(iss, line)) {
      line = strip_checksum_and_dollar(line);
      if (line.empty()) continue;

      auto fields = split_csv(line);
      if (fields.empty()) continue;

      // 只处理 GPCHC
      auto chc = parse_chc(fields);
      if (!chc) continue;

      publish_imu_from_chc(*chc);
    }
  }

  void publish_imu_from_chc(const ChcData & d)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;

    // ---- orientation (完全对齐你们 py：heading = radians(90 - heading)) ----
    const double yaw   = std::isfinite(d.heading_deg) ? (M_PI / 180.0) * (90.0 - d.heading_deg) : NAN;
    const double pitch = std::isfinite(d.pitch_deg)   ? (M_PI / 180.0) * d.pitch_deg : NAN;
    const double roll  = std::isfinite(d.roll_deg)    ? (M_PI / 180.0) * d.roll_deg : NAN;

    double qx=0, qy=0, qz=0, qw=1;
    if (std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw)) {
      rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw);
    }
    imu.orientation.x = qx;
    imu.orientation.y = qy;
    imu.orientation.z = qz;
    imu.orientation.w = qw;

    // ---- linear_acceleration (对齐你们 py：x=acc_y, y=-acc_x, z=acc_z) ----
    if (std::isfinite(d.lin_acc_x) && std::isfinite(d.lin_acc_y) && std::isfinite(d.lin_acc_z)) {
      imu.linear_acceleration.x = d.lin_acc_y * gravity_;
      imu.linear_acceleration.y = -d.lin_acc_x * gravity_;
      imu.linear_acceleration.z = d.lin_acc_z * gravity_;
    }

    // ---- angular_velocity (对齐你们 py：x=rad(av_y), y=rad(-av_x), z=rad(av_z)) ----
    if (std::isfinite(d.ang_vel_x) && std::isfinite(d.ang_vel_y) && std::isfinite(d.ang_vel_z)) {
      imu.angular_velocity.x = (M_PI / 180.0) * d.ang_vel_y;
      imu.angular_velocity.y = (M_PI / 180.0) * (-d.ang_vel_x);
      imu.angular_velocity.z = (M_PI / 180.0) * d.ang_vel_z;
    }

    // ---- covariance (对齐你们 py：0.01 对角) ----
    imu.angular_velocity_covariance[0] = 0.01;
    imu.angular_velocity_covariance[4] = 0.01;
    imu.angular_velocity_covariance[8] = 0.01;

    imu_pub_->publish(imu);
  }

private:
  // params
  std::string bind_address_;
  int port_{10110};
  std::string frame_id_;
  std::string imu_topic_;
  bool exit_on_bind_fail_{true};
  double gravity_{9.80665};

  // ros
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  // asio
  boost::asio::io_context io_;
  udp::socket socket_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NmeaUdpImuDriverNode)
