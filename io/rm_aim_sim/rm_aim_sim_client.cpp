#include "rm_aim_sim_client.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <map>
#include <regex>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace io::rm_aim_sim
{
namespace
{
using Json = nlohmann::json;
using namespace std::chrono_literals;

constexpr const char * PROTOCOL = "rm_aim_sim/2";
constexpr const char * SIM_ROOT = "/dev/shm/rm_aim_sim_";
constexpr std::size_t MAX_JSON_BYTES = 65536;
constexpr int64_t MAX_SEQUENCE = 9007199254740991LL;  // 2^53 - 1

bool finite_number(double value) { return std::isfinite(value); }

bool valid_channel(const std::string & channel)
{
  static const std::regex pattern("[A-Za-z0-9_-]{1,80}");
  return std::regex_match(channel, pattern);
}

bool all_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
}  // namespace

std::shared_ptr<Client> Client::instance(const std::string & channel, int timeout_ms)
{
  static std::mutex registry_mutex;
  static std::map<std::string, std::weak_ptr<Client>> registry;

  std::lock_guard<std::mutex> lock(registry_mutex);
  auto it = registry.find(channel);
  if (it != registry.end()) {
    if (auto existing = it->second.lock()) return existing;
  }

  auto created = std::shared_ptr<Client>(new Client(channel, timeout_ms));
  registry[channel] = created;
  return created;
}

Client::Client(std::string channel, int timeout_ms) : channel_(std::move(channel))
{
  if (!valid_channel(channel_)) throw std::runtime_error("rm_aim_sim: 非法通道名 " + channel_);

  const std::string path = SIM_ROOT + channel_;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);

  while (true) {
    directory_ = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (directory_ >= 0) break;
    if (Clock::now() >= deadline)
      throw std::runtime_error(
        "rm_aim_sim: 等待仿真通道超时，请先启动仿真并确认 channel 一致：" + path);
    std::this_thread::sleep_for(10ms);
  }

  struct stat info {};
  if (
    ::fstat(directory_, &info) != 0 || info.st_uid != ::getuid() ||
    (info.st_mode & 0777) != 0700) {
    ::close(directory_);
    directory_ = -1;
    throw std::runtime_error("rm_aim_sim: 共享目录必须属于当前用户且权限为 0700");
  }

  lock_ = ::openat(
    directory_, "client.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0) {
    if (lock_ >= 0) ::close(lock_);
    ::close(directory_);
    lock_ = directory_ = -1;
    throw std::runtime_error(
      "rm_aim_sim: 无法锁定 client.lock，同通道可能已有另一个自瞄客户端在运行");
  }

  token_ =
    std::to_string(::getpid()) + "-" + std::to_string(Clock::now().time_since_epoch().count());
  poll_thread_ = std::thread(&Client::poll_loop, this);
}

Client::~Client()
{
  try {
    send(false, false, 0.0, 0.0);
  } catch (...) {
  }

  quit_ = true;
  if (poll_thread_.joinable()) poll_thread_.join();
  if (lock_ >= 0) ::close(lock_);
  if (directory_ >= 0) ::close(directory_);
}

nlohmann::json Client::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

bool Client::has_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_state_;
}

bool Client::fresh(std::chrono::milliseconds max_age) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_state_ && (Clock::now() - last_state_time_) <= max_age;
}

bool Client::remote_ready() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_state_ || (Clock::now() - last_state_time_) > 250ms) return false;

  try {
    return state_.at("ipc").at("enabled").get<bool>() && !state_.at("paused").get<bool>();
  } catch (...) {
    return false;
  }
}

std::string Client::session_id() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  try {
    return state_.at("session_id").get<std::string>();
  } catch (...) {
    return {};
  }
}

std::string Client::publisher_id() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  try {
    return state_.at("publisher_id").get<std::string>();
  } catch (...) {
    return {};
  }
}

bool Client::gimbal_feedback(
  double & yaw, double & pitch, double & yaw_vel, double & pitch_vel) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  try {
    const auto & gimbal = state_.at("gimbal");
    const double y = gimbal.at("yaw").get<double>();
    const double p = gimbal.at("pitch").get<double>();
    const double yv = gimbal.at("yaw_vel").get<double>();
    const double pv = gimbal.at("pitch_vel").get<double>();
    if (!finite_number(y) || !finite_number(p) || !finite_number(yv) || !finite_number(pv))
      return false;
    yaw = y;
    pitch = p;
    yaw_vel = yv;
    pitch_vel = pv;
    return true;
  } catch (...) {
    return false;
  }
}

double Client::bullet_speed(double fallback) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  try {
    const double speed = state_.at("weapon").at("speed").get<double>();
    if (finite_number(speed) && speed >= 1.0 && speed <= 100.0) return speed;
  } catch (...) {
  }
  return fallback;
}

bool Client::frame_quaternion(Clock::time_point t, Eigen::Quaterniond & q) const
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (!has_frame_quaternion_) return false;

  const double delta = std::chrono::duration<double>(t - frame_time_).count();
  if (std::abs(delta) > 0.010) return false;

  q = frame_quaternion_;
  return true;
}

Eigen::Quaterniond Client::imu_at(Clock::time_point t) const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_samples_.empty()) return Eigen::Quaterniond::Identity();
  if (imu_samples_.size() == 1 || t <= imu_samples_.front().first)
    return imu_samples_.front().second;
  if (t >= imu_samples_.back().first) return imu_samples_.back().second;

  auto it = std::lower_bound(
    imu_samples_.begin(), imu_samples_.end(), t,
    [](const auto & sample, Clock::time_point value) { return sample.first < value; });
  const auto & hi = *it;
  const auto & lo = *(it - 1);

  const double span = std::chrono::duration<double>(hi.first - lo.first).count();
  const double elapsed = std::chrono::duration<double>(t - lo.first).count();
  const double k = span > 1e-9 ? std::clamp(elapsed / span, 0.0, 1.0) : 1.0;
  return lo.second.slerp(k, hi.second).normalized();
}

bool Client::send(
  bool control, bool fire, double yaw, double pitch, double yaw_vel, double pitch_vel,
  double yaw_acc, double pitch_acc)
{
  for (double value : {yaw, pitch, yaw_vel, pitch_vel, yaw_acc, pitch_acc}) {
    if (!finite_number(value) || std::abs(value) > 1000.0)
      throw std::runtime_error("rm_aim_sim: 指令包含非法角度或前馈");
  }

  Json snapshot;
  bool state_fresh = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_) return false;
    state_fresh = (Clock::now() - last_state_time_) <= 250ms;
    snapshot = state_;
  }

  bool ready = false;
  std::string session;
  int64_t last_sequence = -1;
  try {
    ready = state_fresh && !snapshot.at("paused").get<bool>() &&
            snapshot.at("ipc").at("enabled").get<bool>();
    session = snapshot.at("session_id").get<std::string>();
    last_sequence = snapshot.at("ipc").at("last_seq").get<int64_t>();
  } catch (...) {
    return false;
  }

  control = control && ready;
  fire = fire && control;

  std::lock_guard<std::mutex> lock(send_mutex_);
  if (sequence_session_ != session) {
    sequence_session_ = session;
    sequence_ = 0;
  }
  sequence_ = std::max(sequence_, last_sequence + 1);
  if (sequence_ > MAX_SEQUENCE) throw std::runtime_error("rm_aim_sim: 指令序号耗尽");

  Json command{
    {"protocol", PROTOCOL},
    {"type", "command"},
    {"publisher_id", snapshot.at("publisher_id")},
    {"session_id", session},
    {"seq", sequence_},
    {"control", control},
    {"fire", fire},
    {"yaw", yaw},
    {"pitch", pitch},
    {"yaw_vel", yaw_vel},
    {"pitch_vel", pitch_vel},
    {"yaw_acc", yaw_acc},
    {"pitch_acc", pitch_acc}};

  write_json("command.json", command);
  ++sequence_;
  return true;
}

bool Client::read_frame(
  cv::Mat & bgr, nlohmann::json & metadata, Clock::time_point & t,
  std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + timeout;

  while (Clock::now() < deadline) {
    if (!fresh()) {
      std::this_thread::sleep_for(2ms);
      continue;
    }

    const Json snapshot = state();
    std::string request;
    bool created = false;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (request_id_.empty()) {
        request_id_ = token_ + "-" + std::to_string(request_count_++);
        created = true;
      }
      request = request_id_;
    }

    if (created) {
      try {
        write_json(
          "request.json",
          {{"publisher_id", snapshot.at("publisher_id")}, {"request_id", request}});
      } catch (const std::exception &) {
        clear_pending_request();
        throw;
      }
    }

    const Json frame = read_json("frame.json");
    if (frame.value("request_id", "") != request) {
      std::this_thread::sleep_for(2ms);
      continue;
    }

    bool retry = false;
    try {
      if (frame.value("protocol", "") != PROTOCOL || frame.value("type", "") != "frame")
        throw std::runtime_error("frame.json 的 protocol/type 不正确");
      if (
        frame.at("publisher_id") != snapshot.at("publisher_id") ||
        frame.at("session_id") != snapshot.at("session_id")) {
        retry = true;
      } else {
        const double age =
          snapshot.at("sim_time").get<double>() - frame.at("sim_time").get<double>();
        if (!std::isfinite(age) || age > 0.2 || age < -0.1) {
          retry = true;
        } else {
          const auto & camera = frame.at("camera");
          const int width = camera.at("width").get<int>();
          const int height = camera.at("height").get<int>();
          const std::string encoding = frame.at("encoding").get<std::string>();
          const int channels = encoding == "rgb8" ? 3 : encoding == "rgba8" ? 4 : 0;

          const std::string filename = "frame-" + request + ".raw";
          if (
            channels == 0 || width < 160 || width > 3840 || height < 120 || height > 2160 ||
            frame.at("frame_file").get<std::string>() != filename)
            throw std::runtime_error("frame.json 的图像布局不合法");

          const std::size_t bytes =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * channels;
          if (frame.at("image_bytes").get<std::size_t>() != bytes)
            throw std::runtime_error("frame.json 的 image_bytes 与宽高/通道数不符");

          const int fd = ::openat(
            directory_, filename.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
          if (fd < 0) throw std::runtime_error("无法打开 " + filename + "（仿真可能已重启）");

          struct stat info {};
          if (
            ::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) ||
            static_cast<std::size_t>(info.st_size) != bytes) {
            ::close(fd);
            throw std::runtime_error("图像文件大小或类型不匹配");
          }

          cv::Mat raw(height, width, CV_MAKETYPE(CV_8U, channels));
          try {
            read_all(fd, raw.data, bytes);
          } catch (...) {
            ::close(fd);
            throw;
          }
          ::close(fd);

          cv::cvtColor(
            raw, bgr, channels == 3 ? cv::COLOR_RGB2BGR : cv::COLOR_RGBA2BGR);

          Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
          bool has_q = false;
          try {
            const auto values =
              frame.at("gimbal").at("quaternion_wxyz").get<std::vector<double>>();
            if (values.size() == 4 && all_finite(values)) {
              q = Eigen::Quaterniond(values[0], values[1], values[2], values[3]).normalized();
              has_q = true;
            }
          } catch (...) {
          }

          const auto received = Clock::now();
          {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            request_id_.clear();
            has_frame_quaternion_ = has_q;
            frame_quaternion_ = q;
            frame_time_ = received;
          }

          metadata = frame;
          t = received;
          return true;
        }
      }
    } catch (const std::exception & e) {
      clear_pending_request();
      throw std::runtime_error(std::string("rm_aim_sim: 帧校验失败: ") + e.what());
    }

    if (retry) {
      clear_pending_request();
      continue;
    }
    std::this_thread::sleep_for(2ms);
  }

  clear_pending_request();
  throw std::runtime_error(
    "rm_aim_sim: 等待相机帧超时。仿真必须使用图形模式（不能 --headless/--no-ipc）");
}

void Client::poll_loop()
{
  while (!quit_) {
    poll_once();
    std::this_thread::sleep_for(2ms);
  }
}

void Client::poll_once()
{
  const Json value = read_json("state.json");
  if (value.value("protocol", "") != PROTOCOL || value.value("type", "") != "state") return;
  if (
    !value.contains("publisher_id") || !value.contains("publication") ||
    !value.contains("wall_unix_sec") || !value.contains("session_id"))
    return;

  try {
    const double wall =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    const double age = wall - value.at("wall_unix_sec").get<double>();
    if (!std::isfinite(age) || age < -0.1 || age > 0.25) return;
  } catch (...) {
    return;
  }

  bool context_changed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (
      has_state_ && value.at("publisher_id") == state_.at("publisher_id") &&
      value.at("publication") == state_.at("publication"))
      return;

    const std::string context =
      value.value("publisher_id", "") + "|" + value.value("session_id", "");
    context_changed = has_state_ && context != context_key_;
    context_key_ = context;
    state_ = value;
    last_state_time_ = Clock::now();
    has_state_ = true;
  }

  if (context_changed) {
    clear_pending_request();
    clear_imu_samples();
  }
  push_imu_sample(value, Clock::now());
}

void Client::clear_pending_request()
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  request_id_.clear();
}

void Client::clear_imu_samples()
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_samples_.clear();
}

void Client::push_imu_sample(const nlohmann::json & state, Clock::time_point received)
{
  try {
    const auto values = state.at("gimbal").at("quaternion_wxyz").get<std::vector<double>>();
    if (values.size() != 4 || !all_finite(values)) return;

    Eigen::Quaterniond q(values[0], values[1], values[2], values[3]);
    if (q.norm() < 1e-9) return;
    q.normalize();

    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_samples_.emplace_back(received, q);
    while (imu_samples_.size() > 256) imu_samples_.pop_front();
  } catch (...) {
  }
}

nlohmann::json Client::read_json(const char * filename) const
{
  const int fd = ::openat(directory_, filename, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) return Json::object();

  struct stat info {};
  if (
    ::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size <= 0 ||
    static_cast<std::size_t>(info.st_size) > MAX_JSON_BYTES) {
    ::close(fd);
    return Json::object();
  }

  std::string data;
  try {
    data.resize(static_cast<std::size_t>(info.st_size));
    read_all(fd, data.data(), data.size());
  } catch (...) {
    ::close(fd);
    return Json::object();
  }
  ::close(fd);

  auto value = Json::parse(data, nullptr, false);
  return value.is_object() ? value : Json::object();
}

void Client::write_json(const char * filename, const nlohmann::json & value) const
{
  const std::string temporary = std::string(filename) + ".client.tmp";
  const int fd = ::openat(
    directory_, temporary.c_str(), O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC | O_NOFOLLOW,
    0600);
  if (fd < 0) throw std::runtime_error("rm_aim_sim: 无法创建临时文件 " + temporary);

  try {
    const auto data = value.dump();
    std::size_t written = 0;
    while (written < data.size()) {
      const auto count = ::write(fd, data.data() + written, data.size() - written);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("rm_aim_sim: 写入临时文件不完整");
      written += static_cast<std::size_t>(count);
    }
  } catch (...) {
    ::close(fd);
    ::unlinkat(directory_, temporary.c_str(), 0);
    throw;
  }

  ::close(fd);
  if (::renameat(directory_, temporary.c_str(), directory_, filename) != 0)
    throw std::runtime_error("rm_aim_sim: 原子发布 " + std::string(filename) + " 失败");
}

void Client::read_all(int fd, void * output, std::size_t bytes)
{
  auto data = static_cast<char *>(output);
  while (bytes > 0) {
    const auto count = ::read(fd, data, bytes);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) throw std::runtime_error("rm_aim_sim: 读取文件不完整");
    data += count;
    bytes -= static_cast<std::size_t>(count);
  }
}

}  // namespace io::rm_aim_sim
