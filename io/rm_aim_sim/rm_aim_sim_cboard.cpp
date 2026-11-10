#include "rm_aim_sim_cboard.hpp"

#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace io::rm_aim_sim
{
namespace
{
using std::chrono::steady_clock;
constexpr auto SEND_WARNING_INTERVAL = std::chrono::seconds(1);

int json_int(const nlohmann::json & object, const char * key, int fallback)
{
  try {
    return object.at(key).get<int>();
  } catch (...) {
    return fallback;
  }
}
}  // namespace

CBoard::CBoard(const std::string & config_path)
{
  std::string channel = "default";
  int timeout_ms = 8000;

  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["bullet_speed"]) default_bullet_speed_ = yaml["bullet_speed"].as<double>();
    if (yaml["use_default_bullet_speed"])
      use_default_bullet_speed_ = yaml["use_default_bullet_speed"].as<bool>();
    if (yaml["rm_aim_sim"]) {
      const auto & sim = yaml["rm_aim_sim"];
      if (sim["channel"]) channel = sim["channel"].as<std::string>();
      if (sim["connect_timeout_ms"]) timeout_ms = sim["connect_timeout_ms"].as<int>();
    }
  } catch (const YAML::Exception & e) {
    tools::logger()->warn("[RmSimCBoard] 读取配置失败: {}", e.what());
  }

  bullet_speed = default_bullet_speed_;
  client_ = Client::instance(channel, timeout_ms);
  tools::logger()->info("[RmSimCBoard] 已连接 rm_aim_sim 通道 {}", channel);
}

CBoard::~CBoard()
{
  try {
    client_->send(false, false, 0.0, 0.0);
  } catch (...) {
  }
}

void CBoard::refresh()
{
  const auto state = client_->state();
  if (state.empty() || !client_->fresh()) {
    mode = Mode::idle;
    return;
  }

  try {
    publisher_id_ = state.at("publisher_id").get<std::string>();
    session_id_ = state.at("session_id").get<std::string>();
    sim_time_ = state.at("sim_time").get<double>();

    const bool enabled = state.at("ipc").at("enabled").get<bool>();
    const bool paused = state.at("paused").get<bool>();
    mode = (enabled && !paused) ? Mode::auto_aim : Mode::idle;

    double speed = default_bullet_speed_;
    if (!use_default_bullet_speed_) {
      const double measured = state.at("weapon").at("speed").get<double>();
      if (std::isfinite(measured) && measured >= 1.0 && measured <= 100.0) speed = measured;
    }
    bullet_speed = speed;

    const auto & gimbal = state.at("gimbal");
    feedback_yaw_ = gimbal.at("yaw").get<float>();
    feedback_pitch_ = gimbal.at("pitch").get<float>();
    feedback_yaw_vel_ = gimbal.at("yaw_vel").get<float>();
    feedback_pitch_vel_ = gimbal.at("pitch_vel").get<float>();
    yaw_vel = feedback_yaw_vel_;
    pitch_vel = feedback_pitch_vel_;

    const auto & stats = state.at("stats");
    stats_.fired = json_int(stats, "fired", 0);
    stats_.hit = json_int(stats, "hit", 0);
    stats_.resolved = json_int(stats, "resolved", 0);
    stats_.in_flight = json_int(stats, "in_flight", 0);
    stats_.damage = json_int(stats, "damage", 0);
    stats_.kills = json_int(stats, "kills", 0);
    try {
      stats_.hit_rate = stats.at("hit_rate").get<double>();
    } catch (...) {
      stats_.hit_rate = stats_.resolved > 0
                          ? static_cast<double>(stats_.hit) / stats_.resolved
                          : 0.0;
    }
  } catch (const std::exception & e) {
    mode = Mode::idle;
    tools::logger()->debug("[RmSimCBoard] 状态字段解析失败: {}", e.what());
  }
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  Eigen::Quaterniond q;
  // 与图像同帧的四元数优先，避免云台转动时 PnP 使用了稍晚的姿态。
  if (client_->frame_quaternion(timestamp, q)) return q;
  return client_->imu_at(timestamp);
}

void CBoard::send(Command command)
{
  try {
    client_->send(
      command.control, command.shoot, command.yaw, command.pitch, command.yaw_vel,
      command.pitch_vel, 0.0, 0.0);
  } catch (const std::exception & e) {
    const auto now = steady_clock::now();
    if (now - last_send_warning_ >= SEND_WARNING_INTERVAL) {
      last_send_warning_ = now;
      tools::logger()->warn("[RmSimCBoard] 指令发送失败: {}", e.what());
    }
  }
}

bool CBoard::state_fresh() const { return client_->fresh(); }

}  // namespace io::rm_aim_sim
