// 步兵自瞄的 rm_aim_sim 在线调试入口。
//
// 与 standard_mpc_se 的区别只在 IO 层：
//   - 相机来自 rm_aim_sim 共享内存（camera_name: rm_aim_sim）
//   - 下位机反馈/控制由 io::rm_aim_sim::CBoard 映射到 state.json/command.json
// 检测、PnP、跟踪、IMM、Aimer、Planner、Shooter 全部复用正式自瞄代码。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/rm_aim_sim/rm_aim_sim_cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/sim.yaml | 位置参数，rm_aim_sim 仿真用 yaml 配置路径 }";

namespace
{
struct SimOptions
{
  bool allow_fire = false;
  bool show_window = true;
  double window_scale = 0.5;
  bool enable_target_stabilize = false;
  double target_jump_angle_threshold_deg = 1.0;
  double target_stabilize_alpha = 0.3;
};

SimOptions load_options(const std::string & config_path)
{
  SimOptions options;
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["sim_allow_fire"]) options.allow_fire = yaml["sim_allow_fire"].as<bool>();
    if (yaml["show_debug_window"]) options.show_window = yaml["show_debug_window"].as<bool>();
    if (yaml["debug_window_scale"]) options.window_scale = yaml["debug_window_scale"].as<double>();
    if (yaml["enable_target_stabilize"])
      options.enable_target_stabilize = yaml["enable_target_stabilize"].as<bool>();
    if (yaml["target_jump_angle_threshold_deg"])
      options.target_jump_angle_threshold_deg =
        yaml["target_jump_angle_threshold_deg"].as<double>();
    if (yaml["target_stabilize_alpha"])
      options.target_stabilize_alpha = yaml["target_stabilize_alpha"].as<double>();
  } catch (const YAML::Exception & e) {
    tools::logger()->warn("[standard_sim] 读取配置失败: {}", e.what());
  }

  options.window_scale = std::clamp(options.window_scale, 0.1, 2.0);
  options.target_stabilize_alpha = std::clamp(options.target_stabilize_alpha, 0.0, 1.0);
  options.target_jump_angle_threshold_deg =
    std::max(0.0, options.target_jump_angle_threshold_deg);
  return options;
}

void draw_overlay(
  cv::Mat & img, const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const auto_aim::Solver & solver, double yaw,
  double pitch, double bullet_speed, const io::rm_aim_sim::CBoard & cboard)
{
  tools::draw_text(
    img,
    fmt::format(
      "cmd ctrl={} fire={} yaw={:.2f} pitch={:.2f}", command.control, command.shoot,
      command.yaw * 57.3, command.pitch * 57.3),
    {10, 40}, {154, 50, 205});
  tools::draw_text(
    img, fmt::format("gimbal yaw={:.2f} pitch={:.2f}", yaw * 57.3, pitch * 57.3), {10, 80},
    {255, 255, 255});
  tools::draw_text(
    img, fmt::format("bullet_speed={:.2f} m/s", bullet_speed), {10, 120},
    {255, 255, 255});

  const auto & stats = cboard.stats();
  tools::draw_text(
    img,
    fmt::format(
      "sim fired={} hit={} resolved={} rate={:.1f}% damage={} kills={}", stats.fired,
      stats.hit, stats.resolved, stats.hit_rate * 100.0, stats.damage, stats.kills),
    {10, 160}, {0, 255, 0});

  if (targets.empty()) return;

  const auto & target = targets.front();
  const auto armor_xyza_list = target.armor_xyza_list();
  for (const auto & xyza : armor_xyza_list) {
    const auto image_points =
      solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
    tools::draw_points(img, image_points, {0, 255, 0});
  }

  const auto aim_point = aimer.debug_aim_point;
  if (!aim_point.valid) return;

  const auto aim_points = solver.reproject_armor(
    aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
  tools::draw_points(img, aim_points, {0, 0, 255});

  if (!aim_points.empty()) {
    cv::Point2f center{0.0F, 0.0F};
    for (const auto & point : aim_points) center += point;
    center.x /= static_cast<float>(aim_points.size());
    center.y /= static_cast<float>(aim_points.size());
    if (command.shoot)
      tools::draw_text(img, "FIRE", center + cv::Point2f{-20.0F, 24.0F}, {0, 0, 255});
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const auto options = load_options(config_path);

  tools::Exiter exiter;
  tools::Plotter plotter(config_path);

  io::Camera camera(config_path);
  io::rm_aim_sim::CBoard cboard(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);

  const auto make_tracker = [&config_path, &solver]() {
    return std::make_unique<auto_aim::Tracker>(config_path, solver);
  };
  auto tracker = make_tracker();
  auto aimer = std::make_unique<auto_aim::Aimer>(config_path);
  auto planner = std::make_unique<auto_aim::Planner>(config_path);
  auto shooter = std::make_unique<auto_aim::Shooter>(config_path);

  cv::Mat img;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  steady_clock::time_point t;

  io::Mode mode = io::Mode::idle;
  io::Mode last_mode = io::Mode::idle;
  std::string last_session;
  io::Command last_command{false, false, 0.0, 0.0, 0.0, 0.0};
  int frame_count = 0;
  int total_armors = 0;
  int detected_frames = 0;

  tools::logger()->info(
    "[standard_sim] allow_fire={} show_window={} scale={} stabilize={}",
    options.allow_fire, options.show_window, options.window_scale,
    options.enable_target_stabilize);
  if (!options.allow_fire)
    tools::logger()->warn("[standard_sim] sim_allow_fire=false，只瞄准不开火");

  while (!exiter.exit()) {
    camera.read(img, t);
    cboard.refresh();

    // 暂停/恢复、重置、外部控制切换都会产生新 session：
    // 旧目标与旧跟踪状态必须丢弃，不能跨会话继续控制。
    const auto session = cboard.session_id();
    if (!session.empty() && session != last_session) {
      if (!last_session.empty()) {
        tools::logger()->info("[standard_sim] 新 session {}，清空跟踪状态", session);
        tracker = make_tracker();
        aimer = std::make_unique<auto_aim::Aimer>(config_path);
        planner = std::make_unique<auto_aim::Planner>(config_path);
        shooter = std::make_unique<auto_aim::Shooter>(config_path);
        last_command = {false, false, 0.0, 0.0, 0.0, 0.0};
      }
      last_session = session;
    }

    q = cboard.imu_at(t);
    mode = cboard.mode;

    if (mode != last_mode) {
      tools::logger()->info(
        "[standard_sim] mode -> {} (fresh={})", io::MODES[mode], cboard.state_fresh());
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);

    io::Command command{false, false, 0.0, 0.0, 0.0, 0.0};
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Target> targets;

    if (mode == io::Mode::auto_aim) {
      armors = detector.detect(img);
      total_armors += static_cast<int>(armors.size());
      if (!armors.empty()) ++detected_frames;

      targets = tracker->track(armors, t);
      command = aimer->aim(targets, t, cboard.bullet_speed);

      if (!targets.empty()) {
        auto plan = planner->plan(targets.front(), cboard.bullet_speed);
        if (plan.control) {
          command.control = true;
          command.yaw = plan.yaw;
          command.pitch = plan.pitch;
          command.yaw_vel = plan.yaw_vel;
          command.pitch_vel = plan.pitch_vel;
        }
      }

      const io::Command previous = last_command;
      if (options.enable_target_stabilize && command.control && previous.control) {
        const double delta_yaw =
          std::abs(tools::limit_rad(command.yaw - previous.yaw)) * 57.3;
        const double delta_pitch =
          std::abs(tools::limit_rad(command.pitch - previous.pitch)) * 57.3;
        if (
          delta_yaw > options.target_jump_angle_threshold_deg ||
          delta_pitch > options.target_jump_angle_threshold_deg) {
          command.yaw = (1.0 - options.target_stabilize_alpha) * previous.yaw +
                        options.target_stabilize_alpha * command.yaw;
          command.pitch = (1.0 - options.target_stabilize_alpha) * previous.pitch +
                          options.target_stabilize_alpha * command.pitch;
        }
      }

      const Eigen::Vector3d gimbal_ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
      command.shoot =
        shooter->shoot(command, *aimer, targets, gimbal_ypr) && options.allow_fire;
    }

    // 只有控制有效时才记录上一帧指令，与 standard_mpc_se 的稳定逻辑一致。
    if (command.control && mode == io::Mode::auto_aim) last_command = command;

    cboard.send(command);

    // 调试画面与曲线数据。
    nlohmann::json data;
    data["mode"] = io::MODES[mode];
    data["session"] = cboard.session_id();
    data["sim_time"] = cboard.sim_time();
    data["bullet_speed"] = cboard.bullet_speed;
    data["cmd_control"] = command.control;
    data["cmd_fire"] = command.shoot;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["cmd_pitch"] = command.pitch * 57.3;
    data["cmd_yaw_vel"] = command.yaw_vel;
    data["cmd_pitch_vel"] = command.pitch_vel;
    data["feedback_yaw"] = cboard.feedback_yaw() * 57.3;
    data["feedback_pitch"] = cboard.feedback_pitch() * 57.3;
    data["feedback_yaw_vel"] = cboard.feedback_yaw_vel();
    data["feedback_pitch_vel"] = cboard.feedback_pitch_vel();
    data["armor_num"] = static_cast<int>(armors.size());

    const auto & stats = cboard.stats();
    data["fired"] = stats.fired;
    data["hit"] = stats.hit;
    data["resolved"] = stats.resolved;
    data["in_flight"] = stats.in_flight;
    data["hit_rate"] = stats.hit_rate;
    data["damage"] = stats.damage;
    data["kills"] = stats.kills;

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;
      data["aim_yaw_deg"] =
        aimer->debug_aim_point.valid ? aimer->debug_aim_point.xyza[3] * 57.3 : -999.0;
    }

    plotter.plot(data);

    if (options.show_window) {
      const Eigen::Vector3d ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
      draw_overlay(
        img, command, *aimer, targets, solver, ypr[0], ypr[1], cboard.bullet_speed, cboard);

      cv::Mat shown;
      if (std::abs(options.window_scale - 1.0) > 1e-6)
        cv::resize(img, shown, {}, options.window_scale, options.window_scale);
      else
        shown = img;

      cv::imshow("standard_sim (rm_aim_sim)", shown);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }

    ++frame_count;
  }

  cboard.send({false, false, 0.0, 0.0, 0.0, 0.0});

  if (frame_count > 0) {
    const double average_armors =
      static_cast<double>(total_armors) / static_cast<double>(frame_count);
    const double detection_rate = 100.0 * detected_frames / frame_count;
    tools::logger()->info(
      "[standard_sim] frames={} armors/frame={:.2f} detection_rate={:.2f}% fired={} hit={} "
      "hit_rate={:.2f}%",
      frame_count, average_armors, detection_rate, cboard.stats().fired, cboard.stats().hit,
      cboard.stats().hit_rate * 100.0);
  }

  return 0;
}
