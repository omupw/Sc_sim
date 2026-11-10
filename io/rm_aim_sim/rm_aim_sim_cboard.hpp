#ifndef IO__RM_AIM_SIM__CBOARD_HPP
#define IO__RM_AIM_SIM__CBOARD_HPP

#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "rm_aim_sim_client.hpp"

namespace io::rm_aim_sim
{

/**
 * @brief rm_aim_sim 版的 CBoard：把 state.json 当作下位机反馈，
 *        把 command.json 当作下位机控制指令。
 *
 * 与 io::CBoard 保持相同的使用方式：
 *   camera.read(img, t);
 *   cboard.refresh();                 // 本类新增：每个循环先同步一次状态
 *   q    = cboard.imu_at(t);
 *   mode = cboard.mode;               // 外部控制关闭/暂停/断流时为 idle
 *   ...
 *   cboard.send(command);
 *
 * 仿真没有小符/大符模式，因此 mode 只会是 idle 或 auto_aim。
 */
class CBoard
{
public:
  struct Stats
  {
    int fired = 0;
    int hit = 0;
    int resolved = 0;
    int in_flight = 0;
    int damage = 0;
    int kills = 0;
    double hit_rate = 0.0;
  };

  explicit CBoard(const std::string & config_path);
  ~CBoard();

  /// 读取最新仿真状态，更新 mode / bullet_speed / 云台反馈 / 统计。
  /// 每个主循环在 camera.read 之后调用一次即可。
  void refresh();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command);

  float feedback_yaw() const { return feedback_yaw_; }
  float feedback_pitch() const { return feedback_pitch_; }
  float feedback_yaw_vel() const { return feedback_yaw_vel_; }
  float feedback_pitch_vel() const { return feedback_pitch_vel_; }

  const std::string & session_id() const { return session_id_; }
  const std::string & publisher_id() const { return publisher_id_; }
  bool state_fresh() const;
  double sim_time() const { return sim_time_; }
  const Stats & stats() const { return stats_; }
  float bullet_speed_feedback() const { return static_cast<float>(bullet_speed); }

  // 与 io::CBoard 一致的公开字段，方便复用现有上层逻辑。
  double bullet_speed = 21.0;
  double yaw_vel = 0.0;
  double pitch_vel = 0.0;
  Mode mode = Mode::idle;
  ShootMode shoot_mode = ShootMode::left_shoot;
  double ft_angle = 0.0;

private:
  std::shared_ptr<Client> client_;
  std::string publisher_id_;
  std::string session_id_;
  double sim_time_ = 0.0;

  double default_bullet_speed_ = 21.0;
  bool use_default_bullet_speed_ = false;

  float feedback_yaw_ = 0.0F;
  float feedback_pitch_ = 0.0F;
  float feedback_yaw_vel_ = 0.0F;
  float feedback_pitch_vel_ = 0.0F;

  Stats stats_;
  std::chrono::steady_clock::time_point last_send_warning_{};
};

}  // namespace io::rm_aim_sim

#endif  // IO__RM_AIM_SIM__CBOARD_HPP
