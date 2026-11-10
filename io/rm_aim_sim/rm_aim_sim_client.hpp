#ifndef IO__RM_AIM_SIM__CLIENT_HPP
#define IO__RM_AIM_SIM__CLIENT_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

namespace io::rm_aim_sim
{

/**
 * @brief 与 rm_aim_sim 仿真通过本机共享内存通信的参考客户端。
 *
 * 协议版本 rm_aim_sim/2，数据位于 /dev/shm/rm_aim_sim_<channel>/。
 * 同一通道只允许一个客户端，因此 Camera 和 CBoard 适配器必须通过
 * Client::instance() 共享同一个连接，不能各自创建。
 *
 * 线程模型：
 * - 构造时启动一个后台线程读取 state.json，维护最新状态与四元数样本；
 * - read_frame() 只能在采图线程调用，内部保证同时最多一个未完成请求；
 * - send() 可从控制线程调用，序号与会话状态加锁保护。
 */
class Client
{
public:
  using Clock = std::chrono::steady_clock;

  /// 获取（必要时创建）某通道的共享客户端。目录未就绪时最多等待 timeout_ms。
  static std::shared_ptr<Client> instance(const std::string & channel, int timeout_ms = 8000);

  ~Client();

  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  /// 最新状态快照；尚未收到发布时返回空对象。
  nlohmann::json state() const;

  bool has_state() const;

  /// 最近一次收到“新 publication”的时间不超过 max_age。
  bool fresh(std::chrono::milliseconds max_age = std::chrono::milliseconds(250)) const;

  /// 状态新鲜、未被暂停且外部控制已开启。
  bool remote_ready() const;

  std::string session_id() const;
  std::string publisher_id() const;

  /// 读取实际云台反馈，失败返回 false。
  bool gimbal_feedback(double & yaw, double & pitch, double & yaw_vel, double & pitch_vel) const;

  /// 取 state.weapon.speed；无效时返回 fallback。
  double bullet_speed(double fallback) const;

  /// 若 t 对应最近一帧图像，返回该帧采样时刻的四元数（w,x,y,z）。
  bool frame_quaternion(Clock::time_point t, Eigen::Quaterniond & q) const;

  /// 按四元数样本接收时间插值。
  Eigen::Quaterniond imu_at(Clock::time_point t) const;

  /**
   * @brief 发布绝对目标角与开火许可。
   *
   * 状态不新鲜、暂停或外部控制关闭时自动把 control 置为 false；
   * fire 只在 control 有效时为 true。数值非法抛出异常。
   * @return 是否已有可用状态并完成写入。
   */
  bool send(
    bool control, bool fire, double yaw, double pitch, double yaw_vel = 0.0,
    double pitch_vel = 0.0, double yaw_acc = 0.0, double pitch_acc = 0.0);

  /**
   * @brief 阻塞请求并读取一帧，转换为 BGR。
   *
   * @param bgr      输出 CV_8UC3 BGR 图像
   * @param metadata 输出同帧 frame.json 元数据
   * @param t        输出客户端单调接收时刻
   * @param timeout  等待超时
   * @throw std::runtime_error 超时或协议/文件校验失败
   */
  bool read_frame(
    cv::Mat & bgr, nlohmann::json & metadata, Clock::time_point & t,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));

private:
  explicit Client(std::string channel, int timeout_ms);

  void poll_loop();
  void poll_once();
  void clear_pending_request();
  void clear_imu_samples();
  void push_imu_sample(const nlohmann::json & state, Clock::time_point received);

  nlohmann::json read_json(const char * filename) const;
  void write_json(const char * filename, const nlohmann::json & value) const;
  static void read_all(int fd, void * output, std::size_t bytes);

  int directory_ = -1;
  int lock_ = -1;
  std::string channel_;
  std::string token_;

  std::thread poll_thread_;
  std::atomic<bool> quit_{false};

  mutable std::mutex state_mutex_;
  nlohmann::json state_ = nlohmann::json::object();
  Clock::time_point last_state_time_{};
  bool has_state_ = false;
  std::string context_key_;

  mutable std::mutex imu_mutex_;
  std::deque<std::pair<Clock::time_point, Eigen::Quaterniond>> imu_samples_;

  std::mutex send_mutex_;
  int64_t sequence_ = 0;
  std::string sequence_session_;

  mutable std::mutex frame_mutex_;
  std::string request_id_;
  uint64_t request_count_ = 0;
  bool has_frame_quaternion_ = false;
  Eigen::Quaterniond frame_quaternion_ = Eigen::Quaterniond::Identity();
  Clock::time_point frame_time_{};
};

}  // namespace io::rm_aim_sim

#endif  // IO__RM_AIM_SIM__CLIENT_HPP
