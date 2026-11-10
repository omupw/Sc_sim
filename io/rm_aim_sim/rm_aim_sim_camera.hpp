#ifndef IO__RM_AIM_SIM__CAMERA_HPP
#define IO__RM_AIM_SIM__CAMERA_HPP

#include <memory>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "rm_aim_sim_client.hpp"

namespace io::rm_aim_sim
{

/**
 * @brief 从 rm_aim_sim 共享内存读取相机图像的 CameraBase 实现。
 *
 * 通过 yaml 的 rm_aim_sim.channel 选择通道；相机分辨率/内参以每帧
 * frame.json 为准。若 yaml 中的 camera_matrix 与仿真内参不一致，
 * 首次取图会打印警告（不会自动修改坐标）。
 */
class Camera : public CameraBase
{
public:
  explicit Camera(const std::string & config_path);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  void check_calibration(const nlohmann::json & metadata, const cv::Mat & image);

  std::shared_ptr<Client> client_;
  bool checked_calibration_ = false;
  std::vector<double> configured_camera_matrix_;
  std::string channel_;
};

}  // namespace io::rm_aim_sim

#endif  // IO__RM_AIM_SIM__CAMERA_HPP
