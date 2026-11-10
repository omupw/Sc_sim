#include "rm_aim_sim_camera.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace io::rm_aim_sim
{

Camera::Camera(const std::string & config_path)
{
  int timeout_ms = 8000;

  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["camera_matrix"]) {
      configured_camera_matrix_ = yaml["camera_matrix"].as<std::vector<double>>();
    }
    if (yaml["rm_aim_sim"]) {
      const auto & sim = yaml["rm_aim_sim"];
      if (sim["channel"]) channel_ = sim["channel"].as<std::string>();
      if (sim["connect_timeout_ms"]) timeout_ms = sim["connect_timeout_ms"].as<int>();
    }
  } catch (const YAML::Exception & e) {
    tools::logger()->warn("[RmSimCamera] 读取配置失败: {}", e.what());
  }

  if (channel_.empty()) channel_ = "default";
  client_ = Client::instance(channel_, timeout_ms);
  tools::logger()->info("[RmSimCamera] 已连接 rm_aim_sim 通道 {}", channel_);
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  nlohmann::json metadata;
  client_->read_frame(img, metadata, timestamp);

  if (!checked_calibration_) {
    checked_calibration_ = true;
    check_calibration(metadata, img);
  }
}

void Camera::check_calibration(const nlohmann::json & metadata, const cv::Mat & image)
{
  try {
    const auto & camera = metadata.at("camera");
    const int width = camera.at("width").get<int>();
    const int height = camera.at("height").get<int>();
    const auto k = camera.at("K").get<std::vector<double>>();
    const double fx = k.at(0);
    const double cx = k.at(2);
    const double cy = k.at(5);

    tools::logger()->info(
      "[RmSimCamera] 首帧 {}x{}，fx={:.3f}，cx={:.3f}，cy={:.3f}", width, height, fx, cx, cy);

    if (configured_camera_matrix_.size() != 9) {
      tools::logger()->warn(
        "[RmSimCamera] yaml 中没有有效的 camera_matrix，PnP 解算无法与仿真相机对齐");
      return;
    }

    const double cfg_fx = configured_camera_matrix_[0];
    const double cfg_cx = configured_camera_matrix_[2];
    const double cfg_cy = configured_camera_matrix_[5];
    const double tolerance = 1e-3;

    const bool size_matches = std::abs(cfg_cx - cx) <= 0.5 && std::abs(cfg_cy - cy) <= 0.5 &&
                              std::abs(cfg_cx * 2.0 - width) <= 1.0 &&
                              std::abs(cfg_cy * 2.0 - height) <= 1.0;
    const bool focal_matches =
      cfg_fx > 1.0 && std::abs(cfg_fx - fx) / cfg_fx <= tolerance;

    if (!size_matches || !focal_matches) {
      tools::logger()->warn(
        "[RmSimCamera] yaml 内参与仿真不一致（yaml fx={:.3f} cx={:.3f} cy={:.3f}），"
        "请同步 camera_matrix 或恢复仿真相机分辨率/FOV",
        cfg_fx, cfg_cx, cfg_cy);
    } else {
      tools::logger()->info("[RmSimCamera] yaml 内参与仿真相机一致");
    }

    if (image.cols != width || image.rows != height) {
      tools::logger()->warn(
        "[RmSimCamera] BGR 图像尺寸 {}x{} 与元数据 {}x{} 不一致", image.cols, image.rows, width,
        height);
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[RmSimCamera] 无法校验相机标定: {}", e.what());
  }
}

}  // namespace io::rm_aim_sim
