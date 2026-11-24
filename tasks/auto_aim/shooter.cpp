#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  if (yaml["aim_tolerance_ratio"]) aim_tolerance_ratio_ = yaml["aim_tolerance_ratio"].as<double>();
  // 目标自转（小陀螺）时装甲切换频繁，云台在换板过程中会短暂偏离，
  // 需要更严格的开火条件；纯平移时云台存在稳态滞后，收紧容差会直接打不出弹。
  aim_tolerance_ratio_spin_ = aim_tolerance_ratio_;
  if (yaml["aim_tolerance_ratio_spin"])
    aim_tolerance_ratio_spin_ = yaml["aim_tolerance_ratio_spin"].as<double>();
  if (yaml["aim_spin_speed"]) aim_spin_speed_ = yaml["aim_spin_speed"].as<double>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto distance = std::sqrt(tools::square(target_x) + tools::square(target_y));
  auto tolerance = distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

  // 按装甲实际尺寸折算角度容差：近距离小装甲的可用命中窗口远小于固定 2°，
  // 固定容差会在近处过松（打板）或远处过紧（漏打）。ratio 为命中窗口占板宽比例。
  //
  // 自转目标（|wz| 超过 aim_spin_speed）另用更严的 aim_tolerance_ratio_spin：
  // 换板时云台要横向跨越一整块装甲的间距，此时“云台已对上上一条指令”很容易成立，
  // 但实际处于跨板中途，打出去必然偏。复合/自转靶实测 0.8→0.4 命中率 70%→83%。
  double ratio = aim_tolerance_ratio_;
  const bool spinning = std::abs(targets.front().ekf_x()[7]) > aim_spin_speed_;
  if (spinning) ratio = aim_tolerance_ratio_spin_;
  if (ratio > 0 && distance > 0.1) {
    const double half_width = targets.front().armor_type == ArmorType::big
                                ? 230e-3 / 2.0
                                : 135e-3 / 2.0;
    auto geometry_tolerance = std::atan2(half_width * ratio, distance);
    tolerance = std::min(tolerance, geometry_tolerance);
  }

  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  //
  // 三个条件同时成立才开火：
  //   1. 相邻两帧瞄准角没有突变（避免刚切装甲就开火）
  //   2. 云台实际角度已跟上上一条指令（刚转完才开火，避免边转边打）
  //   3. 俯仰也进入容差，否则近处会打高或打低。
  //      俯仰单独按装甲**高度**折算：板宽和板高差得多（135×125mm），
  //      用宽度算出的容差放松了仰角条件，自转换板后 pitch 还没跟上就打出去，
  //      弹丸会从装甲下沿下面飞过再落地（仿真里表现为大量 ground）。
  double pitch_tolerance = tolerance * 2;
  if (ratio > 0 && distance > 0.1) {
    const double half_height = targets.front().armor_type == ArmorType::big
                                 ? 127e-3 / 2.0
                                 : 125e-3 / 2.0;
    pitch_tolerance = std::min(pitch_tolerance, std::atan2(half_height * ratio, distance) * 2);
  }
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    std::abs(gimbal_pos[1] - command.pitch) < pitch_tolerance && aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim