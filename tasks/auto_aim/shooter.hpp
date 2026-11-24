#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
  // 按装甲尺寸/距离折算的角度容差比例；0 表示退回固定 first/second_tolerance_
  double aim_tolerance_ratio_ = 0.0;
  // 目标自转时的容差比例（默认与 aim_tolerance_ratio_ 相同）及其切换阈值 rad/s
  double aim_tolerance_ratio_spin_ = 0.0;
  double aim_spin_speed_ = 3.0;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP