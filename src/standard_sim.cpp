// 使用 rm_aim_sim 自瞄仿真器（Godot）做闭环测试的自瞄入口。
//
// 与 standard.cpp 的区别只在于“相机输入”和“云台通信”：
//   io::SimClient 通过 /dev/shm/rm_aim_sim_<channel>/ 收发原始图像、姿态和云台指令。
// 检测、PnP、跟踪、瞄准、发射决策仍然全部由本仓库的 auto_aim 模块完成。

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/command.hpp"
#include "io/sim/sim_client.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/sim.yaml | 位置参数，yaml配置文件路径 }"
  "{channel        |      | 覆盖 yaml 中的 sim.channel }"
  "{show           |      | 打开检测/瞄准结果窗口（需要图形环境）}"
  "{no-fire        |      | 只跟踪不发弹 }"
  "{duration       | 0    | 运行时长（秒），0 表示一直运行 }";

namespace
{
// 把仿真的 sim_time 映射到 steady_clock 时间点：Tracker/Aimer 内部以 time_point
// 表示绝对时间，而仿真时间不是墙钟时间，需要一个单调、平滑、可复现的换算。
//
// 关键：映射必须严格线性 t = anchor_wall + (sim - anchor_sim)。
// 不做任何向墙钟靠拢的修正 —— 否则仿真落后于墙钟时会反复重锚，
// 造成 Tracker 看到 0.5s 的 dt 跳变并误判目标丢失。
class SimTimeMapper
{
public:
  void update(double latest_sim_time)
  {
    if (init_) return;
    init_ = true;
    anchor_wall_ = Clock::now();
    anchor_sim_ = latest_sim_time;
  }

  Clock::time_point to_point(double sim_time) const
  {
    const double offset = sim_time - anchor_sim_;
    return anchor_wall_ + std::chrono::microseconds(static_cast<int64_t>(offset * 1e6));
  }

private:
  bool init_ = false;
  Clock::time_point anchor_wall_;
  double anchor_sim_ = 0.0;
};

std::vector<double> numbers(const nlohmann::json & node, const char * key, std::size_t expected)
{
  const auto it = node.find(key);
  if (it == node.end() || !it->is_array() || it->size() != expected)
    throw std::runtime_error(std::string("仿真元数据缺少字段: ") + key);
  return it->get<std::vector<double>>();
}

// 云台世界原点（仿真世界系在地面，视觉内部以云台为原点）
Eigen::Vector3d gimbal_origin(const nlohmann::json & frame)
{
  const auto gimbal = frame.find("gimbal");
  if (gimbal == frame.end()) return Eigen::Vector3d::Zero();
  const auto o = gimbal->find("origin");
  if (o == gimbal->end() || !o->is_array() || o->size() != 3) return Eigen::Vector3d::Zero();
  return {(*o)[0].get<double>(), (*o)[1].get<double>(), (*o)[2].get<double>()};
}

// 出膛角诊断用的一帧真值（仿真 state.truth 的装甲中心）
struct TruthSample
{
  double sim_time = 0.0;
  std::vector<Eigen::Vector3d> armors;
};

// 用出膛时刻的真值反查“这发弹本来该指向哪块装甲”，
// 只用于离线分析命中/落地的横向误差，不参与控制。
void analyze_impacts(const io::SimClient & sim, const std::vector<TruthSample> & history)
{
  tools::logger()->info("[出膛诊断] 真值采样 {} 条", history.size());
  if (history.empty() || !sim.ready()) return;
  const auto & events = sim.state().find("events");
  if (events == sim.state().end()) {
    tools::logger()->info("[出膛诊断] 状态里没有 events");
    return;
  }
  tools::logger()->info("[出膛诊断] 事件 {} 条", events->size());
  const auto origin = gimbal_origin(sim.state());

  std::map<int, double> shot_time;
  for (const auto & event : *events) {
    if (event.value("type", "") == "shot")
      shot_time[event.value("id", -1)] = event.value("time", 0.0);
  }
  tools::logger()->info("[出膛诊断] shot 记录 {} 条", shot_time.size());

  struct Stat
  {
    int count = 0;
    double sum = 0.0, max = 0.0;
  } hit_stat, ground_stat, expired_stat;

  int n_no_shot = 0, n_no_truth = 0, n_no_armor = 0;
  for (const auto & event : *events) {
    if (event.value("type", "") != "impact") continue;
    const std::string kind = event.value("kind", "");
    const int id = event.value("id", -1);
    const auto it = shot_time.find(id);
    if (it == shot_time.end()) {
      ++n_no_shot;
      continue;
    }
    // 找目标平面穿越时刻的真值采样。
    // 弹丸沿出膛方向直线飞行，横向不受重力影响，到达目标平面约用
    // 距离/弹速 ≈ 0.17s；必须用**该时刻**的装甲位置比较，否则合法的提前量
    // 会被当成误差（自转 4rad/s 时 0.17s 就转 40°）。
    const double cross_time = it->second + 0.17;
    const TruthSample * nearest = nullptr;
    double best_dt = 1e9;
    for (const auto & sample : history) {
      const double dt = std::abs(sample.sim_time - cross_time);
      if (dt < best_dt) {
        best_dt = dt;
        nearest = &sample;
      }
    }
    if (nearest == nullptr) {
      ++n_no_truth;
      continue;
    }

    const auto & point = event.at("point");
    const double impact_yaw = std::atan2(point[1].get<double>(), point[0].get<double>());
    // 弹丸按抛物线飞行，横向不受重力影响：落点方位角 - 出膛方位角 应为 0
    const auto shot = std::find_if(
      events->begin(), events->end(), [&](const nlohmann::json & e) {
        return e.value("type", "") == "shot" && e.value("id", -1) == id;
      });
    const double shot_yaw = (shot != events->end()) ? shot->value("yaw", 0.0) : impact_yaw;

    double err = 1e9;
    for (const auto & center : nearest->armors) {
      const double yaw = std::atan2(center.y() - origin.y(), center.x() - origin.x());
      err = std::min(err, std::abs(tools::limit_rad(shot_yaw - yaw)) * 57.3);
    }
    if (err > 1e8) {
      ++n_no_armor;
      continue;
    }

    Stat * target = (kind == "hit") ? &hit_stat : (kind == "ground") ? &ground_stat
                                                                     : &expired_stat;
    ++target->count;
    target->sum += err;
    target->max = std::max(target->max, err);
  }

  tools::logger()->info(
    "[出膛诊断] 跳过: 无shot={} 无真值={} 无装甲={}", n_no_shot, n_no_truth, n_no_armor);
  for (const auto & pair : {std::make_pair("hit", hit_stat), std::make_pair("ground", ground_stat),
                            std::make_pair("expired", expired_stat)}) {
    if (pair.second.count == 0) continue;
    tools::logger()->info(
      "[出膛诊断] {:<8} n={:3d} 横向瞄准误差 mean={:.2f}° max={:.2f}°", pair.first,
      pair.second.count, pair.second.sum / pair.second.count, pair.second.max);
  }
}

void draw_result(
  cv::Mat & img, const std::list<auto_aim::Armor> & armors,
  const std::list<auto_aim::Target> & targets, const io::Command & command, const std::string & state)
{
  for (const auto & armor : armors) {
    std::vector<cv::Point> pts;
    for (const auto & p : armor.points) pts.emplace_back(cv::Point2i(p));
    if (pts.size() == 4) cv::polylines(img, pts, true, {0, 255, 0}, 1);
    cv::putText(
      img, fmt::format("{}/{}", auto_aim::ARMOR_NAMES[armor.name], auto_aim::COLORS[armor.color]),
      armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
  }

  cv::putText(
    img, fmt::format("state={} armors={} targets={} cmd=({:.2f},{:.2f}) fire={}", state,
                     armors.size(), targets.size(), command.yaw * 57.3, command.pitch * 57.3,
                     command.shoot ? 1 : 0),
    {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 仿真器运行期间与 OpenCV 窗口共存：强制 Qt5 + xcb 后端。
  // 缺任一项都会在部分 GNOME 环境下触发 GTK 主题插件崩溃：
  //   Settings schema 'org.gnome.settings-daemon.plugins.xsettings'
  //   does not contain a key named 'antialiasing'
  setenv("OPENCV_UI_BACKEND", "QT5", 1);
  setenv("QT_QPA_PLATFORMTHEME", "", 1);
  setenv("QT_QPA_PLATFORM", "xcb", 1);

  auto yaml = YAML::LoadFile(config_path);
  std::string channel = cli.get<std::string>("channel");
  if (channel.empty()) channel = yaml["sim"]["channel"].as<std::string>("default");
  std::string detector_backend = "yolo";
  if (yaml["sim"] && yaml["sim"]["detector"])
    detector_backend = yaml["sim"]["detector"].as<std::string>();
  // 统计出的“图像采样 → 下发指令”延迟的补偿值，单位 s。
  // 仿真时间域内不需要叠加墙钟误差，默认 0 即可。
  double aim_delay = 0.0;
  if (yaml["sim"] && yaml["sim"]["aim_delay"])
    aim_delay = yaml["sim"]["aim_delay"].as<double>();
  // 是否下发目标角速度前馈（消除二阶伺服稳态滞后）
  bool feedforward = true;
  if (yaml["sim"] && yaml["sim"]["feedforward"])
    feedforward = yaml["sim"]["feedforward"].as<bool>();
  double ff_horizon = 0.06;
  if (yaml["sim"] && yaml["sim"]["ff_horizon"])
    ff_horizon = yaml["sim"]["ff_horizon"].as<double>();
  bool ff_acc = false;
  if (yaml["sim"] && yaml["sim"]["ff_acc"]) ff_acc = yaml["sim"]["ff_acc"].as<bool>();
  // 控制频率：false（默认）在每个检测帧下发一次，true 则在帧间也持续下发。
  // 实测仿真的二阶云台在“每帧一次 + 前馈”下跟随最稳；帧间高频重发反而引入抖动，
  // 因此默认关闭，仅作为实验开关保留。
  bool high_rate_control = false;
  if (yaml["sim"] && yaml["sim"]["high_rate_control"])
    high_rate_control = yaml["sim"]["high_rate_control"].as<bool>();
  // 目标角加速度超过该值时不开火（rad/s²）。模拟变速端点这类无论怎么预测都会打偏的
  // 时段；0 表示关闭这个门控。
  double fire_acc_limit = 0.0;
  if (yaml["sim"] && yaml["sim"]["fire_acc_limit"])
    fire_acc_limit = yaml["sim"]["fire_acc_limit"].as<double>();

  const bool show = cli.has("show");
  const bool no_fire = cli.has("no-fire");
  const double duration = cli.get<double>("duration");

  tools::Exiter exiter;

  io::SimClient sim(channel);
  // 两种检测后端共用同一条 PnP/跟踪/瞄准链：
  //   yolo        实体场地训练的网络 + 传统角点矫正
  //   traditional 纯灯条几何，适合仿真渲染画面与实体图像分布不同的场景
  std::unique_ptr<auto_aim::YOLO> yolo;
  std::unique_ptr<auto_aim::Detector> traditional;
  if (detector_backend == "traditional") {
    traditional = std::make_unique<auto_aim::Detector>(config_path, false);
  } else {
    yolo = std::make_unique<auto_aim::YOLO>(config_path, false);
  }
  tools::logger()->info("[Sim] 检测后端: {}", detector_backend);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  SimTimeMapper mapper;
  std::string calibration_key;
  bool calibrated = false;

  const auto begin = Clock::now();
  auto last_state = Clock::now();
  auto last_report = Clock::now();
  auto last_command = Clock::now();
  uint64_t frames = 0, control_ticks = 0;

  // 最近一次跟踪快照：检测低频，控制高频，两者用这个快照衔接。
  std::list<auto_aim::Target> latest_targets;
  double latest_observation_sim_time = -1.0;
  // 云台前馈量（角速度/角加速度），每次控制 tick 重算
  struct Feedforward
  {
    double yaw_vel = 0.0, pitch_vel = 0.0, yaw_acc = 0.0, pitch_acc = 0.0;
  };
  // 观测超时：超过这个仿真时间没有新观测就停控（仿真时间域，暂停时自然冻结）
  constexpr double kObservationTimeout = 0.25;
  // 目标列表为空/超时后的停控间隔
  constexpr auto kIdleStopPeriod = 50ms;

  io::Command command{false, false, 0.0, 0.0};
  Eigen::Vector3d gimbal_pos = Eigen::Vector3d::Zero();
  // 出膛诊断用：SP_SIM_ANALYZE=1 时记录真值历史（只在图形会话里有效）
  const bool analyze = std::getenv("SP_SIM_ANALYZE") != nullptr;
  std::vector<TruthSample> truth_history;

  while (!exiter.exit()) {
    if (duration > 0 &&
        std::chrono::duration<double>(Clock::now() - begin).count() > duration)
      break;

    if (sim.poll_state()) {
      last_state = Clock::now();
      mapper.update(sim.sim_time());
    }
    if (Clock::now() - last_state > 5s) {
      tools::logger()->error("[Sim] 状态停止更新，退出");
      break;
    }
    if (!sim.fresh()) {
      std::this_thread::sleep_for(2ms);
      continue;
    }

    // ---------- 1) 有新图像时做一次检测 + 跟踪（低频） ----------
    sim.request_frame();
    cv::Mat img;
    nlohmann::json meta;
    bool got_new_frame = false;
    if (sim.try_frame(img, meta)) {
      ++frames;

      // 调试：SP_SIM_DUMP=<dir> 每隔 N 帧落盘一张原始帧，便于离线分析检测器输入
      if (const char * dump_dir = std::getenv("SP_SIM_DUMP")) {
        static uint64_t dump_n = 0;
        const int every =
          std::getenv("SP_SIM_DUMP_EVERY") ? std::atoi(std::getenv("SP_SIM_DUMP_EVERY")) : 30;
        if (every > 0 && dump_n % every == 0) {
          cv::imwrite(fmt::format("{}/frame_{:05d}.png", dump_dir, dump_n), img);
        }
        ++dump_n;
      }

      // 同帧标定：使用图像采样时刻的内外参，不使用最新状态替换
      const auto & camera = meta.at("camera");
      const auto k = numbers(camera, "K", 9);
      const auto r_camera2gimbal = numbers(camera, "R_camera2gimbal", 9);
      const auto t_camera2gimbal = numbers(camera, "t_camera2gimbal", 3);
      const std::string key = fmt::format("{:.6f},{:.6f},{:.6f},{:.6f}", k[0], k[4], k[2], k[5]);
      if (!calibrated || key != calibration_key) {
        solver.set_camera_matrix(k[0], k[4], k[2], k[5]);
        Eigen::Matrix3d R = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(r_camera2gimbal.data());
        Eigen::Vector3d t(t_camera2gimbal[0], t_camera2gimbal[1], t_camera2gimbal[2]);
        solver.set_camera_extrinsic(R, t);
        calibration_key = key;
        calibrated = true;
        tools::logger()->info(
          "[Sim] 标定: fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f} {}x{}", k[0], k[4], k[2], k[5],
          camera.at("width").get<int>(), camera.at("height").get<int>());
      }

      // 同帧姿态
      const auto q = numbers(meta.at("gimbal"), "quaternion_wxyz", 4);
      Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
      solver.set_R_gimbal2world(quat);

      const double frame_sim_time = meta.value("sim_time", sim.sim_time());
      const auto t = mapper.to_point(frame_sim_time);

      auto armors = traditional ? traditional->detect(img) : yolo->detect(img);
      auto targets = tracker.track(armors, t);

      latest_targets = targets;
      latest_observation_sim_time = frame_sim_time;

      if (analyze && sim.state().contains("truth")) {
        TruthSample sample;
        sample.sim_time = frame_sim_time;
        for (const auto & a : sim.state().at("truth").at("armors")) {
          sample.armors.emplace_back(
            a["center"][0].get<double>(), a["center"][1].get<double>(),
            a["center"][2].get<double>());
        }
        truth_history.emplace_back(std::move(sample));
        if (truth_history.size() > 4000) truth_history.erase(truth_history.begin());
      }

      if (show) {
        draw_result(img, armors, targets, command, tracker.state());
        cv::imshow("standard_sim", img);
        const auto key_pressed = cv::waitKey(1);
        if (key_pressed == 'q' || key_pressed == 27) break;
      }
      got_new_frame = true;
    }

    // ---------- 2) 控制 tick ----------
    // 默认每个检测帧下发一次（实测跟随最稳）；high_rate_control=true 时在帧间也
    // 持续用同一份跟踪快照预测到当前仿真时刻重发，可提高控制频率，但仿真云台会抖动。
    gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    const double now_sim_time = sim.sim_time();
    const double observation_age = now_sim_time - latest_observation_sim_time;
    const bool have_target = !latest_targets.empty() && latest_observation_sim_time >= 0 &&
                             observation_age <= kObservationTimeout;

    if (!have_target) {
      if (!latest_targets.empty()) {
        tools::logger()->debug("[Sim] 观测超时 {:.2f}s，停止控制", observation_age);
        latest_targets.clear();
      }
      // 空闲时低频发停控，避免打扰仿真（超时机制也会兜底）
      if (sim.external_control() && Clock::now() - last_command > kIdleStopPeriod) {
        sim.send(false, false, 0.0, 0.0);
        last_command = Clock::now();
      }
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // 每帧一次控制的模式：没有新帧就不要重复下发（重复下发会让 shooter 的
    // 上一帧指令与实际姿态的“稳定”判定失效，实测命中率明显下降）。
    if (!high_rate_control && !got_new_frame) {
      std::this_thread::sleep_for(1ms);
      continue;
    }

    auto control_targets = latest_targets;
    const auto now_t =
      mapper.to_point(now_sim_time) + std::chrono::microseconds(static_cast<int64_t>(aim_delay * 1e6));
    // 只有每帧一次控制时，把目标预测到“本帧采样时刻”；帧间高频控制时预测到当前时刻。
    const auto predict_t = high_rate_control ? now_t : mapper.to_point(latest_observation_sim_time) +
                                                  std::chrono::microseconds(
                                                    static_cast<int64_t>(aim_delay * 1e6));
    for (auto & target : control_targets) target.predict(predict_t);

    command = aimer.aim(control_targets, predict_t, sim.bullet_speed(), /*to_now=*/false);
    command.shoot = !no_fire && shooter.shoot(command, aimer, control_targets, gimbal_pos);

    // 目标角速度/角加速度前馈：仿真的云台是二阶伺服（response_hz=6），只给目标角
    // 会让它相对匀速平移/旋转的目标一直滞后 1/w·ω，目标变速时还有 τ·α 的额外滞后。
    // 下发 goal_v / goal_a 可把这两部分滞后压掉；实车的云台下发链路同样支持。
    Feedforward ff;
    double acc_mag = 0.0;  // 只用于开火门控，不参与前馈
    if (feedforward && command.control) {
      const double h = ff_horizon;
      // 用 Aimer 副本预测未来瞄准角：副本有独立的 lock_id_，不污染主 aimer 的锁定
      auto aim_at = [&](double dt) -> std::optional<io::Command> {
        auto future_targets = control_targets;
        const auto future = predict_t + std::chrono::microseconds(static_cast<int64_t>(dt * 1e6));
        for (auto & target : future_targets) target.predict(future);
        auto future_aimer = aimer;
        auto future_command = future_aimer.aim(future_targets, future, sim.bullet_speed(), false);
        if (!future_command.control) return std::nullopt;
        return future_command;
      };
      const auto next = aim_at(h);
      const auto next2 = (ff_acc || fire_acc_limit > 0) ? aim_at(2 * h) : std::nullopt;
      if (next) {
        ff.yaw_vel = tools::limit_rad(next->yaw - command.yaw) / h;
        ff.pitch_vel = (next->pitch - command.pitch) / h;
        if (next2) {
          const double acc_yaw =
            (tools::limit_rad(next2->yaw - next->yaw) - tools::limit_rad(next->yaw - command.yaw)) /
            (h * h);
          const double acc_pitch =
            ((next2->pitch - next->pitch) - (next->pitch - command.pitch)) / (h * h);
          acc_mag = std::max(std::abs(acc_yaw), std::abs(acc_pitch));
          // 角加速度前馈默认关闭：二阶差分噪声较大，实测会降低命中率
          if (ff_acc) {
            ff.yaw_acc = acc_yaw;
            ff.pitch_acc = acc_pitch;
          }
        }
        // 装甲切换时前馈会瞬间跳变，按仿真云台的限幅约束裁剪
        ff.yaw_vel = tools::limit_min_max(ff.yaw_vel, -6.0, 6.0);
        ff.pitch_vel = tools::limit_min_max(ff.pitch_vel, -6.0, 6.0);
        ff.yaw_acc = tools::limit_min_max(ff.yaw_acc, -50.0, 50.0);
        ff.pitch_acc = tools::limit_min_max(ff.pitch_acc, -50.0, 50.0);
      }
    }
    if (fire_acc_limit > 0 && acc_mag > fire_acc_limit) command.shoot = false;

    sim.send(
      command.control, command.shoot, command.yaw, command.pitch, ff.yaw_vel, ff.pitch_vel,
      ff.yaw_acc, ff.pitch_acc);
    last_command = Clock::now();
    ++control_ticks;

    if (Clock::now() - last_report > 1s) {
      last_report = Clock::now();
      const auto stats = sim.stats();
      tools::logger()->info(
        "[Sim] frames={} ctrl={} state={} targets={} cmd=({:.2f},{:.2f})° fire={} | "
        "fired={} hit={} resolved={} rate={:.1f}%",
        frames, control_ticks, tracker.state(), control_targets.size(), command.yaw * 57.3,
        command.pitch * 57.3, command.shoot, stats.value("fired", 0), stats.value("hit", 0),
        stats.value("resolved", 0), stats.value("hit_rate", 0.0) * 100);

      // SP_SIM_EKF=1：打印 EKF 中心与速度，便于核对提前量是否收敛
      if (std::getenv("SP_SIM_EKF")) {
        const auto x = control_targets.front().ekf_x();
        // 同帧真值（state 最新值）：判断瞄准角相对目标真实方位是超前还是滞后。
        double truth_yaw = std::nan("");
        double vx = 0, vy = 0;
        if (sim.state().contains("truth")) {
          const auto & truth = sim.state().at("truth");
          const double cx = truth.at("center")[0].get<double>();
          const double cy = truth.at("center")[1].get<double>();
          const double cz = truth.at("center")[2].get<double>();
          const auto origin = gimbal_origin(sim.state());
          truth_yaw = std::atan2(cy - origin.y(), cx - origin.x());
          static double last_t = -1, last_x = 0, last_y = 0;
          if (last_t > 0 && now_sim_time > last_t) {
            const double dt = now_sim_time - last_t;
            vx = (cx - last_x) / dt;
            vy = (cy - last_y) / dt;
          }
          last_t = now_sim_time;
          last_x = cx;
          last_y = cy;
        }
        tools::logger()->info(
          "[ekf] center=({:.3f},{:.3f}) v=({:.3f},{:.3f}) wz={:.2f} r={:.3f} | cmd={:.2f}° "
          "gimbal={:.2f}° truth={:.2f}° v_true=({:.2f},{:.2f}) lead={:.2f}°",
          x[0], x[2], x[1], x[3], x[7], x[8], command.yaw * 57.3, gimbal_pos[0] * 57.3,
          truth_yaw * 57.3, vx, vy, (command.yaw - truth_yaw) * 57.3);
      }
    }

    // 高频控制模式：没有新帧时让出 CPU，控制照常以 1ms 量级周期运行
    if (!got_new_frame && !show) std::this_thread::sleep_for(1ms);
  }

  sim.send(false, false, 0.0, 0.0);
  if (show) cv::destroyAllWindows();

  const auto stats = sim.stats();
  tools::logger()->info(
    "[Sim] 结束: frames={} fired={} hit={} resolved={} rate={:.1f}%", frames,
    stats.value("fired", 0), stats.value("hit", 0), stats.value("resolved", 0),
    stats.value("hit_rate", 0.0) * 100);
  if (analyze) analyze_impacts(sim, truth_history);
  return 0;
}
