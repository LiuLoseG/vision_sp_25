#ifndef IO__SIM_SIM_CLIENT_HPP
#define IO__SIM_SIM_CLIENT_HPP

#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>

#include <nlohmann/json.hpp>

namespace io
{
// rm_aim_sim（Godot 自瞄仿真器）本机共享内存客户端，协议 rm_aim_sim/2。
//
// 数据位于 /dev/shm/rm_aim_sim_<channel>/：
//   state.json  仿真 -> 客户端（最新值覆盖）
//   command.json 客户端 -> 仿真（最新值覆盖）
//   request.json / frame.json / frame-<id>.raw  相机图像握手
//
// 仅支持 Linux 同机同用户；非线程安全，必须由单个线程独占使用。
// 出错（协议不符、目录消失、帧文件丢失）抛 std::runtime_error。
class SimClient
{
public:
  explicit SimClient(const std::string & channel);
  ~SimClient();

  SimClient(const SimClient &) = delete;
  SimClient & operator=(const SimClient &) = delete;

  const std::string & channel() const { return channel_; }

  // 读取状态；有新发布返回 true。重复/过期/损坏的包被忽略。
  bool poll_state();

  // 最后一次新状态的接收年龄是否在阈值内
  bool fresh() const;

  // 是否拿到过有效状态
  bool ready() const { return !state_.is_null(); }

  const nlohmann::json & state() const { return state_; }

  double sim_time() const;
  bool paused() const;
  bool external_control() const;  // 仿真侧“本机自瞄控制”是否开启
  double bullet_speed() const;

  // 请求一帧图像；已有未完成请求或状态不新鲜时忽略。
  void request_frame();

  // 取回匹配当前请求的一帧图像。返回 true 时 bgr 是独立的 BGR 存储，
  // metadata 为同帧 frame.json（含 sim_time/gimbal/camera 等）。
  bool try_frame(cv::Mat & bgr, nlohmann::json & metadata);

  // 是否已有未完成的图像请求（用于判断是否在等图）
  bool waiting_for_frame() const { return !request_.empty(); }

  // 发送绝对目标角指令。control/fire 会被状态与新鲜度门控。
  // 返回 false 表示本次未发送（状态不可用或数值非法）。
  bool send(
    bool control, bool fire, double yaw, double pitch, double yaw_vel = 0.0,
    double pitch_vel = 0.0, double yaw_acc = 0.0, double pitch_acc = 0.0);

  // 仿真侧统计（只读，直接来自 state.json）
  nlohmann::json stats() const;

private:
  std::string channel_;
  int directory_ = -1;
  int lock_ = -1;

  nlohmann::json state_;
  std::string request_;
  std::string token_;
  std::string sequence_session_;
  int64_t sequence_ = 0;
  uint64_t request_count_ = 0;

  std::chrono::steady_clock::time_point last_state_;
  std::chrono::steady_clock::time_point requested_;

  nlohmann::json read_json(const char * filename) const;
  void write_json(const char * filename, const nlohmann::json & value) const;
  static void read_all(int fd, void * output, size_t bytes);
  static nlohmann::json context(const nlohmann::json & packet);
};

}  // namespace io

#endif  // IO__SIM_SIM_CLIENT_HPP
