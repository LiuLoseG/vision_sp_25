#include "sim_client.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <regex>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"

namespace io
{
namespace
{
constexpr const char * kProtocol = "rm_aim_sim/2";
const auto kFreshTimeout = std::chrono::milliseconds(200);
const auto kFrameTimeout = std::chrono::seconds(5);
}  // namespace

SimClient::SimClient(const std::string & channel) : channel_(channel)
{
  if (!std::regex_match(channel_, std::regex("[A-Za-z0-9_-]{1,80}")))
    throw std::runtime_error("非法的仿真通道名: " + channel_);

  const std::string path = "/dev/shm/rm_aim_sim_" + channel_;
  directory_ = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
  if (directory_ < 0)
    throw std::runtime_error("无法打开 " + path + "，请先启动仿真器（./run.sh -- --remote）");

  struct stat info {};
  if (
    ::fstat(directory_, &info) != 0 || info.st_uid != ::getuid() ||
    (info.st_mode & 0777) != 0700) {
    ::close(directory_);
    directory_ = -1;
    throw std::runtime_error("仿真 IPC 目录必须属于当前用户且权限为 0700: " + path);
  }

  lock_ = ::openat(directory_, "client.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0) {
    if (lock_ >= 0) ::close(lock_);
    ::close(directory_);
    directory_ = -1;
    lock_ = -1;
    throw std::runtime_error("通道 " + channel_ + " 已被另一个客户端占用");
  }

  token_ = std::to_string(::getpid()) + "-" +
           std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

  tools::logger()->info("[SimClient] 已连接通道 {}", channel_);
}

SimClient::~SimClient()
{
  // 尽力停控，避免客户端退出后仿真端继续按旧指令跟踪
  try {
    if (ready()) send(false, false, 0.0, 0.0);
  } catch (...) {
  }
  if (lock_ >= 0) ::close(lock_);
  if (directory_ >= 0) ::close(directory_);
}

bool SimClient::poll_state()
{
  auto value = read_json("state.json");
  if (value.value("protocol", "") != kProtocol || value.value("type", "") != "state") return false;

  // 拒绝崩溃/重启前遗留的过期状态
  const double wall =
    std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  const auto it_wall = value.find("wall_unix_sec");
  if (it_wall == value.end() || !it_wall->is_number()) return false;
  const double age = wall - it_wall->get<double>();
  if (!std::isfinite(age) || age < -0.1 || age > 0.25) return false;

  if (ready()) {
    const bool same_publisher = value.value("publisher_id", "") == state_.value("publisher_id", "");
    const bool same_publication = value.value("publication", -1LL) == state_.value("publication", -2LL);
    if (same_publisher && same_publication) return false;
  }

  // 发布者或会话/标定变化时，丢弃未完成的图像请求
  if (!ready() || context(value) != context(state_)) {
    request_.clear();
  }

  state_ = std::move(value);
  last_state_ = std::chrono::steady_clock::now();
  return true;
}

bool SimClient::fresh() const
{
  return ready() && std::chrono::steady_clock::now() - last_state_ <= kFreshTimeout;
}

double SimClient::sim_time() const
{
  if (!ready()) return 0.0;
  const auto it = state_.find("sim_time");
  return (it != state_.end() && it->is_number()) ? it->get<double>() : 0.0;
}

bool SimClient::paused() const { return !ready() || state_.value("paused", true); }

bool SimClient::external_control() const
{
  if (!ready()) return false;
  const auto ipc = state_.find("ipc");
  if (ipc == state_.end()) return false;
  return ipc->value("enabled", false);
}

double SimClient::bullet_speed() const
{
  if (!ready()) return 0.0;
  const auto weapon = state_.find("weapon");
  if (weapon == state_.end()) return 0.0;
  const auto it = weapon->find("speed");
  return (it != weapon->end() && it->is_number()) ? it->get<double>() : 0.0;
}

nlohmann::json SimClient::stats() const
{
  if (!ready()) return nlohmann::json::object();
  const auto it = state_.find("stats");
  return (it != state_.end()) ? *it : nlohmann::json::object();
}

void SimClient::request_frame()
{
  if (!request_.empty() || !fresh()) return;
  request_ = token_ + "-" + std::to_string(request_count_++);
  requested_ = std::chrono::steady_clock::now();
  write_json(
    "request.json", {{"publisher_id", state_.at("publisher_id")}, {"request_id", request_}});
}

bool SimClient::try_frame(cv::Mat & bgr, nlohmann::json & metadata)
{
  if (request_.empty()) return false;
  if (std::chrono::steady_clock::now() - requested_ > kFrameTimeout)
    throw std::runtime_error("等待仿真图像超时：请用图形模式（不要 --headless）运行仿真器");

  auto frame = read_json("frame.json");
  if (frame.value("request_id", "") != request_) return false;
  if (
    !fresh() || frame.value("protocol", "") != kProtocol || frame.value("type", "") != "frame" ||
    context(frame) != context(state_)) {
    request_.clear();
    return false;
  }

  const double sim_age = sim_time() - frame.value("sim_time", 0.0);
  if (!std::isfinite(sim_age) || sim_age > 0.2 || sim_age < -0.1) {
    request_.clear();
    return false;
  }

  const auto & camera = frame.at("camera");
  const int width = camera.at("width").get<int>();
  const int height = camera.at("height").get<int>();
  const std::string encoding = frame.value("encoding", "");
  const int channels = encoding == "rgb8" ? 3 : encoding == "rgba8" ? 4 : 0;
  const std::string filename = "frame-" + request_ + ".raw";
  if (
    channels == 0 || width < 160 || width > 3840 || height < 120 || height > 2160 ||
    frame.value("frame_file", "") != filename)
    throw std::runtime_error("仿真图像元数据不合法（尺寸/编码/文件名）");

  const size_t bytes = static_cast<size_t>(width) * height * channels;
  if (frame.value("image_bytes", 0ULL) != bytes)
    throw std::runtime_error("仿真图像 image_bytes 与尺寸不匹配");

  int fd = ::openat(directory_, filename.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) throw std::runtime_error("仿真图像文件消失：仿真器可能已重启，请重新连接");

  cv::Mat raw;
  try {
    struct stat info {};
    if (
      ::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) ||
      info.st_size != static_cast<off_t>(bytes))
      throw std::runtime_error("仿真图像文件长度/类型不匹配");
    raw.create(height, width, CV_MAKETYPE(CV_8U, channels));
    read_all(fd, raw.data, bytes);
  } catch (...) {
    ::close(fd);
    throw;
  }
  ::close(fd);

  cv::cvtColor(raw, bgr, channels == 3 ? cv::COLOR_RGB2BGR : cv::COLOR_RGBA2BGR);
  metadata = std::move(frame);
  request_.clear();  // 已有独立 BGR 存储后才确认上一帧，允许下一帧请求
  return true;
}

bool SimClient::send(
  bool control, bool fire, double yaw, double pitch, double yaw_vel, double pitch_vel,
  double yaw_acc, double pitch_acc)
{
  if (!ready()) return false;
  for (double number : {yaw, pitch, yaw_vel, pitch_vel, yaw_acc, pitch_acc})
    if (!std::isfinite(number) || std::abs(number) > 1000) return false;

  const std::string session = state_.value("session_id", "");
  if (sequence_session_ != session) {
    sequence_session_ = session;
    sequence_ = 0;
  }
  const int64_t last_seq = state_.at("ipc").value("last_seq", -1LL);
  sequence_ = std::max(sequence_, last_seq + 1);

  const bool allow = fresh() && !paused() && external_control();
  const bool final_control = control && allow;
  const bool final_fire = fire && final_control;

  write_json(
    "command.json",
    {{"protocol", kProtocol},      {"type", "command"},
     {"publisher_id", state_.at("publisher_id")}, {"session_id", session},
     {"seq", sequence_++},         {"control", final_control},
     {"fire", final_fire},         {"yaw", yaw},
     {"pitch", pitch},             {"yaw_vel", yaw_vel},
     {"pitch_vel", pitch_vel},     {"yaw_acc", yaw_acc},
     {"pitch_acc", pitch_acc}});
  return final_control;
}

nlohmann::json SimClient::context(const nlohmann::json & packet)
{
  const auto & camera = packet.at("camera");
  return nlohmann::json::array(
    {packet.value("publisher_id", ""), packet.value("session_id", ""),
     packet.value("appearance", nlohmann::json::object()), packet.value("paused", false),
     camera.value("width", 0), camera.value("height", 0), camera.at("K"),
     camera.at("distortion"), camera.at("R_camera2gimbal"), camera.at("t_camera2gimbal")});
}

void SimClient::read_all(int fd, void * output, size_t bytes)
{
  auto data = static_cast<char *>(output);
  while (bytes) {
    const auto count = ::read(fd, data, bytes);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) throw std::runtime_error("读取仿真文件不完整");
    data += count;
    bytes -= static_cast<size_t>(count);
  }
}

nlohmann::json SimClient::read_json(const char * filename) const
{
  int fd = ::openat(directory_, filename, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) return nlohmann::json::object();

  struct stat info {};
  std::string data;
  try {
    if (
      ::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size <= 0 ||
      info.st_size > 65536)
      throw std::runtime_error("仿真 JSON 文件长度/类型不合法");
    data.resize(static_cast<size_t>(info.st_size));
    read_all(fd, data.data(), data.size());
  } catch (...) {
    ::close(fd);
    throw;
  }
  ::close(fd);

  auto value = nlohmann::json::parse(data, nullptr, false);
  return value.is_object() ? value : nlohmann::json::object();
}

void SimClient::write_json(const char * filename, const nlohmann::json & value) const
{
  const std::string temporary = std::string(filename) + ".client.tmp";
  int fd = ::openat(
    directory_, temporary.c_str(), O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (fd < 0) throw std::runtime_error(std::string("无法创建临时文件 ") + temporary);

  try {
    const auto data = value.dump();
    size_t written = 0;
    while (written < data.size()) {
      const auto count = ::write(fd, data.data() + written, data.size() - written);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("写入仿真文件不完整");
      written += static_cast<size_t>(count);
    }
  } catch (...) {
    ::close(fd);
    ::unlinkat(directory_, temporary.c_str(), 0);
    throw;
  }
  ::close(fd);

  if (::renameat(directory_, temporary.c_str(), directory_, filename) != 0)
    throw std::runtime_error(std::string("无法原子发布 ") + filename);
}

}  // namespace io
