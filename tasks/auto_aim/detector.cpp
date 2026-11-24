#include "detector.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <numeric>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
Detector::Detector(const std::string & config_path, bool debug)
: classifier_(config_path), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  threshold_ = yaml["threshold"].as<double>();
  max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;  // degree to rad
  min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
  max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
  min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
  min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
  max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
  max_side_ratio_ = yaml["max_side_ratio"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;  // degree to rad
  if (yaml["lightbar_use_max_channel"])
    use_max_channel_ = yaml["lightbar_use_max_channel"].as<bool>();
  if (yaml["lightbar_strict_color"])
    strict_color_ = yaml["lightbar_strict_color"].as<bool>();
  if (yaml["lightbar_color_mask"]) {
    color_mask_ = yaml["lightbar_color_mask"].as<bool>();
    mask_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
    if (yaml["lightbar_color_diff"])
      color_diff_ = yaml["lightbar_color_diff"].as<double>();
  }
  if (yaml["relax_type_check"]) relax_type_check_ = yaml["relax_type_check"].as<bool>();

  save_path_ = "patterns";
  std::filesystem::create_directory(save_path_);
}

std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
{
  // 彩色图转灰度图
  cv::Mat gray_img;
  if (use_max_channel_) {
    // 纯色灯条用 max(B,R) 更鲁棒：它保留了灯条的绝对亮度，不会因为色彩权重被压暗
    std::vector<cv::Mat> channels;
    cv::split(bgr_img, channels);
    cv::max(channels[0], channels[2], gray_img);
  } else {
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
  }

  // 进行二值化
  cv::Mat binary_img;
  if (color_mask_) {
    // 只保留目标颜色通道的亮像素。
    // 注意：不能用色差(B-R)掩码 —— 仿真/强光下灯条中心会过曝成白色，
    // 色差为 0 会把灯条从中间挖空、只剩细边，导致灯条长度被严重低估。
    std::vector<cv::Mat> ch;
    cv::split(bgr_img, ch);
    const bool want_blue = (mask_color_ == Color::blue);
    const cv::Mat & target = want_blue ? ch[0] : ch[2];
    const cv::Mat & other = want_blue ? ch[2] : ch[0];

    cv::Mat bright;
    cv::threshold(target, bright, threshold_, 255, cv::THRESH_BINARY);

    if (color_diff_ > 0) {
      cv::Mat diff, pure, white_src, white, keep;
      cv::subtract(target, other, diff);
      cv::threshold(diff, pure, color_diff_, 255, cv::THRESH_BINARY);
      // 灯条中心过曝区域（两色通道都很高）也算灯条本体
      cv::min(ch[0], ch[2], white_src);
      cv::threshold(white_src, white, 200, 255, cv::THRESH_BINARY);
      cv::bitwise_or(pure, white, keep);
      cv::bitwise_and(bright, keep, binary_img);
    } else {
      binary_img = bright;
    }
  } else {
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
  }
  // 只在 debug 模式下弹窗，否则无 GUI 环境（部署/仿真）会阻塞或抛异常
  if (debug_) cv::imshow("binary_img", binary_img);
  // 调试用：把二值图与原始图落盘，便于离线分析检测器的实际输入
  if (const char * dump_dir = std::getenv("SP_DETECTOR_DUMP")) {
    static int dump_n = 0;
    if (dump_n % 50 == 0) {
      cv::imwrite(fmt::format("{}/bin_{:04d}.png", dump_dir, dump_n), binary_img);
      cv::imwrite(fmt::format("{}/raw_{:04d}.png", dump_dir, dump_n), bgr_img);
    }
    ++dump_n;
  }

  // 获取轮廓点
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // 获取灯条
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  const bool lb_dbg = debug_ || std::getenv("SP_DETECTOR_DEBUG");
  int lb_dbg_n = 0;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) {
      if (lb_dbg && lb_dbg_n++ < 12) {
        tools::logger()->info(
          "[lb] reject len={:.1f} ratio={:.2f} ang_err={:.1f}° (阈值 len>{:.0f} ratio {:.1f}~{:.1f} "
          "ang<{:.0f}°)",
          lightbar.length, lightbar.ratio, lightbar.angle_error * 57.3, min_lightbar_length_,
          min_lightbar_ratio_, max_lightbar_ratio_, max_angle_error_ * 57.3);
      }
      continue;
    }

    lightbar.color = get_color(bgr_img, contour);
    // 严格色彩模式下，颜色不纯（非蓝非红）的轮廓直接丢弃
    if (lightbar.color == Color::extinguish) continue;
    if (lb_dbg && lb_dbg_n++ < 12) {
      tools::logger()->info(
        "[lb] accept len={:.1f} ratio={:.2f} color={} at ({:.0f},{:.0f})", lightbar.length,
        lightbar.ratio, COLORS[lightbar.color], lightbar.center.x, lightbar.center.y);
    }
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  // 将灯条从左到右排序
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  if (debug_ || std::getenv("SP_DETECTOR_DEBUG")) {
    static int dbg = 0;
    if (dbg++ % 30 == 0) {
      tools::logger()->info(
        "[detector] 轮廓={} 灯条={} 阈值={:.0f} max通道={}", contours.size(), lightbars.size(),
        threshold_, use_max_channel_);
    }
  }

  // 获取装甲板
  std::list<Armor> armors;
  for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
    for (auto right = std::next(left); right != lightbars.end(); right++) {
      if (left->color != right->color) continue;

      auto armor = Armor(*left, *right);
      if (!check_geometry(armor)) {
        if (lb_dbg && lb_dbg_n++ < 16) {
          tools::logger()->info(
            "[pair] 几何拒绝 ratio={:.2f} side_ratio={:.2f} rect_err={:.1f}° "
            "(阈值 {:.1f}~{:.1f} / <{:.1f} / <{:.0f}°)",
            armor.ratio, armor.side_ratio, armor.rectangular_error * 57.3, min_armor_ratio_,
            max_armor_ratio_, max_side_ratio_, max_rectangular_error_ * 57.3);
        }
        continue;
      }

      armor.pattern = get_pattern(bgr_img, armor);
      classifier_.classify(armor);
      if (lb_dbg && lb_dbg_n++ < 16) {
        tools::logger()->info(
          "[pair] 分类 name={} conf={:.3f} (阈值>{:.2f})", ARMOR_NAMES[armor.name],
          armor.confidence, min_confidence_);
      }
      // 仿真/新场地：分类器置信度不可靠，可按开关跳过图案校验
      if (!relax_type_check_ && !check_name(armor)) continue;
      if (relax_type_check_) {
        // 编号统一成普通步兵，保证相邻帧名字稳定（前哨站/基地保留，便于排除）
        if (armor.name != ArmorName::outpost && armor.name != ArmorName::base) {
          armor.name = ArmorName::three;
          armor.type = ArmorType::small;
        } else {
          armor.type = get_type(armor);
        }
      } else {
        armor.type = get_type(armor);
        if (!check_type(armor)) continue;
      }

      armor.center_norm = get_center_norm(bgr_img, armor.center);
      armors.emplace_back(armor);
    }
  }

  // 检查装甲板是否存在共用灯条的情况
  for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++) {
    for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++) {
      if (
        armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
        armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id) {
        continue;
      }

      // 装甲板重叠, 保留roi小的
      if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
        auto area1 = armor1->pattern.cols * armor1->pattern.rows;
        auto area2 = armor2->pattern.cols * armor2->pattern.rows;
        if (area1 < area2)
          armor2->duplicated = true;
        else
          armor1->duplicated = true;
      }

      // 装甲板相连，保留置信度大的
      if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
        if (armor1->confidence < armor2->confidence)
          armor1->duplicated = true;
        else
          armor2->duplicated = true;
      }
    }
  }

  armors.remove_if([&](const Armor & a) { return a.duplicated; });

  if (debug_) show_result(binary_img, bgr_img, lightbars, armors, frame_count);

  return armors;
}

bool Detector::detect(Armor & armor, const cv::Mat & bgr_img)
{
  // 取得四个角点
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];
  // 计算向量和调整后的点
  auto lt2b = bl - tl;
  auto rt2b = br - tr;
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;
  auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;
  // 构造新的四个角点
  std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
  auto armor_rotaterect = cv::minAreaRect(points);
  cv::Rect boundingBox = armor_rotaterect.boundingRect();
  // 检查boundingBox是否超出图像边界
  if (
    boundingBox.x < 0 || boundingBox.y < 0 || boundingBox.x + boundingBox.width > bgr_img.cols ||
    boundingBox.y + boundingBox.height > bgr_img.rows) {
    return false;
  }

  // 在图像上裁剪出这个矩形区域（ROI）
  cv::Mat armor_roi = bgr_img(boundingBox);
  if (armor_roi.empty()) {
    return false;
  }

  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);
  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
  // cv::imshow("binary_img", binary_img);
  // 获取轮廓点
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // 获取灯条
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue;

    lightbar.color = get_color(bgr_img, contour);
    // lightbar_points_corrector(lightbar, gray_img); //关闭PCA
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  if (lightbars.size() < 2) return false;

  // 将灯条从左到右排序
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  // 计算与 tl_roi, bl_roi 和 br_roi, tr_roi 距离最近的灯条
  Lightbar * closest_left_lightbar = nullptr;
  Lightbar * closest_right_lightbar = nullptr;
  float min_distance_tl_bl = std::numeric_limits<float>::max();
  float min_distance_br_tr = std::numeric_limits<float>::max();
  for (auto & lightbar : lightbars) {
    float distance_tl_bl =
      cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_tl_bl < min_distance_tl_bl) {
      min_distance_tl_bl = distance_tl_bl;
      closest_left_lightbar = &lightbar;
    }
    float distance_br_tr =
      cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_br_tr < min_distance_br_tr) {
      min_distance_br_tr = distance_br_tr;
      closest_right_lightbar = &lightbar;
    }
  }

  // tools::logger()->debug(
  // "min_distance_br_tr + min_distance_tl_bl is {}", min_distance_br_tr + min_distance_tl_bl);
  // std::vector<cv::Point2f> points2f{
  //   closest_left_lightbar->top, closest_left_lightbar->bottom, closest_right_lightbar->bottom,
  //   closest_right_lightbar->top};
  // tools::draw_points(armor_roi, points2f, {0, 0, 255}, 2);
  // cv::imshow("armor_roi", armor_roi);

  if (
    closest_left_lightbar && closest_right_lightbar &&
    min_distance_br_tr + min_distance_tl_bl < 15) {
    // 将四个点从armor_roi坐标系转换到原始图像坐标系
    armor.points[0] = closest_left_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[1] = closest_right_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[2] = closest_right_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[3] = closest_left_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    return true;
  }

  return false;
}

bool Detector::check_geometry(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_;
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
  auto length_ok = lightbar.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}

bool Detector::check_geometry(const Armor & armor) const
{
  auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
  auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
  auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;

  // 仿真/新场地的额外约束：真实装甲板的两根灯条几乎等长，
  // 且"灯条间距/灯条长度"约 2.1。放宽模式下用这条硬约束排除
  // "把不同装甲板的两根灯条错配成一对"的情况。
  if (relax_type_check_) {
    if (armor.side_ratio > 1.4) return false;
    if (armor.ratio < 1.4 || armor.ratio > 3.6) return false;
  }

  return ratio_ok && side_ratio_ok && rectangular_error_ok;
}

bool Detector::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于分类器的迭代
  if (name_ok && !confidence_ok) save(armor);

  // 出现 5号 则显示 debug 信息。但不过滤。
  if (armor.name == ArmorName::five) tools::logger()->debug("See pattern 5");

  return name_ok && confidence_ok;
}

bool Detector::check_type(const Armor & armor) const
{
  if (relax_type_check_) return true;

  auto name_ok = armor.type == ArmorType::small
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name == ArmorName::one || armor.name == ArmorName::base);

  // 保存异常的图案，用于分类器的迭代
  if (!name_ok) {
    tools::logger()->debug(
      "see strange armor: {} {}", ARMOR_TYPES[armor.type], ARMOR_NAMES[armor.name]);
    save(armor);
  }

  return name_ok;
}

Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
{
  // 色差掩码模式下二值图已经只包含目标颜色，无需再判颜色
  if (color_mask_) return mask_color_;

  if (strict_color_) {
    // 只统计轮廓内部的真实像素。原来的实现采样轮廓边界点，细长灯条上会混入背景，
    // 而且无法区分"纯红/纯蓝灯条"与"橙色场地标记线"，导致仿真/复杂光照下误判。
    cv::Mat mask = cv::Mat::zeros(bgr_img.size(), CV_8U);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, 0, 255, cv::FILLED);
    const cv::Scalar mean = cv::mean(bgr_img, mask);
    const double B = mean[0], G = mean[1], R = mean[2];

    // 灯条是纯色高亮：蓝条 B 远高于 R，红条 R 远高于 B；
    // 场地橙色标线 G/R 比很高（约 0.65），红队灯条 G/R 约 0.36，可据此区分。
    const bool is_blue = B > 100 && R < 0.45 * B && G < 0.80 * B;
    const bool is_red = R > 100 && B < 0.45 * R && G < 0.50 * R;
    if (is_blue) return Color::blue;
    if (is_red) return Color::red;
    return Color::extinguish;  // 既不像灯条也不像可攻击目标，丢弃
  }

  int red_sum = 0, blue_sum = 0;

  for (const auto & point : contour) {
    red_sum += bgr_img.at<cv::Vec3b>(point)[2];
    blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
  }

  return blue_sum > red_sum ? Color::blue : Color::red;
}

cv::Mat Detector::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
{
  // 延长灯条获得装甲板角点
  // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
  auto tl = armor.left.center - armor.left.top2bottom * 1.125;
  auto bl = armor.left.center + armor.left.top2bottom * 1.125;
  auto tr = armor.right.center - armor.right.top2bottom * 1.125;
  auto br = armor.right.center + armor.right.top2bottom * 1.125;

  auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
  auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
  auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
  auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
  auto roi_tl = cv::Point(roi_left, roi_top);
  auto roi_br = cv::Point(roi_right, roi_bottom);
  auto roi = cv::Rect(roi_tl, roi_br);

  return bgr_img(roi);
}

ArmorType Detector::get_type(const Armor & armor)
{
  /// 优先根据当前armor.ratio判断
  /// TODO: 25赛季是否还需要根据比例判断大小装甲？能否根据图案直接判断？

  if (armor.ratio > 3.0) {
    // tools::logger()->debug(
    //   "[Detector] get armor type by ratio: BIG {} {:.2f}", ARMOR_NAMES[armor.name], armor.ratio);
    return ArmorType::big;
  }

  if (armor.ratio < 2.5) {
    // tools::logger()->debug(
    //   "[Detector] get armor type by ratio: SMALL {} {:.2f}", ARMOR_NAMES[armor.name], armor.ratio);
    return ArmorType::small;
  }

  // tools::logger()->debug("[Detector] get armor type by name: {}", ARMOR_NAMES[armor.name]);

  // 英雄、基地只能是大装甲板
  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }

  // 其他所有（工程、哨兵、前哨站、步兵）都是小装甲板
  /// TODO: 基地顶装甲是小装甲板
  return ArmorType::small;
}

cv::Point2f Detector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void Detector::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, armor.pattern);
}

void Detector::show_result(
  const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
  const std::list<Armor> & armors, int frame_count) const
{
  auto detection = bgr_img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  for (const auto & lightbar : lightbars) {
    auto info = fmt::format(
      "{:.1f} {:.1f} {:.1f} {}", lightbar.angle_error * 57.3, lightbar.ratio, lightbar.length,
      COLORS[lightbar.color]);
    tools::draw_text(detection, info, lightbar.top, {0, 255, 255});
    tools::draw_points(detection, lightbar.points, {0, 255, 255}, 3);
  }

  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio,
      armor.rectangular_error * 57.3, armor.confidence, ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});
  }

  cv::Mat binary_img2;
  cv::resize(binary_img, binary_img2, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::resize(detection, detection, {}, 0.5, 0.5);     // 显示时缩小图片尺寸

  // cv::imshow("threshold", binary_img2);
  cv::imshow("detection", detection);
}

void Detector::lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const
{
  // 配置参数
  constexpr float MAX_BRIGHTNESS = 25;  // 归一化最大亮度值
  constexpr float ROI_SCALE = 0.07;     // ROI扩展比例
  constexpr float SEARCH_START = 0.4;   // 搜索起始位置比例（原0.8/2）
  constexpr float SEARCH_END = 0.6;     // 搜索结束位置比例（原1.2/2）

  // 扩展并裁剪ROI
  cv::Rect roi_box = lightbar.rotated_rect.boundingRect();
  roi_box.x -= roi_box.width * ROI_SCALE;
  roi_box.y -= roi_box.height * ROI_SCALE;
  roi_box.width += 2 * roi_box.width * ROI_SCALE;
  roi_box.height += 2 * roi_box.height * ROI_SCALE;

  // 边界约束
  roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);

  // 归一化ROI
  cv::Mat roi = gray_img(roi_box);
  const float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // 计算质心
  const cv::Moments moments = cv::moments(roi);
  const cv::Point2f centroid(
    moments.m10 / moments.m00 + roi_box.x, moments.m01 / moments.m00 + roi_box.y);

  // 生成稀疏点云（优化性能）
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; ++i) {
    for (int j = 0; j < roi.cols; ++j) {
      const float weight = roi.at<float>(i, j);
      if (weight > 1e-3) {          // 忽略极小值提升性能
        points.emplace_back(j, i);  // 坐标相对于ROI区域
      }
    }
  }

  // PCA计算对称轴方向
  cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
  axis /= cv::norm(axis);
  if (axis.y > 0) axis = -axis;  // 统一方向

  const auto find_corner = [&](int direction) -> cv::Point2f {
    const float dx = axis.x * direction;
    const float dy = axis.y * direction;
    const float search_length = lightbar.length * (SEARCH_END - SEARCH_START);

    std::vector<cv::Point2f> candidates;

    // 横向采样多个候选线
    const int half_width = (lightbar.width - 2) / 2;
    for (int i_offset = -half_width; i_offset <= half_width; ++i_offset) {
      // 计算搜索起点
      cv::Point2f start_point(
        centroid.x + lightbar.length * SEARCH_START * dx + i_offset,
        centroid.y + lightbar.length * SEARCH_START * dy);

      // 沿轴搜索亮度跳变点
      cv::Point2f corner = start_point;
      float max_diff = 0;
      bool found = false;

      for (float step = 0; step < search_length; ++step) {
        const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

        // 边界检查
        if (
          cur_point.x < 0 || cur_point.x >= gray_img.cols || cur_point.y < 0 ||
          cur_point.y >= gray_img.rows) {
          break;
        }

        // 计算亮度差（使用双线性插值提升精度）
        const auto prev_val = gray_img.at<uchar>(cv::Point2i(cur_point - cv::Point2f(dx, dy)));
        const auto cur_val = gray_img.at<uchar>(cv::Point2i(cur_point));
        const float diff = prev_val - cur_val;

        if (diff > max_diff && prev_val > mean_val) {
          max_diff = diff;
          corner = cur_point - cv::Point2f(dx, dy);  // 跳变发生在上一位置
          found = true;
        }
      }

      if (found) {
        candidates.push_back(corner);
      }
    }

    // 返回候选点均值
    return candidates.empty()
             ? cv::Point2f(-1, -1)
             : std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0)) /
                 static_cast<float>(candidates.size());
  };

  // 并行检测顶部和底部
  lightbar.top = find_corner(1);
  lightbar.bottom = find_corner(-1);
}

}  // namespace auto_aim