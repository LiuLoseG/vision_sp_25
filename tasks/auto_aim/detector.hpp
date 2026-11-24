#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim
{

class Detector
{
public:
  Detector(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count = -1);

  bool detect(Armor & armor, const cv::Mat & bgr_img);

  friend class YOLOV8;

private:
  Classifier classifier_;

  double threshold_;
  double max_angle_error_;
  double min_lightbar_ratio_, max_lightbar_ratio_;
  double min_lightbar_length_;
  double min_armor_ratio_, max_armor_ratio_;
  double max_side_ratio_;
  double min_confidence_;
  double max_rectangular_error_;

  bool debug_;
  std::string save_path_;
  // 用 max(B,R) 代替加权灰度作为灯条亮度：灯条是纯色高亮体，
  // 加权灰度会把纯蓝灯条（B高、R低）压暗到阈值以下，仿真渲染尤其明显。
  bool use_max_channel_ = false;
  // 严格色彩判别：取轮廓内部均值并要求颜色足够纯，可排除橙色场地标线等干扰。
  bool strict_color_ = false;
  // 按敌方颜色做色差掩码：只有目标颜色（红或蓝）的亮像素保留，
  // 可完全排除白线/橙色标线等环境干扰，并让灯条轮廓更完整。
  bool color_mask_ = false;
  Color mask_color_ = Color::blue;
  double color_diff_ = 50;
  // 放宽“图案与大小必须匹配”的校验：步兵也可使用 1~5 任意编号，
  // 并且分类器在仿真/新场地上的置信度不可靠。默认关闭。
  bool relax_type_check_ = false;

  // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  bool check_geometry(const Lightbar & lightbar) const;
  bool check_geometry(const Armor & armor) const;
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  void save(const Armor & armor) const;
  void show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP