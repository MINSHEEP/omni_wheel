#ifndef LSLIDAR_DRIVER__SCAN_ANGLE_MASK_HPP_
#define LSLIDAR_DRIVER__SCAN_ANGLE_MASK_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lslidar_driver {

constexpr int kFullCircleCentidegrees = 36000;
constexpr double kRadiansToCentidegrees = 18000.0 / 3.14159265358979323846;

// 将任意角度归一化到雷达协议使用的 [0, 36000) 厘度范围。
inline int normalizeCentidegrees(long int angle) {
  const long int normalized =
      (angle % kFullCircleCentidegrees + kFullCircleCentidegrees) % kFullCircleCentidegrees;
  return static_cast<int>(normalized);
}

// LaserScan 使用逆时针弧度，而该 X10 驱动生成扫描时使用 atan2(-y, x)，
// 与雷达协议方位角方向相反；因此必须先取反再转换，才能与裁剪参数对齐。
inline int scanAngleToLidarCentidegrees(double scan_angle_radians) {
  if (!std::isfinite(scan_angle_radians)) {
    return -1;
  }
  return normalizeCentidegrees(
      static_cast<long int>(std::lround(-scan_angle_radians * kRadiansToCentidegrees)));
}

// 判断某个 LaserScan 槽位是否落入配置的屏蔽扇区。区间采用 [start, end)
// 语义，并显式支持跨越 0° 的区间；start == end 表示不屏蔽任何角度，
// 与驱动既有配置契约保持一致。
inline bool isScanAngleDisabled(double scan_angle_radians, const std::vector<long int>& disable_min,
                                const std::vector<long int>& disable_max) {
  const int azimuth = scanAngleToLidarCentidegrees(scan_angle_radians);
  if (azimuth < 0) {
    return false;
  }

  const std::size_t range_count = std::min(disable_min.size(), disable_max.size());
  for (std::size_t index = 0; index < range_count; ++index) {
    const int start = normalizeCentidegrees(disable_min[index]);
    const int end = normalizeCentidegrees(disable_max[index]);
    if (start == end) {
      continue;
    }
    if ((start < end && azimuth >= start && azimuth < end) ||
        (start > end && (azimuth >= start || azimuth < end))) {
      return true;
    }
  }
  return false;
}

}  // namespace lslidar_driver

#endif  // LSLIDAR_DRIVER__SCAN_ANGLE_MASK_HPP_
