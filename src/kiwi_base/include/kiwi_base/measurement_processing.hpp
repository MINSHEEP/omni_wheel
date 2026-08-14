/**
 * @file measurement_processing.hpp
 * @brief 提供传感器符号修正和静止死区处理。
 */
#pragma once

#include <array>

namespace kiwi_base
{

using Vector3 = std::array<double, 3>;

/**
 * 逐轴应用符号和死区：result = sign * raw，绝对值不超过 deadband 时输出零。
 * 非有限输入也输出零，避免异常反馈进入里程计积分或 EKF。
 */
Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband);

/**
 * 逐轴应用符号、死区和比例：先过滤未缩放的静止噪声，再对有效测量应用 scale。
 * scale 只用于传感器量程修正，必须为正数，不能替代坐标符号 sign。
 */
Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband, const Vector3 &scale);

}  // namespace kiwi_base
