#ifndef LSLIDAR_DRIVER_M10_PACKET_DECODER_HPP
#define LSLIDAR_DRIVER_M10_PACKET_DECODER_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lslidar_driver {

struct M10PacketPoint {
  int azimuth;
  float distance;
};

// M10P 总长度包含 20 字节非点数据；点数据必须按两个字节成对且位于接收缓冲区内。
inline bool isValidM10PPacketSize(size_t packet_size, size_t buffer_size) {
  constexpr size_t kPacketOverhead = 20;
  constexpr size_t kMaximumPacketSize = 180;
  return packet_size >= kPacketOverhead && packet_size <= kMaximumPacketSize &&
         packet_size <= buffer_size && (packet_size - kPacketOverhead) % 2 == 0;
}

// 按原始数据槽位解码 M10/M10P；无效距离只丢点，不改变后续点的名义方位角。
inline std::vector<M10PacketPoint> decodeM10PacketSlots(const uint8_t* data, size_t data_size,
                                                        size_t slot_count, size_t angle_offset,
                                                        size_t data_offset,
                                                        float angle_interval_degrees,
                                                        float distance_resolution) {
  std::vector<M10PacketPoint> decoded_points;
  if (data == nullptr || slot_count == 0 || angle_offset >= data_size ||
      data_size - angle_offset < 2 || data_offset >= data_size ||
      !std::isfinite(angle_interval_degrees) || angle_interval_degrees < 0.0f ||
      !std::isfinite(distance_resolution) || distance_resolution <= 0.0f) {
    return decoded_points;
  }

  const size_t available_slots = (data_size - data_offset) / 2;
  const size_t decodable_slots = std::min(slot_count, available_slots);
  decoded_points.reserve(decodable_slots);

  const int start_angle = ((data[angle_offset] << 8) + data[angle_offset + 1]) % 36000;
  // 协议角度单位为 0.01°；先转成整数再按槽位向下取整，避免 float 边界少 0.01°。
  const int angle_span =
      static_cast<int>(std::lround(static_cast<double>(angle_interval_degrees) * 100.0));
  for (size_t slot_idx = 0; slot_idx < decodable_slots; ++slot_idx) {
    const size_t point_offset = data_offset + slot_idx * 2;
    const uint16_t raw_distance = (data[point_offset] << 8) + data[point_offset + 1];
    if (raw_distance == 0xFFFF) {
      continue;
    }

    M10PacketPoint point;
    const int angle_offset_in_hundredths =
        static_cast<int>(static_cast<int64_t>(angle_span) * static_cast<int64_t>(slot_idx) /
                         static_cast<int64_t>(slot_count));
    point.azimuth = (start_angle + angle_offset_in_hundredths) % 36000;
    point.distance = static_cast<float>(raw_distance) * distance_resolution;
    decoded_points.push_back(point);
  }
  return decoded_points;
}

}  // namespace lslidar_driver

#endif  // LSLIDAR_DRIVER_M10_PACKET_DECODER_HPP
