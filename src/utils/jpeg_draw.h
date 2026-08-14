// Vẽ bbox lên JPEG đã encode (CPU) — không phụ thuộc nvdsosd/nvll_osd.
#pragma once

#include <cstdint>
#include <vector>

namespace vehicle {
namespace utils {

// Encode RGB24 (row-major, stride = width*3) → JPEG.
bool encodeRgbToJpeg(const uint8_t* rgb, int width, int height, int quality,
                     std::vector<uint8_t>* jpeg_out);

// Vẽ khung xanh lá (border_px) lên JPEG in-place. Trả về false nếu decode/encode lỗi.
bool drawGreenRectOnJpeg(std::vector<uint8_t>* jpeg, float left, float top, float width,
                         float height, int border_px = 3);

}  // namespace utils
}  // namespace vehicle
