#include "utils/jpeg_draw.h"

#include <algorithm>
#include <cstdio>
#include <jpeglib.h>

#include <csetjmp>
#include <vector>

namespace vehicle {
namespace utils {
namespace {

struct JpegErrorMgr {
  jpeg_error_mgr pub;
  jmp_buf jump;
};

void jpegErrorExit(j_common_ptr cinfo) {
  auto* err = reinterpret_cast<JpegErrorMgr*>(cinfo->err);
  longjmp(err->jump, 1);
}

struct RgbImage {
  std::vector<uint8_t> pixels;
  int width = 0;
  int height = 0;
};

bool decodeJpeg(const std::vector<uint8_t>& jpeg, RgbImage* out) {
  if (jpeg.empty() || out == nullptr) return false;

  jpeg_decompress_struct cinfo{};
  JpegErrorMgr jerr{};
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = jpegErrorExit;
  if (setjmp(jerr.jump)) {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpeg.data(), static_cast<unsigned long>(jpeg.size()));
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }
  cinfo.out_color_space = JCS_RGB;
  jpeg_start_decompress(&cinfo);

  out->width = static_cast<int>(cinfo.output_width);
  out->height = static_cast<int>(cinfo.output_height);
  const int row_stride = out->width * 3;
  out->pixels.assign(static_cast<size_t>(row_stride) * static_cast<size_t>(out->height), 0);

  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW row = out->pixels.data() + static_cast<size_t>(cinfo.output_scanline) * row_stride;
    jpeg_read_scanlines(&cinfo, &row, 1);
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return out->width > 0 && out->height > 0;
}

bool encodeJpeg(const RgbImage& img, int quality, std::vector<uint8_t>* out) {
  if (out == nullptr || img.pixels.empty() || img.width <= 0 || img.height <= 0) return false;

  jpeg_compress_struct cinfo{};
  JpegErrorMgr jerr{};
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = jpegErrorExit;
  if (setjmp(jerr.jump)) {
    jpeg_destroy_compress(&cinfo);
    return false;
  }

  jpeg_create_compress(&cinfo);
  unsigned char* buf = nullptr;
  unsigned long buf_size = 0;
  jpeg_mem_dest(&cinfo, &buf, &buf_size);

  cinfo.image_width = static_cast<JDIMENSION>(img.width);
  cinfo.image_height = static_cast<JDIMENSION>(img.height);
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  const int row_stride = img.width * 3;
  while (cinfo.next_scanline < cinfo.image_height) {
    JSAMPROW row =
        const_cast<JSAMPROW>(img.pixels.data() + static_cast<size_t>(cinfo.next_scanline) * row_stride);
    jpeg_write_scanlines(&cinfo, &row, 1);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  if (buf == nullptr || buf_size == 0) return false;
  out->assign(buf, buf + buf_size);
  free(buf);
  return true;
}

void drawRectRgb(RgbImage* img, int x1, int y1, int x2, int y2, int border_px) {
  if (img == nullptr || img->pixels.empty()) return;
  x1 = std::max(0, std::min(x1, img->width - 1));
  x2 = std::max(0, std::min(x2, img->width - 1));
  y1 = std::max(0, std::min(y1, img->height - 1));
  y2 = std::max(0, std::min(y2, img->height - 1));
  if (x2 <= x1 || y2 <= y1) return;
  border_px = std::max(1, border_px);

  auto put = [&](int x, int y) {
    uint8_t* p = img->pixels.data() + (static_cast<size_t>(y) * img->width + x) * 3;
    p[0] = 0;
    p[1] = 255;
    p[2] = 0;
  };

  for (int t = 0; t < border_px; ++t) {
    const int yt = y1 + t;
    const int yb = y2 - t;
    if (yt <= yb) {
      for (int x = x1; x <= x2; ++x) {
        put(x, yt);
        put(x, yb);
      }
    }
    const int xl = x1 + t;
    const int xr = x2 - t;
    if (xl <= xr) {
      for (int y = y1; y <= y2; ++y) {
        put(xl, y);
        put(xr, y);
      }
    }
  }
}

}  // namespace

bool encodeRgbToJpeg(const uint8_t* rgb, int width, int height, int quality,
                     std::vector<uint8_t>* jpeg_out) {
  if (rgb == nullptr || jpeg_out == nullptr || width <= 0 || height <= 0) return false;
  RgbImage img;
  img.width = width;
  img.height = height;
  img.pixels.assign(rgb, rgb + static_cast<size_t>(width) * static_cast<size_t>(height) * 3u);
  return encodeJpeg(img, quality, jpeg_out);
}

bool drawGreenRectOnJpeg(std::vector<uint8_t>* jpeg, float left, float top, float width,
                         float height, int border_px) {
  if (jpeg == nullptr || jpeg->empty()) return false;
  if (width < 1.0f || height < 1.0f) return false;

  RgbImage img;
  if (!decodeJpeg(*jpeg, &img)) return false;

  const int x1 = static_cast<int>(left);
  const int y1 = static_cast<int>(top);
  const int x2 = static_cast<int>(left + width);
  const int y2 = static_cast<int>(top + height);
  drawRectRgb(&img, x1, y1, x2, y2, border_px);

  std::vector<uint8_t> encoded;
  if (!encodeJpeg(img, 85, &encoded)) return false;
  *jpeg = std::move(encoded);
  return true;
}

}  // namespace utils
}  // namespace vehicle
