#include "png_writer.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <algorithm>

namespace agent {

namespace {

uint32_t crc32_of(const uint8_t* data, size_t length, uint32_t crc = 0xFFFFFFFFu) {
  static uint32_t table[256];
  static bool built = false;
  if (!built) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      table[i] = c;
    }
    built = true;
  }
  for (size_t i = 0; i < length; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return crc;
}

void push_be32(std::vector<uint8_t>& out, uint32_t value) {
  out.push_back(uint8_t(value >> 24));
  out.push_back(uint8_t(value >> 16));
  out.push_back(uint8_t(value >> 8));
  out.push_back(uint8_t(value));
}

void push_chunk(std::vector<uint8_t>& out, const char type[4], const std::vector<uint8_t>& payload) {
  push_be32(out, uint32_t(payload.size()));

  std::vector<uint8_t> crc_input;
  crc_input.reserve(4 + payload.size());
  crc_input.insert(crc_input.end(), type, type + 4);
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());

  out.insert(out.end(), crc_input.begin(), crc_input.end());
  push_be32(out, crc32_of(crc_input.data(), crc_input.size()) ^ 0xFFFFFFFFu);
}

} // namespace

bool write_png_rgb(const std::string& path,
                   uint32_t width, uint32_t height,
                   const uint8_t* rgb,
                   std::string& error) {
  if (rgb == nullptr || width == 0 || height == 0) {
    error = "empty image";
    return false;
  }

  // Raw scanlines, each prefixed with filter type 0 (None).
  std::vector<uint8_t> raw;
  raw.reserve(size_t(height) * (size_t(width) * 3 + 1));
  for (uint32_t y = 0; y < height; ++y) {
    raw.push_back(0);
    const uint8_t* row = rgb + size_t(y) * size_t(width) * 3;
    raw.insert(raw.end(), row, row + size_t(width) * 3);
  }

  // zlib stream using stored (uncompressed) deflate blocks.
  std::vector<uint8_t> z;
  z.push_back(0x78);  // CMF: deflate, 32K window
  z.push_back(0x01);  // FLG: no dict, check bits make 0x7801 % 31 == 0

  constexpr size_t block_max = 65535;
  size_t offset = 0;
  while (offset < raw.size()) {
    const size_t block = std::min(block_max, raw.size() - offset);
    const bool final_block = (offset + block) >= raw.size();

    z.push_back(final_block ? 1 : 0);
    z.push_back(uint8_t(block & 0xFF));
    z.push_back(uint8_t(block >> 8));
    z.push_back(uint8_t(~block & 0xFF));
    z.push_back(uint8_t((~block >> 8) & 0xFF));
    z.insert(z.end(), raw.begin() + offset, raw.begin() + offset + block);
    offset += block;
  }

  // Adler-32 of the uncompressed data.
  uint32_t a = 1, b = 0;
  for (uint8_t byte : raw) {
    a = (a + byte) % 65521;
    b = (b + a) % 65521;
  }
  push_be32(z, (b << 16) | a);

  std::vector<uint8_t> png = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };

  std::vector<uint8_t> ihdr;
  push_be32(ihdr, width);
  push_be32(ihdr, height);
  ihdr.push_back(8);  // bit depth
  ihdr.push_back(2);  // colour type 2 = truecolour RGB
  ihdr.push_back(0);  // deflate
  ihdr.push_back(0);  // adaptive filtering
  ihdr.push_back(0);  // no interlace
  push_chunk(png, "IHDR", ihdr);
  push_chunk(png, "IDAT", z);
  push_chunk(png, "IEND", {});

  FILE* file = fopen(path.c_str(), "wb");
  if (file == nullptr) {
    error = "cannot open '" + path + "' for writing: " + strerror(errno);
    return false;
  }

  const size_t written = fwrite(png.data(), 1, png.size(), file);
  const bool closed_ok = (fclose(file) == 0);

  if (written != png.size() || !closed_ok) {
    error = "short write to '" + path + "'";
    return false;
  }
  return true;
}

} // namespace agent
