#pragma once

/**
 * Minimal PNG writer for display captures.
 *
 * Why hand-rolled: no image library is vendored, and a screenshot path should
 * not drag in a dependency. PNG's container is simple, and zlib's "stored"
 * (uncompressed) deflate blocks are legal, so a valid PNG needs only CRC32 and
 * Adler32 -- both a few lines. Display captures are tiny (128x64 for an ST7920,
 * a few hundred KB even for a TFT), so skipping compression costs nothing that
 * matters.
 */

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace agent {

// Write 8-bit RGB pixels (3 bytes per pixel, row-major, top row first) to path.
// Returns false and sets error on failure.
bool write_png_rgb(const std::string& path,
                   uint32_t width, uint32_t height,
                   const uint8_t* rgb,
                   std::string& error);

} // namespace agent
