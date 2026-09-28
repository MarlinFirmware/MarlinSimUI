//
// Bed markings: origin, safe homing point, probeable area, leveling mesh grid.
// See bed_markings.h.
//

#include "bed_markings.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/gtc/constants.hpp>

#include <src/inc/MarlinConfig.h>
#if HAS_BED_PROBE || HAS_LEVELING
  #include <src/module/probe.h>
#endif
#if HAS_LEVELING
  #include <src/feature/bedlevel/bedlevel.h>
#endif

#undef abs
#undef min
#undef max

using renderer::vertex_data_t;
using VertexBuffer = renderer::Buffer<vertex_data_t>;

namespace {

// Light colors that read well on the dark blue-to-red bed
constexpr glm::vec4 col_origin { 1.00f, 1.00f, 1.00f, 1.0f },  // White
                    col_home   { 1.00f, 0.88f, 0.20f, 1.0f },  // Yellow
                    col_probe  { 0.95f, 0.95f, 0.95f, 1.0f },  // White (dashed)
                    col_grid   { 0.55f, 0.80f, 1.00f, 1.0f };  // Light blue

constexpr float lift = 0.2f;       // Above the bed surface, clear of depth fighting
constexpr float symbol_r = 5.0f,   // Origin / safe-home circle radius
                symbol_w = 0.8f,   // Symbol stroke width
                probe_w = 0.8f,    // Probe area outline width
                grid_w = 0.5f,     // Mesh grid line width
                dash = 6.0f, gap = 4.0f;

// Marlin native XY -> the simulated bed's XY. Cartesian beds start at 0,0; a Delta
// (BED_CENTER_AT_0_0) has its center at 0,0 and the simulator's bed at X_BED_SIZE/2.
const glm::vec2 to_bed { float(X_BED_SIZE) * 0.5f - float(X_CENTER), float(Y_BED_SIZE) * 0.5f - float(Y_CENTER) };

// Flat strokes on the bed, following its surface. Points are Marlin native XY.
struct Painter {
  VertexBuffer& buf;
  const BedMarkings::BedZ& bed_z;

  glm::vec3 at(const glm::vec2 p) const {
    const glm::vec2 s = p + to_bed;
    return { s.x, bed_z(s) + lift, -s.y };
  }

  // Face up (+Y) so back-face culling keeps it
  void tri(glm::vec3 a, glm::vec3 b, glm::vec3 c, const glm::vec4& col) {
    if (glm::cross(b - a, c - a).y < 0) std::swap(b, c);
    constexpr glm::vec3 up { 0, 1, 0 };
    buf.add_vertex({a, up, col});
    buf.add_vertex({b, up, col});
    buf.add_vertex({c, up, col});
  }

  void quad(const glm::vec2 p0, const glm::vec2 p1, const glm::vec2 p2, const glm::vec2 p3, const glm::vec4& col) {
    const glm::vec3 a = at(p0), b = at(p1), c = at(p2), d = at(p3);
    tri(a, b, c, col);
    tri(a, c, d, col);
  }

  // A straight stroke, split into short pieces so it follows a tilted or leveled bed
  void line(const glm::vec2 a, const glm::vec2 b, const float w, const glm::vec4& col) {
    const glm::vec2 d = b - a;
    const float len = glm::length(d);
    if (len <= 0) return;
    const glm::vec2 n = glm::vec2(-d.y, d.x) / len * (w * 0.5f);
    const int pieces = std::max(1, int(std::ceil(len / 10.0f)));
    for (int i = 0; i < pieces; ++i) {
      const glm::vec2 p = a + d * (float(i) / pieces), q = a + d * (float(i + 1) / pieces);
      quad(p - n, q - n, q + n, p + n, col);
    }
  }

  void dashed(const glm::vec2 a, const glm::vec2 b, const float w, const glm::vec4& col) {
    const float len = glm::length(b - a);
    if (len <= 0) return;
    const glm::vec2 u = (b - a) / len;
    for (float s = 0; s < len; s += dash + gap)
      line(a + u * s, a + u * std::min(s + dash, len), w, col);
  }

  void ring(const glm::vec2 c, const float r, const float w, const glm::vec4& col, const bool dashes=false) {
    const int sides = std::max(24, int(r * 0.6f));
    const float ri = r - w * 0.5f, ro = r + w * 0.5f, step = glm::two_pi<float>() / sides;
    for (int i = 0; i < sides; ++i) {
      const float a0 = i * step, a1 = a0 + step;
      if (dashes && std::fmod(a0 * r, dash + gap) >= dash) continue; // Dashes by arc length
      const glm::vec2 d0 { std::cos(a0), std::sin(a0) }, d1 { std::cos(a1), std::sin(a1) };
      quad(c + d0 * ri, c + d0 * ro, c + d1 * ro, c + d1 * ri, col);
    }
  }

  // Circle with a cross (+) or an X through it
  void target(const glm::vec2 c, const glm::vec4& col, const bool diagonal) {
    ring(c, symbol_r, symbol_w, col);
    const float k = symbol_r * 1.4f * (diagonal ? float(M_SQRT1_2) : 1.0f);
    if (diagonal) {
      line(c + glm::vec2(-k, -k), c + glm::vec2(k, k), symbol_w, col);
      line(c + glm::vec2(-k, k), c + glm::vec2(k, -k), symbol_w, col);
    }
    else {
      line(c + glm::vec2(-k, 0), c + glm::vec2(k, 0), symbol_w, col);
      line(c + glm::vec2(0, -k), c + glm::vec2(0, k), symbol_w, col);
    }
  }
};

// Everything the markings are drawn from, read from Marlin
struct Inputs {
  bool has_probe_area = false, round_area = false;
  glm::vec2 probe_min {}, probe_max {};
  float probe_radius = 0;
  std::vector<float> grid_x, grid_y;
  bool has_home = false;
  glm::vec2 home {};

  void gather() {
    #if HAS_BED_PROBE
      has_probe_area = true;
      probe_min = { probe.min_x(), probe.min_y() };
      probe_max = { probe.max_x(), probe.max_y() };
      #if IS_KINEMATIC
        round_area = true;
        probe_radius = probe.probe_radius();
      #endif
    #endif

    // The live mesh if there is one, otherwise the grid G29 would probe
    #if ABL_USES_GRID || HAS_MESH
      bool live = true;
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        live = bedlevel.has_mesh();
      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
        live = false;
      #endif
      for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i) {
        #if HAS_MESH
          if (live) { grid_x.push_back(bedlevel.get_mesh_x(i)); continue; }
        #endif
        grid_x.push_back(probe.min_x() + (probe.max_x() - probe.min_x()) * i / (GRID_MAX_POINTS_X - 1));
      }
      for (uint8_t j = 0; j < GRID_MAX_POINTS_Y; ++j) {
        #if HAS_MESH
          if (live) { grid_y.push_back(bedlevel.get_mesh_y(j)); continue; }
        #endif
        grid_y.push_back(probe.min_y() + (probe.max_y() - probe.min_y()) * j / (GRID_MAX_POINTS_Y - 1));
      }
      UNUSED(live);
    #endif

    #if ENABLED(Z_SAFE_HOMING)
      has_home = true;
      home = { float(Z_SAFE_HOMING_X_POINT), float(Z_SAFE_HOMING_Y_POINT) };
    #endif
  }

  std::vector<float> signature() const {
    std::vector<float> s { float(has_probe_area), probe_min.x, probe_min.y, probe_max.x, probe_max.y, probe_radius,
                           float(has_home), home.x, home.y, float(grid_x.size()), float(grid_y.size()) };
    s.insert(s.end(), grid_x.begin(), grid_x.end());
    s.insert(s.end(), grid_y.begin(), grid_y.end());
    return s;
  }
};

} // namespace

void BedMarkings::create(std::shared_ptr<renderer::ShaderProgram> program) {
  mesh = renderer::create_mesh();
  auto m = renderer::get_mesh_by_id(mesh);
  m->set_shader_program(program);
  m->m_shader_instance->set_uniform("u_tint", &no_tint); // Uniforms are per program; don't inherit the bed tint
  m->buffer_vector<vertex_data_t>().push_back(VertexBuffer::create());
  m->m_visible = visible;
  signature.clear();
}

bool BedMarkings::update(const BedZ& bed_z, const bool force) {
  Inputs in;
  in.gather();
  std::vector<float> sig = in.signature();
  if (!force && sig == signature) return false;
  signature = std::move(sig);

  auto m = renderer::get_mesh_by_id(mesh);
  if (!m) return false;
  auto buf = m->buffer<vertex_data_t>();
  buf->data().clear();
  Painter p { *buf, bed_z };

  // Mesh grid first so the outline and symbols draw over it
  if (!in.grid_x.empty() && !in.grid_y.empty()) {
    const float x0 = in.grid_x.front(), x1 = in.grid_x.back(),
                y0 = in.grid_y.front(), y1 = in.grid_y.back();
    for (const float x : in.grid_x) p.line({x, y0}, {x, y1}, grid_w, col_grid);
    for (const float y : in.grid_y) p.line({x0, y}, {x1, y}, grid_w, col_grid);
  }

  if (in.has_probe_area) {
    if (in.round_area)
      p.ring({float(X_CENTER), float(Y_CENTER)}, in.probe_radius, probe_w, col_probe, true);
    else {
      const glm::vec2 a = in.probe_min, c = in.probe_max, b { c.x, a.y }, d { a.x, c.y };
      p.dashed(a, b, probe_w, col_probe);
      p.dashed(b, c, probe_w, col_probe);
      p.dashed(c, d, probe_w, col_probe);
      p.dashed(d, a, probe_w, col_probe);
    }
  }

  if (in.has_home) p.target(in.home, col_home, true);  // Safe homing point: circle with an X
  p.target({0, 0}, col_origin, false);                 // Origin: circle with a cross

  return true;
}

void BedMarkings::set_visible(const bool vis) {
  visible = vis;
  if (auto m = renderer::get_mesh_by_id(mesh)) m->m_visible = vis;
}
