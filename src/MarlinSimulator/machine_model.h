#pragma once

//
// Procedural 3D printer models for the Viewport: Bedslinger, Cube (Z-bed) and Delta.
// Geometry is sized from the Marlin configuration and animated from the nozzle position.
//
// GL coordinates: X = Marlin X, Y = Marlin Z (up), Z = -Marlin Y.
//

#include <functional>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "renderer/renderer.h"

// Simple-geometry models size themselves from the Marlin config.
enum MachineType : uint8_t { MACHINE_BEDSLINGER, MACHINE_CUBE, MACHINE_DELTA, MACHINE_TYPE_COUNT };

// Placement of the moving parts for one frame
struct MachinePose {
  glm::vec3 nozzle {};        // Nozzle position in Marlin coordinates (mm), bed-relative
  glm::vec3 bed_offset {};    // World translation of the bed and everything printed on it (GL)
  glm::vec3 nozzle_world {};  // Nozzle position in world GL coordinates
  bool has_towers = false;    // Delta: 'towers' holds the simulated carriage heights
  glm::vec3 towers {};        // Delta: carriage heights (A, B, C) from the tower steppers
};

// Delta geometry, from the Marlin config (DELTA_RADIUS, DELTA_DIAGONAL_ROD,
// DELTA_HEIGHT, PRINTABLE_RADIUS). This is the simulated machine's physical
// build, so runtime M665 changes (Marlin's belief) intentionally don't alter it.
struct DeltaGeometry {
  float radius, diagonal_rod, height, printable_radius;
};

// Set by --machine on the command line; MACHINE_TYPE_COUNT = use the Marlin config
extern MachineType machine_type_option;
MachineType machine_type_from_name(const char* name); // MACHINE_TYPE_COUNT if unknown

class MachineModel {
public:
  static const char* type_name(const MachineType type);
  static MachineType default_type();  // From the Marlin kinematics (DELTA, IS_CORE, ...)
  static bool is_available(const MachineType type); // DELTA builds: Delta only. Otherwise: all but Delta.
  static DeltaGeometry default_delta();

  void build(const MachineType type, std::shared_ptr<renderer::ShaderProgram> program);
  void destroy();

  MachinePose pose(const glm::vec3 nozzle) const;  // Nozzle in Marlin coordinates
  void update(const MachinePose& pose);
  void queue_render() const;                        // Add all parts to the render list
  void set_visible(const bool visible);

  // World-space bounds of the machine at rest, for camera framing
  glm::vec3 bounds_min {}, bounds_max {};

  MachineType type = MACHINE_BEDSLINGER;
  DeltaGeometry delta = default_delta();
  bool visible = true;

private:
  using Animate = std::function<void(renderer::Mesh&, const MachinePose&)>;
  using Geometry = std::function<void(renderer::Buffer<renderer::vertex_data_t>&)>;
  struct Part {
    renderer::mesh_id_t mesh;
    Animate animate;
  };

  renderer::mesh_id_t add_part(Geometry geometry, Animate animate = nullptr, const bool in_bounds = true);
  void build_bedslinger();
  void build_cube();
  void build_delta();

  std::vector<Part> parts;
  std::shared_ptr<renderer::ShaderProgram> program;
};
