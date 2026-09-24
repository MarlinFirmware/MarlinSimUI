#pragma once

/**
 * Agent interface global instance and route registration.
 *
 * Handlers registered here run ON THE SIMULATION THREAD (called from
 * Kernel::execute_loop via AgentServer::service), so reading kernel and Marlin
 * state from inside a handler is safe.
 */

#include "agent_server.h"

namespace agent {

extern AgentServer server;

// Register the built-in routes. Call once before start().
void register_routes();

} // namespace agent
