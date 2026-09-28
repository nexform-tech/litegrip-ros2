// litegrip_shm_layout_probe.cpp — print the shared memory structs' size/offsetof
// as JSON.
//
// Purpose: the Python side (test/test_shm_layout.py) mirrors the same structs
// with ctypes, and then the field offsets of the two sides are compared item by
// item. A ctypes Structure can only validate the total size when the .so is
// loaded; a field-level offset drift (say, a field was inserted on the C side
// and the Python side was not updated) is not caught by the total-size check —
// this probe fills that hole.
//
// Example output:
//   {
//     "num_joints": 1,
//     "state_size": 168,
//     "state_offsets": {"position": 0, "velocity": 8, ...},
//     ...
//   }

#include <cstddef>
#include <cstdio>

#include "litegrip_ros2_control/litegrip_shm.h"

#define LITEGRIP_STATE_FIELDS(X) \
  X(position)                    \
  X(velocity)                    \
  X(effort)                      \
  X(temperature_mos)             \
  X(temperature_coil)            \
  X(error_code)                  \
  X(feedback_age_s)              \
  X(feedback_received)           \
  X(fault_code)                  \
  X(stamp_s)                     \
  X(heartbeat_s)                 \
  X(connected)                   \
  X(enabled)                     \
  X(faulted)                     \
  X(dry_run)                     \
  X(latched)                     \
  X(stopped)                     \
  X(cycle_count)                 \
  X(applied_command_cycle)       \
  X(command_age_s)               \
  X(last_error)

#define LITEGRIP_COMMAND_FIELDS(X) \
  X(position)                      \
  X(enable)                        \
  X(estop)                         \
  X(stamp_s)                       \
  X(cycle_count)

#define LITEGRIP_HEADER_FIELDS(X) \
  X(magic)                        \
  X(layout_version)               \
  X(state_seq)                    \
  X(command_seq)                  \
  X(state_publish_count)          \
  X(command_publish_count)        \
  X(state_torn_reads)             \
  X(command_torn_reads)

namespace {

void print_state_offsets() {
  std::printf("\"state_offsets\": {");
  bool first = true;
#define EMIT(field)                                               \
  std::printf("%s\"%s\": %zu", first ? "" : ", ", #field,         \
              offsetof(LitegripState, field));                    \
  first = false;
  LITEGRIP_STATE_FIELDS(EMIT)
#undef EMIT
  std::printf("}");
}

void print_command_offsets() {
  std::printf("\"command_offsets\": {");
  bool first = true;
#define EMIT(field)                                               \
  std::printf("%s\"%s\": %zu", first ? "" : ", ", #field,         \
              offsetof(LitegripCommand, field));                  \
  first = false;
  LITEGRIP_COMMAND_FIELDS(EMIT)
#undef EMIT
  std::printf("}");
}

void print_header_offsets() {
  std::printf("\"header_offsets\": {");
  bool first = true;
#define EMIT(field)                                               \
  std::printf("%s\"%s\": %zu", first ? "" : ", ", #field,         \
              offsetof(LitegripHeader, field));                   \
  first = false;
  LITEGRIP_HEADER_FIELDS(EMIT)
#undef EMIT
  std::printf("}");
}

}  // namespace

int main() {
  std::printf("{\n");
  std::printf("  \"num_joints\": %d,\n", LITEGRIP_SHM_NUM_JOINTS);
  std::printf("  \"magic\": %u,\n", LITEGRIP_SHM_MAGIC);
  std::printf("  \"layout_version\": %u,\n", LITEGRIP_SHM_LAYOUT_VERSION);
  std::printf("  \"state_size\": %zu,\n", sizeof(LitegripState));
  std::printf("  \"command_size\": %zu,\n", sizeof(LitegripCommand));
  std::printf("  \"header_size\": %zu,\n", sizeof(LitegripHeader));
  std::printf("  \"segment_size\": %zu,\n", litegrip_shm_segment_size());
  std::printf("  ");
  print_state_offsets();
  std::printf(",\n  ");
  print_command_offsets();
  std::printf(",\n  ");
  print_header_offsets();
  std::printf("\n}\n");
  return 0;
}
