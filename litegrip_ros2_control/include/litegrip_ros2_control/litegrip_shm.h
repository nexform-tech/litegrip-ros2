// litegrip_shm.h — shared memory contract between the LiteGrip gripper hardware
// daemon and the ros2_control plugin.
//
// This is the single source of truth for the layout across languages
// (C++ plugin / Python daemon):
//   * C++ side: includes this header directly.
//   * Python side: calls the C API of liblitegrip_shm.so through ctypes and
//     mirrors LitegripState / LitegripCommand with ctypes.Structure (the field
//     order must match byte for byte; test/test_shm_layout.py cross-checks it).
//
// Why copy this arrangement from litearm_ros2_control (instead of letting the
// plugin call Python directly)
// ---------------------------------------------------------------------------
// A ros2_control hardware component **must be a C++ pluginlib plugin**, while
// the gripper's off-the-shelf driver and safety layer are Python (the LiteGrip
// SDK is Python only). So the layering has to be:
//
//     C++ SystemInterface (real-time side, memcpy + conversion only)
//        ⇅ seqlock shared memory (this file)
//     Python daemon (non-RT side: owns can0, runs the safety layer, sends MIT frames)
//
// Exactly isomorphic to the arm, so the debugging tools, parameter style and
// readiness checks match on both sides — one less mental model to carry.
//
// Design points
// -------------
// 1. Every struct member is a double (8-byte natural alignment) → no implicit
//    padding and no ambiguity across languages; the size and offset of each
//    field are pinned down by static assertions (see litegrip_shm.cpp).
// 2. Each data block has its own seqlock (single writer / single reader,
//    lock-free):
//      state   block: daemon writes, ROS reads — the RT reader never blocks
//      command block: ROS writes, daemon reads — the RT writer never blocks
//    The caller supplies the reader's retry limit; exceeding it returns
//    LITEGRIP_SHM_TORN so the layer above can pick a degradation strategy.
// 3. The seqlock's acquire/release semantics live entirely on the C++ side (see
//    litegrip_shm.cpp); Python only memcpys whole structs in and out and never
//    touches shared memory directly, so no memory ordering has to be expressed
//    in Python.
// 4. Timestamps uniformly use CLOCK_MONOTONIC seconds (on Linux Python's
//    time.monotonic() and C++ std::chrono::steady_clock share one source), so
//    the two sides can compare them directly for timeout decisions.
// 5. **The unit is the motor angle in rad** (not the opening in m). Rationale:
//    the gripper's red line, torque budget allocation and MIT frame
//    quantization are all defined and verified in motor angle; converting
//    opening (m) ⇄ motor angle (rad) is a **presentation-layer** concern and
//    lives in the C++ plugin (using the calibration parameters passed in
//    through the URDF).
//    ⇒ "one number, one unit" holds on both sides: always rad in shm, always
//      opening in m on the ROS joint side.

#ifndef LITEGRIP_ROS2_CONTROL__LITEGRIP_SHM_H_
#define LITEGRIP_ROS2_CONTROL__LITEGRIP_SHM_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of joints this hardware component controls: always 1.
 *
 * A real LiteGrip has only one transmission; the two fingers cannot be
 * controlled independently, and the only joint exposed is the master joint
 * (gripper_opening_joint). The two fingers are its <mimic> and have no
 * independent command interface.
 */
#define LITEGRIP_SHM_NUM_JOINTS 1

/** Default shared memory object name (POSIX shm, must start with '/'). */
#define LITEGRIP_SHM_DEFAULT_NAME "/litegrip_hw"

/** Layout magic value 'L''G''R''P'. */
#define LITEGRIP_SHM_MAGIC 0x4C475250u

/** Layout version. Any field added or removed must bump it, and old segments
 *  are rebuilt. */
#define LITEGRIP_SHM_LAYOUT_VERSION 1u

#define LITEGRIP_SHM_OK 0
#define LITEGRIP_SHM_ERR_GENERIC (-1)
#define LITEGRIP_SHM_ERR_OPEN (-2)
#define LITEGRIP_SHM_ERR_TRUNCATE (-3)
#define LITEGRIP_SHM_ERR_MAP (-4)
#define LITEGRIP_SHM_ERR_LAYOUT (-5)
#define LITEGRIP_SHM_ERR_VERSION (-6)
#define LITEGRIP_SHM_ERR_INVALID_ARG (-7)
/** A torn read was still detected after max_retries attempts; the output keeps
 *  its pre-call contents. */
#define LITEGRIP_SHM_TORN 1

/* ──── LitegripState.last_error: daemon suppression / status codes ──── */
/*
 * As with the arm, the daemon has exactly two behaviours: follow the command
 * frame, or **not follow it** (hold/stop). last_error says why it is currently
 * not following — normal following is LITEGRIP_DAEMON_OK.
 * These codes are for diagnostics/reporting only and do not affect the seqlock
 * layout.
 */
#define LITEGRIP_DAEMON_OK 0
/** Not yet connected to the hardware (daemon starting up / SDK unavailable / no
 *  fresh feedback before enabling). */
#define LITEGRIP_DAEMON_CONNECTING 1
/** Command frame is stale: the ROS-side control loop has stopped publishing
 *  (process exited / controller crashed). */
#define LITEGRIP_DAEMON_HOLDING_STALE_COMMAND 2
/** The ROS side requested a soft emergency stop. */
#define LITEGRIP_DAEMON_HOLDING_ESTOP 3
/** A motor fault is present (the SDK's error_code is neither 0 nor 1). */
#define LITEGRIP_DAEMON_HOLDING_MOTOR_FAULT 4
/** Feedback is missing or timed out (not even a poll frame gets a reply). */
#define LITEGRIP_DAEMON_HOLDING_FEEDBACK_STALE 5
/** Temperature reached this layer's protection threshold (stop **before** the
 *  driver trips by itself). */
#define LITEGRIP_DAEMON_HOLDING_OVERTEMP 6
/** The ROS side requested disable (enable=0); the motor then goes limp and the
 *  gripper can be pushed by an external force. */
#define LITEGRIP_DAEMON_DISABLED 7
/** The daemon is in its shutdown sequence. */
#define LITEGRIP_DAEMON_SHUTTING_DOWN 9
/** The command frame held a non-finite number or a value outside the
 *  representable/red-line range; this frame was rejected. */
#define LITEGRIP_DAEMON_HOLDING_BAD_COMMAND 10
/** ★ A safe stop has been **latched**: it cannot self-recover; a human must
 *  investigate and restart the daemon. */
#define LITEGRIP_DAEMON_LATCHED_SAFE_STOP 11

/* ──────────────────────── daemon → ROS: state block ──────────────────────── */

/**
 * Gripper state plus daemon health. The daemon publishes it once per control
 * cycle.
 *
 * The first three fields (position/velocity/torque) map one to one onto the arm
 * joint's position/velocity/effort — this is where "feedback like any other
 * joint" lands: joint_state_broadcaster treats both identically, so the layers
 * above need not distinguish the device.
 *
 * ⚠ The **semantic boundaries** of the last three fields (keeping the old
 *   driver's convention, tightened to one sentence each):
 *   error_code      raw health code: 0=disabled 1=enabled, anything else is a
 *                   fault. **1 means enabled, not an error** — that value must
 *                   never be written into fault_code.
 *   fault_code      this layer's unified fault code (see LitegripState.fault_code)
 *   last_error      why the daemon is **not following** (LITEGRIP_DAEMON_*)
 *   The three do different jobs and must never stand in for one another.
 */
typedef struct LitegripState {
  /** Motor angle in rad. **Mind the unit**: this is the driver-side reading,
   *  not the opening in m. */
  double position[LITEGRIP_SHM_NUM_JOINTS];
  /** Motor angular velocity in rad/s. */
  double velocity[LITEGRIP_SHM_NUM_JOINTS];
  /** Measured torque in N·m (for DM this is a current estimate; it includes
   *  friction and is fairly noisy). */
  double effort[LITEGRIP_SHM_NUM_JOINTS];
  /** MOS temperature in °C. */
  double temperature_mos[LITEGRIP_SHM_NUM_JOINTS];
  /** Coil/rotor temperature in °C. */
  double temperature_coil[LITEGRIP_SHM_NUM_JOINTS];

  /** Raw health code (0=disabled 1=enabled, anything else is a fault).
   *  **Passed through verbatim, never interpreted**. */
  double error_code[LITEGRIP_SHM_NUM_JOINTS];
  /** Time since the most recent feedback, in s; -1 if none was ever received. */
  double feedback_age_s[LITEGRIP_SHM_NUM_JOINTS];
  /** Total number of feedback frames received (to tell whether the feedback
   *  path is alive). */
  double feedback_received[LITEGRIP_SHM_NUM_JOINTS];
  /**
   * This layer's unified fault code. Ranges:
   *   0        no fault
   *   1        rejected by the safety gate (out of range / non-finite / over a ceiling)
   *   2        no feedback (communication lost / SDK unavailable / sampling error)
   *   3        bridge-layer internal error (contract violation)
   *   4        the initial state before enabling is unsafe
   *   5        hardware safe stop (latched)
   *   100 + n  motor fault, n = the SDK's raw error_code (losslessly recoverable)
   * The motor range starts at 100 to keep it apart from the bridge codes —
   * otherwise motor fault 8 would collide with the number 8 reserved by the
   * bridge layer.
   */
  double fault_code[LITEGRIP_SHM_NUM_JOINTS];

  /** CLOCK_MONOTONIC instant this state frame corresponds to, in s. */
  double stamp_s;
  /** Daemon heartbeat instant in s (same source as stamp_s; used to decide
   *  whether the daemon is alive). */
  double heartbeat_s;
  /** 1 if the daemon has successfully connected to the hardware (dry-run
   *  included). */
  double connected;
  /** 1 if the motor is enabled. */
  double enabled;
  /** 1 if a fault code is present. */
  double faulted;
  /** 1 if the daemon is running in dry-run (no hardware) mode. */
  double dry_run;
  /** ★ 1 if a safe stop is latched: no further motion frames are sent and it
   *  **does not self-recover**. */
  double latched;
  /**
   * Whether it is stopped: 1 when |velocity| < 1e-3 (strictly less than).
   *
   * ⚠ **A weak predicate; do not use it on its own**: dq in a DM4310 feedback
   * frame is a 12-bit field (range ±30 rad/s), one LSB is ≈ 0.0147 rad/s, and
   * **no code in those 12 bits represents 0 exactly** — the closest code decodes
   * to −30/4095 = −0.007326 rad/s (the decoded value of the zero-speed code).
   * Since |−0.0073| is not below 1e-3 ⇒ **as long as feedback is alive this
   * field is in practice always 0**. So its real meaning is "**no evidence of a
   * stop was observed**", not "a standstill was observed". That direction is the
   * conservative one (reporting a standstill as "not stopped" errs on the safe
   * side). ⇒ To decide whether the gripper has really stopped, look at
   * connected / fault_code / last_error together.
   */
  double stopped;
  /** Cumulative count of the daemon's control cycles. */
  double cycle_count;
  /** cycle of the last command the daemon actually sent (lets the ROS side
   *  confirm that a command took effect). */
  double applied_command_cycle;
  /** Age of the command frame as observed by the daemon, in s (above the
   *  threshold it has entered hold). */
  double command_age_s;
  /** Reason for the most recent "not following" (LITEGRIP_DAEMON_*). */
  double last_error;
} LitegripState;

/* ──────────────────────── ROS → daemon: command block ─────────────────────── */

/**
 * Desired state. The ROS side publishes it once per control cycle.
 *
 * ★★ **Only position is exposed** (decided by the user on 2026-09-23): the
 *    command side matches the arm joints, "give position only, MIT underneath".
 *    So there are **no** velocity or torque fields here — they are not
 *    "interfaces that were deleted" but **daemon parameters**:
 *
 *      trajectory advance rate ceiling (formerly GripperCommand.max_velocity_rad_s)
 *          → the max_velocity_rad_s parameter of hw_daemon.py
 *      torque budget (formerly GripperCommand.torque_limit_nm)
 *          → the torque_limit_nm parameter of hw_daemon.py
 *
 *    Parameterized rather than per-command: they describe "how this gripper is
 *    allowed to move" and belong to the deployment configuration; they must not
 *    be decided by each command publisher individually (otherwise one slip of
 *    the hand could loosen the whole torque argument). Changing them requires
 *    restarting the daemon, and that too is deliberate.
 *
 * ⚠ Position is a **motor angle in rad** (the only unit in shm, see point 5 at
 *   the top of the file). The red line, the torque budget allocation and MIT
 *   quantization are all defined and verified in this unit.
 */
typedef struct LitegripCommand {
  /** Target motor angle in rad. Any value beyond the red line or non-finite
   *  makes the daemon reject the whole frame. */
  double position[LITEGRIP_SHM_NUM_JOINTS];

  /** 1 = the ROS side requests enable / keep enabled; 0 = requests disable. */
  double enable;
  /** Soft emergency stop: when non-zero the daemon stops following commands and
   *  enters hold, until it is cleared and enable is requested again. */
  double estop;
  /** CLOCK_MONOTONIC instant of this command frame, in s (the daemon uses it to
   *  decide whether the command is stale). */
  double stamp_s;
  /** Publication counter on the ROS side (+1 per publish; used to diagnose
   *  dropped frames / liveness). */
  double cycle_count;
} LitegripCommand;

/* ────────────────────────────── header and handle ────────────────────────── */

/** Segment header, for diagnostics and readiness checks. */
typedef struct LitegripHeader {
  uint32_t magic;
  uint32_t layout_version;
  uint64_t state_seq;
  uint64_t command_seq;
  uint64_t state_publish_count;
  uint64_t command_publish_count;
  uint64_t state_torn_reads;
  uint64_t command_torn_reads;
} LitegripHeader;

/** Opaque handle (really points at the internal mmap context). */
typedef void *litegrip_shm_handle_t;

/* ───────────────────────────── C API ───────────────────────────── */

/** Byte size of LitegripState (used by the Python side to verify the layout). */
size_t litegrip_shm_state_size(void);
/** Byte size of LitegripCommand. */
size_t litegrip_shm_command_size(void);
/** Byte size of LitegripHeader. */
size_t litegrip_shm_header_size(void);
/** Total size of the shared memory segment. */
size_t litegrip_shm_segment_size(void);

/**
 * Open (optionally create) the shared memory segment.
 *
 * With create != 0: create and initialize the segment if it does not exist; if
 * it exists but the magic/version does not match (a stale segment), delete and
 * rebuild it. With create == 0: return LITEGRIP_SHM_ERR_LAYOUT if the segment
 * does not exist.
 *
 * Returns LITEGRIP_SHM_OK and writes the handle on success; a negative error
 * code on failure.
 */
int litegrip_shm_open(const char *name, int create, litegrip_shm_handle_t *out);

/** Unmap and close the handle (idempotent; the caller is responsible for
 *  nulling the handle). */
void litegrip_shm_close(litegrip_shm_handle_t handle);

/** Remove the shared memory object (call it after all users have exited;
 *  returns LITEGRIP_SHM_OK if it does not exist). */
int litegrip_shm_unlink(const char *name);

/**
 * Publish state (daemon side, single writer).
 *
 * Internally: seqlock to odd → memcpy → release fence → seqlock to even.
 */
int litegrip_shm_publish_state(litegrip_shm_handle_t handle,
                              const LitegripState *state);

/**
 * Read state (ROS side, single reader).
 *
 * max_retries < 0 means retry forever (an RT control loop is advised to use 0
 * or a few retries and then degrade).
 * Returns LITEGRIP_SHM_OK on success; when the retries are exhausted it returns
 * LITEGRIP_SHM_TORN (*out is left unmodified).
 */
int litegrip_shm_read_state(litegrip_shm_handle_t handle, LitegripState *out,
                           int max_retries);

/** Publish a command (ROS side, single writer). */
int litegrip_shm_publish_command(litegrip_shm_handle_t handle,
                                const LitegripCommand *command);

/** Read a command (daemon side, single reader). Same semantics as
 *  litegrip_shm_read_state. */
int litegrip_shm_read_command(litegrip_shm_handle_t handle,
                             LitegripCommand *out, int max_retries);

/** Read the header (lock-free snapshot, for diagnostics/readiness checks only;
 *  no consistency guarantee). */
int litegrip_shm_read_header(litegrip_shm_handle_t handle, LitegripHeader *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LITEGRIP_ROS2_CONTROL__LITEGRIP_SHM_H_
