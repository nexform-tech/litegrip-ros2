#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python-side binding (ctypes) for the shared-memory contract.

The contract itself is defined in
``include/litegrip_ros2_control/litegrip_shm.h`` — this module is merely a
consumer of it and does not redefine the layout semantics:

* The struct field order/width is byte-for-byte identical to the C side, and
  is cross-checked by :func:`describe_layout` against the C-side probe
  (``litegrip_shm_layout_probe``) (see ``test/test_shm_layout.py``).
* The seqlock's acquire/release semantics are implemented entirely on the C
  side; the Python side does no raw memory access at all, only whole-struct +
  length transfers in and out, so there is no need to express memory ordering
  in Python.

Timestamp convention: ``stamp_s`` / ``heartbeat_s`` are both
``CLOCK_MONOTONIC`` seconds, the same source as :func:`time.monotonic` under
Linux, and directly comparable with the C++ ``steady_clock``.

★ Units: ``position`` / ``velocity`` in shm are **motor angle in rad and
  rad/s**, not opening in m. Converting opening (m) ⇄ motor angle (rad) is the
  job of the **presentation layer** (the C++ plugin), because the red lines,
  the torque budget and MIT quantization are all defined and verified in motor
  angle.
"""

import ctypes
import ctypes.util
import os
from pathlib import Path
from typing import List, Optional

NUM_JOINTS = 1
"""Number of controllable joints, matching LITEGRIP_SHM_NUM_JOINTS on the C
side (a real gripper has only one drive train)."""

DEFAULT_SHM_NAME = "/litegrip_hw"
"""Default shared-memory object name."""

SHM_OK = 0
SHM_TORN = 1
SHM_ERR_OPEN = -2
SHM_ERR_LAYOUT = -5
SHM_ERR_VERSION = -6

# ── LitegripState.last_error: daemon suppression/status reason codes ──
# Must match LITEGRIP_DAEMON_* in include/litegrip_ros2_control/litegrip_shm.h.
DAEMON_OK = 0
DAEMON_CONNECTING = 1
DAEMON_HOLDING_STALE_COMMAND = 2
DAEMON_HOLDING_ESTOP = 3
DAEMON_HOLDING_MOTOR_FAULT = 4
DAEMON_HOLDING_FEEDBACK_STALE = 5
DAEMON_HOLDING_OVERTEMP = 6
DAEMON_DISABLED = 7
DAEMON_SHUTTING_DOWN = 9
DAEMON_HOLDING_BAD_COMMAND = 10
DAEMON_LATCHED_SAFE_STOP = 11

DAEMON_STATUS_TEXT = {
    DAEMON_OK: "following ros2_control commands",
    DAEMON_CONNECTING: "not yet connected to the gripper (starting up / SDK "
                       "unavailable / no fresh feedback before enabling)",
    DAEMON_HOLDING_STALE_COMMAND: "command frame is stale: the ros2_control "
                                  "side control loop stopped publishing",
    DAEMON_HOLDING_ESTOP: "software emergency stop in progress",
    DAEMON_HOLDING_MOTOR_FAULT: "motor fault present",
    DAEMON_HOLDING_FEEDBACK_STALE: "feedback missing or timed out",
    DAEMON_HOLDING_OVERTEMP: "temperature reached this layer's protection "
                             "threshold",
    DAEMON_DISABLED: "disable requested (motor is de-energized; the gripper "
                     "can be pushed by an external force)",
    DAEMON_SHUTTING_DOWN: "daemon is shutting down",
    DAEMON_HOLDING_BAD_COMMAND: "command frame contained a non-finite number "
                                "or a value out of the representable range; "
                                "rejected",
    DAEMON_LATCHED_SAFE_STOP: "★ safe stop latched (not self-recoverable; "
                              "restart only after manual investigation)",
}

_ERR_TEXT = {
    SHM_OK: "ok",
    SHM_TORN: "torn read (writer was updating, retries exhausted)",
    SHM_ERR_OPEN: "cannot open the shared-memory segment",
    -1: "generic error",
    -3: "ftruncate failed",
    -4: "mmap failed",
    SHM_ERR_LAYOUT: "shared-memory segment does not exist or its size does "
                    "not match",
    SHM_ERR_VERSION: "shared-memory layout version mismatch",
    -7: "invalid argument",
}

# Fault code segmentation (consistent with safety_gate.FaultCode): motor codes
# start at 100, so that motor fault 8 and the bridge layer's reserved 8 do not
# collide on the same number.
MOTOR_FAULT_BASE = 100


def error_text(code: int) -> str:
    """Render a shared-memory return code as text (unknown codes are passed
    through as-is, not swallowed)."""
    return _ERR_TEXT.get(code, f"unknown return code {code}")


class ShmError(RuntimeError):
    """A shared-memory operation failed."""

    def __init__(self, message: str, code: int = 0) -> None:
        super().__init__(f"{message} (code={code})")
        self.code = code


# ─────────────────────────── struct mirrors ───────────────────────────
# Field order must match litegrip_shm.h exactly; everything is double, so
# there is no implicit padding and the cross-language layout is unambiguous.

StateFields = [
    ("position", ctypes.c_double * NUM_JOINTS),
    ("velocity", ctypes.c_double * NUM_JOINTS),
    ("effort", ctypes.c_double * NUM_JOINTS),
    ("temperature_mos", ctypes.c_double * NUM_JOINTS),
    ("temperature_coil", ctypes.c_double * NUM_JOINTS),
    ("error_code", ctypes.c_double * NUM_JOINTS),
    ("feedback_age_s", ctypes.c_double * NUM_JOINTS),
    ("feedback_received", ctypes.c_double * NUM_JOINTS),
    ("fault_code", ctypes.c_double * NUM_JOINTS),
    ("stamp_s", ctypes.c_double),
    ("heartbeat_s", ctypes.c_double),
    ("connected", ctypes.c_double),
    ("enabled", ctypes.c_double),
    ("faulted", ctypes.c_double),
    ("dry_run", ctypes.c_double),
    ("latched", ctypes.c_double),
    ("stopped", ctypes.c_double),
    ("cycle_count", ctypes.c_double),
    ("applied_command_cycle", ctypes.c_double),
    ("command_age_s", ctypes.c_double),
    ("last_error", ctypes.c_double),
]

CommandFields = [
    ("position", ctypes.c_double * NUM_JOINTS),
    ("enable", ctypes.c_double),
    ("estop", ctypes.c_double),
    ("stamp_s", ctypes.c_double),
    ("cycle_count", ctypes.c_double),
]

HeaderFields = [
    ("magic", ctypes.c_uint32),
    ("layout_version", ctypes.c_uint32),
    ("state_seq", ctypes.c_uint64),
    ("command_seq", ctypes.c_uint64),
    ("state_publish_count", ctypes.c_uint64),
    ("command_publish_count", ctypes.c_uint64),
    ("state_torn_reads", ctypes.c_uint64),
    ("command_torn_reads", ctypes.c_uint64),
]


class LitegripState(ctypes.Structure):
    """Gripper state and health, daemon → ros2_control. Field semantics are in
    litegrip_shm.h."""

    _fields_ = StateFields

    def summary(self) -> str:
        """One-line summary for logging (so it is not assembled separately in
        each place)."""
        return (
            f"pos={self.position[0]:+.4f} rad vel={self.velocity[0]:+.4f} rad/s "
            f"tau={self.effort[0]:+.3f} Nm enabled={bool(self.enabled)} "
            f"fault={int(self.fault_code[0])} last_error={int(self.last_error)}"
        )


class LitegripCommand(ctypes.Structure):
    """Command, ros2_control → daemon. **Position is the only quantity**, see
    litegrip_shm.h."""

    _fields_ = CommandFields


class LitegripHeader(ctypes.Structure):
    """Segment header diagnostic information."""

    _fields_ = HeaderFields


# ────────────────────── locating and loading the library ──────────────────


def _candidate_lib_paths() -> List[Path]:
    """List the candidate paths for liblitegrip_shm.so in priority order.

    Both runtime layouts must be covered:

    * ROS install tree:
      ``<prefix>/lib/python3.10/site-packages/litegrip_ros2_control/``
      → going up 3 levels gives ``<prefix>/lib/``.
    * Running straight from the source tree (when running unit tests):
      ``<ws>/src/litegrip/litegrip_ros2_control/python/...``
      → look in ``<ws>/build/litegrip_ros2_control/`` for the colcon build
      artifacts.
    """
    candidates: List[Path] = []

    override = os.environ.get("LITEGRIP_SHM_LIB")
    if override:
        candidates.append(Path(override))

    here = Path(__file__).resolve()
    package_root = here.parents[2]          # .../litegrip_ros2_control
    workspace_root = package_root.parents[2]  # .../<ws> (src/litegrip/<pkg> ⇒ up 2)
    if not (workspace_root / "src").is_dir():
        # If the layout is different (e.g. installed directly under a prefix),
        # fall back one level so the candidate paths do not end up wrong.
        workspace_root = package_root.parents[1]

    # 1) ROS install tree
    candidates.append(here.parents[3] / "liblitegrip_shm.so")
    # 2) colcon build tree
    candidates.append(workspace_root / "build" / package_root.name /
                      "liblitegrip_shm.so")
    # 3) colcon install tree (built but not sourced)
    candidates.append(workspace_root / "install" / package_root.name / "lib" /
                      "liblitegrip_shm.so")
    # 4) any AMENT_PREFIX_PATH prefix
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "lib" / "liblitegrip_shm.so")
    # 5) leave it to the dynamic linker
    found = ctypes.util.find_library("litegrip_shm")
    if found:
        candidates.append(Path(found))

    return candidates


def _declare_signatures(lib: ctypes.CDLL) -> None:
    """Declare the C API signatures (without declarations, ctypes truncates
    64-bit return values to int)."""
    size_t = ctypes.c_size_t
    lib.litegrip_shm_state_size.restype = size_t
    lib.litegrip_shm_command_size.restype = size_t
    lib.litegrip_shm_header_size.restype = size_t
    lib.litegrip_shm_segment_size.restype = size_t

    lib.litegrip_shm_open.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)
    ]
    lib.litegrip_shm_open.restype = ctypes.c_int
    lib.litegrip_shm_close.argtypes = [ctypes.c_void_p]
    lib.litegrip_shm_close.restype = None
    lib.litegrip_shm_unlink.argtypes = [ctypes.c_char_p]
    lib.litegrip_shm_unlink.restype = ctypes.c_int

    lib.litegrip_shm_publish_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitegripState)
    ]
    lib.litegrip_shm_publish_state.restype = ctypes.c_int
    lib.litegrip_shm_read_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitegripState), ctypes.c_int
    ]
    lib.litegrip_shm_read_state.restype = ctypes.c_int

    lib.litegrip_shm_publish_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitegripCommand)
    ]
    lib.litegrip_shm_publish_command.restype = ctypes.c_int
    lib.litegrip_shm_read_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitegripCommand), ctypes.c_int
    ]
    lib.litegrip_shm_read_command.restype = ctypes.c_int

    lib.litegrip_shm_read_header.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitegripHeader)
    ]
    lib.litegrip_shm_read_header.restype = ctypes.c_int


def _verify_sizes(lib: ctypes.CDLL, path: Path) -> None:
    """Verify struct sizes at load time, so that a ctypes mirror / .so version
    mismatch does not cause silent misreads."""
    expected = (
        ("LitegripState", ctypes.sizeof(LitegripState),
         lib.litegrip_shm_state_size()),
        ("LitegripCommand", ctypes.sizeof(LitegripCommand),
         lib.litegrip_shm_command_size()),
        ("LitegripHeader", ctypes.sizeof(LitegripHeader),
         lib.litegrip_shm_header_size()),
    )
    for name, mirror, native in expected:
        if mirror != native:
            raise ShmError(
                f"{name} layout mismatch: the ctypes mirror is {mirror} bytes, "
                f"{path.name} reports {native} bytes — the two header files are "
                f"out of sync; run colcon build again before proceeding (do "
                f"not work around it by guessing the field order)"
            )


def _load_library() -> ctypes.CDLL:
    """Load liblitegrip_shm.so and verify the layout sizes while loading."""
    errors: List[str] = []
    for path in _candidate_lib_paths():
        if not path.exists():
            errors.append(f"{path} (does not exist)")
            continue
        try:
            lib = ctypes.CDLL(str(path), use_errno=True)
        except OSError as exc:  # pragma: no cover - environment-dependent
            errors.append(f"{path} ({exc})")
            continue
        _declare_signatures(lib)
        _verify_sizes(lib, path)
        return lib
    raise ShmError(
        "cannot locate liblitegrip_shm.so; run colcon build and source the "
        "install space first, or set the LITEGRIP_SHM_LIB environment "
        "variable. Tried:\n  " + "\n  ".join(errors)
    )


_lib: Optional[ctypes.CDLL] = None


def lib() -> ctypes.CDLL:
    """The lazily loaded .so (loaded only once per process)."""
    global _lib
    if _lib is None:
        _lib = _load_library()
    return _lib


def describe_layout() -> dict:
    """Export the layout mirrored by this module, for test/test_shm_layout.py
    to compare against the C-side probe."""
    return {
        "num_joints": NUM_JOINTS,
        "state_size": ctypes.sizeof(LitegripState),
        "command_size": ctypes.sizeof(LitegripCommand),
        "header_size": ctypes.sizeof(LitegripHeader),
        "segment_size": lib().litegrip_shm_segment_size(),
        "state_offsets": {name: getattr(LitegripState, name).offset
                          for name, _ in StateFields},
        "command_offsets": {name: getattr(LitegripCommand, name).offset
                            for name, _ in CommandFields},
        "header_offsets": {name: getattr(LitegripHeader, name).offset
                           for name, _ in HeaderFields},
    }


# ─────────────────────────── channel ───────────────────────────


class ShmChannel:
    """A user of one shared-memory segment (open/close + moving the two blocks
    of data in and out).

    There are two roles in practice:

    * daemon: ``ShmChannel(name, create=True)`` — it creates and owns the
      segment.
    * plugin (the C++ counterpart): ``ShmChannel(name, create=False)`` — the
      segment must already exist.

    ``close()`` is idempotent; ``unlink()`` is only called when the daemon
    exits (it owns the segment).
    """

    def __init__(self, name: str = DEFAULT_SHM_NAME, create: bool = False) -> None:
        self.name = name
        self._handle = ctypes.c_void_p()
        code = lib().litegrip_shm_open(
            name.encode("utf-8"), 1 if create else 0, ctypes.byref(self._handle))
        if code != SHM_OK:
            raise ShmError(
                f"failed to open shared-memory segment {name!r}: "
                f"{error_text(code)}"
                + (" (still failing with create=True; most likely permissions "
                   "or /dev/shm is full)"
                   if create else
                   " (the segment does not exist yet — is the daemon "
                   "running?)"),
                code)

    # ── closing ──
    def close(self) -> None:
        if self._handle:
            lib().litegrip_shm_close(self._handle)
            self._handle = ctypes.c_void_p()

    def unlink(self) -> None:
        """Delete the segment (call it after all users have exited)."""
        code = lib().litegrip_shm_unlink(self.name.encode("utf-8"))
        if code != SHM_OK:
            raise ShmError(
                f"failed to unlink shared-memory segment {self.name!r}: "
                f"{error_text(code)}", code)

    def __enter__(self) -> "ShmChannel":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # ── state (daemon writes / plugin reads) ──
    def publish_state(self, state: LitegripState) -> None:
        code = lib().litegrip_shm_publish_state(self._handle, ctypes.byref(state))
        if code != SHM_OK:
            raise ShmError(f"failed to publish state: {error_text(code)}", code)

    def read_state(self, retries: int = 8) -> Optional[LitegripState]:
        """Read one state frame. Returning ``None`` means a torn read (retries
        exhausted) — the caller should keep the previous frame."""
        out = LitegripState()
        code = lib().litegrip_shm_read_state(self._handle, ctypes.byref(out), retries)
        if code == SHM_OK:
            return out
        if code == SHM_TORN:
            return None
        raise ShmError(f"failed to read state: {error_text(code)}", code)

    # ── command (plugin writes / daemon reads) ──
    def publish_command(self, command: LitegripCommand) -> None:
        code = lib().litegrip_shm_publish_command(self._handle,
                                                 ctypes.byref(command))
        if code != SHM_OK:
            raise ShmError(f"failed to publish command: {error_text(code)}",
                           code)

    def read_command(self, retries: int = 8) -> Optional[LitegripCommand]:
        out = LitegripCommand()
        code = lib().litegrip_shm_read_command(self._handle, ctypes.byref(out),
                                              retries)
        if code == SHM_OK:
            return out
        if code == SHM_TORN:
            return None
        raise ShmError(f"failed to read command: {error_text(code)}", code)

    def read_header(self) -> LitegripHeader:
        out = LitegripHeader()
        code = lib().litegrip_shm_read_header(self._handle, ctypes.byref(out))
        if code != SHM_OK:
            raise ShmError(f"failed to read header: {error_text(code)}", code)
        return out
