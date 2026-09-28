#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litegrip_hw_daemon — hardware daemon for the LiteGrip gripper.

Where it sits in the chain
--------------------------
    controller_manager ── ros2_control controllers (JTC / forward)
          ⇅  joint interfaces: command position only; state position/velocity/effort
    LitegripSystem (C++ plugin; litearm's JTC or forward controller drives it)
          ⇅  seqlock shared memory (the contract between this file and litegrip_shm.h)
    litegrip_hw_daemon (this file, a **separate process**)
          ⇅  LiteGrip SDK
    can0 (= the STM32 board's if0 gs_usb bridge) → DM4310

**This process has no ROS dependency whatsoever** (it does not import rclpy): it
only reads parameters, reads shared memory, and calls the SDK. This mirrors the
arm's daemon, and the benefit is that the real-time side has no Python in it at all.

What it is responsible for — and why it all lives here
------------------------------------------------------
1. **Exclusive hardware ownership**: opens can0 and sends DM MIT frames. No
   second process may touch this bus at the same time; the adapter turns that
   into a hard constraint with a construction-time single-instance guard, rather
   than relying on documentation to remind people.
2. **Safety gate**: every command goes through :func:`safety_gate.check_command`
   (red line / finiteness / ceilings). A rejection means **the whole command is
   discarded and not one frame is sent** — never silently clamped.
3. **Trajectory rate limiting**: the "target position" becomes a rate-limited
   trajectory (advancing at most ``max_velocity_rad_s × elapsed`` per cycle), and
   what is sent out is the trajectory's **current point**, not its endpoint.
   ★ The dq field in the MIT frame is **always 0** — rate is not expressed via dq.
4. **Torque budget allocation**: the N·m given by a command is not an "output
   torque" but a **budget**: it is allocated to kp and kd so that
   ``kp·e_b + kd·v_b ≤ budget``, where e_b/v_b are **worst-case bounds** (not
   the current sample). See :mod:`.sdk_adapter` for the rationale.
5. **Fault accounting**: distinguishes **transient** (a single command was
   rejected; cleared by the next valid command) from **persistent**
   (motor/communication/internal; cleared only once sampling observes health
   again). Masking a hardware fault with one ordinary command is the most
   dangerous class of error in a bridge like this.
6. **Latched safe stop**: once latched it **sends no further motion frames** and
   **does not self-recover** — a human must investigate and restart this process.

⚠ Exiting disables the gripper (important operational constraint)
-----------------------------------------------------------------
When this process exits it runs the adapter's teardown: ``disable()`` +
``disconnect()``. **Disabling means the gripper goes limp** (both fingers can be
pushed by an external force). Therefore:
  * **Do not** stop the daemon while it is holding something; if you need to keep
    gripping, leave the process running — it holds position automatically once
    the command frame goes stale (see ``--command-timeout-s``).
  * A true power-cut emergency stop can only be provided by the hardware
    circuit: if this process hangs, the CAN link drops, or Python raises, none of
    the software-level protections take effect.

Usage
-----
    litegrip_hw_daemon --hw-config <config/litegrip_hw.yaml> [--dry-run]

Precedence: **command line > --hw-config file > code defaults**.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import shm_bridge
from .driver_adapter import (
    AdapterLatched,
    MockAdapter,
    Sample,
    SdkUnavailable,
    read_limits_from_sdk,
)
from .safety_gate import (
    MAX_COMMAND_VELOCITY_CEILING_RAD_S,
    TORQUE_LIMIT_CEILING_NM,
    FaultCode,
    GateLimits,
    check_command,
)

# ── Defaults (code defaults = the lowest-priority tier) ──────────────────────
DEFAULT_RATE_HZ = 50.0
DEFAULT_COMMAND_TIMEOUT_S = 0.2
DEFAULT_FEEDBACK_TIMEOUT_S = 0.5
DEFAULT_TEMPERATURE_LIMIT_C = 80
DEFAULT_MAX_VELOCITY_RAD_S = MAX_COMMAND_VELOCITY_CEILING_RAD_S
DEFAULT_TORQUE_LIMIT_NM = TORQUE_LIMIT_CEILING_NM
DEFAULT_SAFETY_BASELINE = "3.5"
DEFAULT_CHANNEL = "can0"
DEFAULT_CAN_ID = 0x08
DEFAULT_MST_ID = 0x18
DEFAULT_KP = 20.0
DEFAULT_KD = 0.5
DEFAULT_EXIT_HOLD_S = 2.0

#: CLI overrides with a ``--`` prefix → (config key, type).
#: One source of truth: listed here once, shared by ``load_hw_config``'s
#: unknown-key check and by the CLI overrides.
_CLI_OVERRIDES = (
    ("channel", "channel", str),
    ("can-id", "can_id", int),
    ("mst-id", "mst_id", int),
    ("rate-hz", "control_rate_hz", float),
    ("command-timeout-s", "command_timeout_s", float),
    ("feedback-timeout-s", "feedback_timeout_s", float),
    ("temperature-limit-c", "temperature_limit_c", int),
    ("max-velocity-rad-s", "max_velocity_rad_s", float),
    ("torque-limit-nm", "torque_limit_nm", float),
    ("safety-baseline", "safety_baseline", str),
    ("max-position-error-rad", "max_position_error_rad", float),
    ("max-feedback-velocity-rad-s", "max_feedback_velocity_rad_s", float),
    ("kp", "kp", float),
    ("kd", "kd", float),
    ("sdk-path", "sdk_path", str),
)

#: Keys allowed in the config file but **not** handled by the _CLI_OVERRIDES
#: loop: the dry_run / hardware_enable switches are merged separately in
#: resolve_config (None distinguishes "not given" from "explicitly given"), and
#: shm_name is managed on its own by --shm-name.
_ALLOWED_KEYS = ({key for _flag, key, _type in _CLI_OVERRIDES}
                 | {"dry_run", "hardware_enable", "shm_name"})


def load_hw_config(path: str) -> Dict[str, Any]:
    """Read ``--hw-config`` (plain YAML, not a ROS parameter file).

    **An unknown key raises immediately** instead of being silently ignored: a
    typo in the config file that keeps an old value in force is the hardest class
    of problem to track down ("I set it and nothing happens"). Same rule as the
    arm's daemon.
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment-dependent
        raise RuntimeError(
            "Reading --hw-config requires PyYAML (rosdep: python3-yaml); "
            "or use command-line arguments instead"
        ) from exc

    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    if not isinstance(raw, dict):
        raise RuntimeError(
            f"{path}: the top level must be a mapping, got {type(raw).__name__}")

    # A single "gripper:" wrapper is allowed (that is the shape this package's
    # config/litegrip_hw.yaml uses); a flat mapping is allowed too. Both are
    # accepted, but **only one level**, to avoid the ambiguity of which level wins.
    if "gripper" in raw:
        if len(raw) != 1:
            raise RuntimeError(
                f"{path} has both a 'gripper:' wrapper and top-level keys "
                f"{sorted(set(raw) - {'gripper'})} — only one shape is allowed"
            )
        raw = raw["gripper"]
        if not isinstance(raw, dict):
            raise RuntimeError(f"{path}: 'gripper:' must be a mapping")

    unknown = sorted(set(raw) - _ALLOWED_KEYS)
    if unknown:
        raise RuntimeError(
            f"{path} has unknown keys {unknown}; allowed keys: "
            f"{sorted(_ALLOWED_KEYS)}"
        )
    return raw


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="LiteGrip gripper hardware daemon "
                    "(owns can0 exclusively, talks to ros2_control over shared memory)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--shm-name", default=shm_bridge.DEFAULT_SHM_NAME,
                        help="shared memory segment name; must match the "
                             "plugin's shm_name parameter")

    # ── Two switches (same semantics as the old driver) ──
    parser.add_argument("--dry-run", action=argparse.BooleanOptionalAction,
                        default=None,
                        help="true = use the mock adapter (no hardware access). "
                             "Defaults to false")
    parser.add_argument("--hardware-enable", action=argparse.BooleanOptionalAction,
                        default=None,
                        help="hardware master switch: **must be given together "
                             "with --no-dry-run** to connect to hardware")

    parser.add_argument("--hw-config", default=None,
                        help="optional YAML parameter file "
                             "(see config/litegrip_hw.yaml)")

    # ── Other overrides: default None = "not given", filled in by config/defaults ──
    parser.add_argument("--channel", default=None)
    parser.add_argument("--can-id", type=int, default=None)
    parser.add_argument("--mst-id", type=int, default=None)
    parser.add_argument("--rate-hz", type=float, default=None)
    parser.add_argument("--command-timeout-s", type=float, default=None)
    parser.add_argument("--feedback-timeout-s", type=float, default=None)
    parser.add_argument("--temperature-limit-c", type=int, default=None)
    parser.add_argument("--max-velocity-rad-s", type=float, default=None,
                        help="command trajectory advance rate ceiling "
                             "(rate is a parameter, not a command field)")
    parser.add_argument("--torque-limit-nm", type=float, default=None,
                        help="torque budget (budget is a parameter, not a command "
                             "field)")
    parser.add_argument("--safety-baseline", default=None)
    parser.add_argument("--max-position-error-rad", type=float, default=None)
    parser.add_argument("--max-feedback-velocity-rad-s", type=float, default=None)
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--kd", type=float, default=None)
    parser.add_argument("--sdk-path", default=None)
    parser.add_argument("--exit-hold-s", type=float, default=DEFAULT_EXIT_HOLD_S,
                        help="seconds to keep publishing state / holding position "
                             "before exiting (0 = exit immediately)")
    return parser.parse_args(argv)


def resolve_config(args: argparse.Namespace) -> Dict[str, Any]:
    """Merge three layers: code defaults < --hw-config file < command line.

    ``--dry-run`` and ``--hardware-enable`` use ``None`` to mean "not given",
    which keeps "not given" separate from "explicitly set to the default value" —
    the former follows the config/defaults, the latter overrides the config.
    """
    config: Dict[str, Any] = {
        "channel": DEFAULT_CHANNEL,
        "can_id": DEFAULT_CAN_ID,
        "mst_id": DEFAULT_MST_ID,
        "control_rate_hz": DEFAULT_RATE_HZ,
        "command_timeout_s": DEFAULT_COMMAND_TIMEOUT_S,
        "feedback_timeout_s": DEFAULT_FEEDBACK_TIMEOUT_S,
        "temperature_limit_c": DEFAULT_TEMPERATURE_LIMIT_C,
        "max_velocity_rad_s": DEFAULT_MAX_VELOCITY_RAD_S,
        "torque_limit_nm": DEFAULT_TORQUE_LIMIT_NM,
        "safety_baseline": DEFAULT_SAFETY_BASELINE,
        # -1 = "not given" (sentinel); neither worst-case bound gets an invented
        # default, see the config comments
        "max_position_error_rad": -1.0,
        "max_feedback_velocity_rad_s": -1.0,
        "kp": DEFAULT_KP,
        "kd": DEFAULT_KD,
        "dry_run": True,           # ★ safe default: no hardware access
        "hardware_enable": False,  # ★ hardware master switch defaults off
        "sdk_path": "",
    }
    if args.hw_config:
        config.update(load_hw_config(args.hw_config))
    for flag, key, _type in _CLI_OVERRIDES:
        value = getattr(args, flag.replace("-", "_"))
        if value is not None:
            config[key] = value
    if args.dry_run is not None:
        config["dry_run"] = bool(args.dry_run)
    if args.hardware_enable is not None:
        config["hardware_enable"] = bool(args.hardware_enable)
    # The segment name comes from the command line only: it must match the
    # plugin's shm_name parameter verbatim, and the plugin's value comes from the
    # URDF — letting the config file change it too would only add one more place
    # where the two can disagree.
    config["shm_name"] = args.shm_name
    config["exit_hold_s"] = args.exit_hold_s
    return config


class GripperDaemon:
    """The loop that feeds shared-memory commands to the adapter and writes
    samples back into shared memory."""

    def __init__(self, config: Dict[str, Any], log=print) -> None:
        self._config = config
        self._log = log
        self._closing = False
        self._channel: Optional[shm_bridge.ShmChannel] = None
        self._adapter: Optional[Any] = None
        self._limits: Optional[GateLimits] = None

        # bookkeeping
        self._cycle = 0
        self._applied_cycle = -1
        self._last_command_stamp = 0.0
        self._command_fault = 0          # transient: next valid command clears it
        self._persistent_fault = 0       # persistent: only a healthy sample clears it
        self._latched = False
        self._last_error = shm_bridge.DAEMON_CONNECTING
        self._last_state: Optional[Sample] = None
        self._estop_seen = False
        #: Log throttle timestamp for rejected commands (a rejection must be
        #: visible, but it must not flood the log).
        self._last_reject_log_s = 0.0

    # ── Startup phase ─────────────────────────────────────────────────────
    def _setup_sdk_path(self) -> None:
        sdk_path = str(self._config.get("sdk_path") or "")
        if sdk_path and sdk_path not in sys.path:
            sys.path.insert(0, sdk_path)

    def _resolve_limits(self) -> float:
        """Get the torque ceiling. **It can only be tightened, never loosened.**"""
        requested = float(self._config["torque_limit_nm"])
        if not _is_finite(requested) or requested <= 0.0:
            self._log(f"[ERROR] torque_limit_nm={requested} is not a positive "
                      f"finite number — falling back to the hard ceiling "
                      f"{TORQUE_LIMIT_CEILING_NM} N·m")
            return TORQUE_LIMIT_CEILING_NM
        if requested > TORQUE_LIMIT_CEILING_NM:
            self._log(f"[ERROR] torque_limit_nm={requested} exceeds the hard "
                      f"ceiling {TORQUE_LIMIT_CEILING_NM} N·m — value rejected, "
                      f"using the hard ceiling instead. The torque ceiling can "
                      f"only be tightened, never loosened.")
            return TORQUE_LIMIT_CEILING_NM
        return requested

    def _resolve_velocity_limit(self) -> float:
        """Get the command trajectory rate ceiling.
        **It can only be tightened, never loosened.**"""
        requested = float(self._config["max_velocity_rad_s"])
        if not _is_finite(requested) or requested <= 0.0:
            self._log(f"[ERROR] max_velocity_rad_s={requested} is not a positive "
                      f"finite number — falling back to the hard ceiling "
                      f"{MAX_COMMAND_VELOCITY_CEILING_RAD_S} rad/s. "
                      f"★ 0 does not mean 'slowest', it means 'the trajectory "
                      f"never reaches the target'.")
            return MAX_COMMAND_VELOCITY_CEILING_RAD_S
        if requested > MAX_COMMAND_VELOCITY_CEILING_RAD_S:
            self._log(f"[ERROR] max_velocity_rad_s={requested} exceeds the hard "
                      f"ceiling {MAX_COMMAND_VELOCITY_CEILING_RAD_S} rad/s — value "
                      f"rejected, using the hard ceiling instead. It can only be "
                      f"tightened, never loosened.")
            return MAX_COMMAND_VELOCITY_CEILING_RAD_S
        return requested

    def _build_limits(self) -> Optional[GateLimits]:
        """Fetch the red line from the SDK. Returns ``None`` if it cannot be
        fetched ⇒ the gate then rejects every command by default."""
        self._setup_sdk_path()
        torque = self._resolve_limits()
        velocity = self._resolve_velocity_limit()
        baseline = str(self._config["safety_baseline"])
        try:
            limits = read_limits_from_sdk(
                torque, safety_baseline=baseline, max_velocity_rad_s=velocity)
        except SdkUnavailable as exc:
            self._log(f"[ERROR] cannot fetch the LiteGrip SDK red line: {exc} — "
                      f"state will still be published, but **every command will be "
                      f"rejected** (fail-closed)")
            return None
        self._log(f"[INFO] red line loaded from the SDK: position "
                  f"[{limits.red_min_rad}, {limits.red_max_rad}] rad, torque "
                  f"ceiling {limits.torque_limit_nm} N·m, command trajectory "
                  f"rate ceiling {limits.max_velocity_rad_s} rad/s "
                  f"(safety baseline {baseline})")
        return limits

    def _build_adapter(self):
        """Build the execution layer. Three paths, **no fourth one**:
        ① dry_run=True → MockAdapter; ② hardware_enable missing → None (rejects
        every command, never pretends to be working); ③ both switches on → the
        hardware adapter (owns can0 exclusively).
        """
        if self._config["dry_run"]:
            return MockAdapter(self._limits)

        if not self._config["hardware_enable"]:
            self._log("[ERROR] dry_run=False but hardware_enable=False — "
                      "**the hardware path will not start**. Enabling hardware "
                      "requires both switches to hold at once. This process will "
                      "not open can0 and will not send any CAN frame; every "
                      "command will be rejected.")
            return None

        if self._limits is None:
            self._log("[ERROR] hardware_enable=True but the safety limits could "
                      "not be fetched — **refusing to set up the hardware path**. "
                      "No red line means no motor drive.")
            return None

        # ★ Local import: the dry_run path never reaches here, so the sdk_adapter
        #   module (and the litegrip it imports indirectly) never enters
        #   sys.modules at all under dry_run.
        from .sdk_adapter import HardwareConfig, SdkHardwareAdapter  # noqa: PLC0415

        try:
            hw_config = HardwareConfig(
                channel=str(self._config["channel"]),
                can_id=int(self._config["can_id"]),
                mst_id=int(self._config["mst_id"]),
                canfd_mode=False,
                kp=float(self._config["kp"]),
                kd=float(self._config["kd"]),
                control_rate_hz=float(self._config["control_rate_hz"]),
                feedback_timeout_s=float(self._config["feedback_timeout_s"]),
                temperature_limit_c=int(self._config["temperature_limit_c"]),
                max_position_error_rad=_optional(self._config,
                                                 "max_position_error_rad"),
                max_feedback_velocity_rad_s=_optional(
                    self._config, "max_feedback_velocity_rad_s"),
            )
            adapter = SdkHardwareAdapter(self._limits, hw_config)
            adapter.start()
        except Exception as exc:  # noqa: BLE001 - must surface as visible state

            self._log(f"[ERROR] failed to start the hardware adapter: {exc!r} — "
                      f"commands will be rejected")
            return None

        self._log(f"[WARN] ★ hardware path started: this process now **owns "
                  f"{hw_config.channel} exclusively** and drives the motor. Make "
                  f"sure there is no second process on the bus and that the "
                  f"hardware emergency stop is available.")
        if _optional(self._config, "max_feedback_velocity_rad_s") is None:
            self._log("[ERROR] max_feedback_velocity_rad_s **is not configured** — "
                      "the worst-case torque budget is missing its velocity term, "
                      "so **this layer refuses to send any motion frame** "
                      "(fail-closed). This is not a fault but missing "
                      "configuration: calibrate this value in the parameter file "
                      "and restart. ★ No default will be invented for you.")
        return adapter

    # ── Loop ─────────────────────────────────────────────────────────────
    def _handle_command(self, command: Optional[shm_bridge.LitegripCommand],
                        now: float) -> None:
        """Command frame read → gate → written into the adapter's target slot.

        ★ The same iron rule as the old driver: **a rejection means zero frames**.
        The gate returns before ``adapter.set_target``, so the execution layer's
        target slot is never written at all and the background thread has nothing
        to send — a structural guarantee, not something that relies on the
        adapter's own discipline.
        """
        if command is None:
            return
        # ★★ cycle_count distinguishes "there has never been a command" from
        #    "the command's value happens to be 0". A freshly created shared
        #    memory segment is **all zeros**, and 0.0 rad falls **outside** the
        #    red line [-1.24, -0.01] — without that distinction the daemon would
        #    report this "command that never existed" as an out-of-range command
        #    and log BAD_COMMAND the moment it starts (measured: fault_code=1 on
        #    the very first frame, which looks like a hardware/config fault but is
        #    really just zero initialization). The plugin's cycle_count starts at
        #    1 and increases, so <= 0 means "never published".
        if command.cycle_count <= 0.0:
            # No command has ever been published: report CONNECTING ("it is up,
            # waiting for a command") rather than BAD_COMMAND — the latter sends
            # people off to check the red line / configuration.
            self._last_error = shm_bridge.DAEMON_CONNECTING
            return
        self._last_command_stamp = command.stamp_s

        if command.estop != 0.0:
            if not self._estop_seen:
                self._estop_seen = True
                self._log("[WARN] soft emergency-stop request received")
            self._last_error = shm_bridge.DAEMON_HOLDING_ESTOP
            return
        self._estop_seen = False

        adapter = self._adapter
        if adapter is None:
            # Having no execution layer is a **persistent** state (decided by the
            # configuration at startup, it will not fix itself), so it goes into
            # the persistent register and does not vanish with the next command.
            self._persistent_fault = int(FaultCode.INTERNAL)
            self._last_error = (shm_bridge.DAEMON_DISABLED
                                if not self._config["dry_run"]
                                else shm_bridge.DAEMON_CONNECTING)
            return

        if self._latched:
            self._last_error = shm_bridge.DAEMON_LATCHED_SAFE_STOP
            return

        ages = now - self._last_command_stamp
        if self._last_command_stamp > 0.0 and \
                ages > float(self._config["command_timeout_s"]):
            # Command is stale: do not send a new target; let the adapter
            # stop/hold along its own trajectory.
            self._last_error = shm_bridge.DAEMON_HOLDING_STALE_COMMAND
            return

        target = float(command.position[0])
        decision = check_command(
            target,
            float(self._config["torque_limit_nm"]),
            float(self._config["max_velocity_rad_s"]),
            self._limits,
        )
        if not decision.accepted:
            # Touch only the transient one — a bad command can neither create a
            # persistent fault nor shrink or mask an existing one.
            self._command_fault = int(decision.fault_code)
            self._last_error = shm_bridge.DAEMON_HOLDING_BAD_COMMAND
            # ★ A rejection must be **visible**. If we only wrote last_error, all
            #   you would see in the field is fault_code=1 with no idea which
            #   command or which check tripped — and "a command that was sent gets
            #   silently dropped" is exactly the phenomenon that is hardest to
            #   track down in a bridge like this. Throttled to 5s so it does not
            #   flood the log.
            now_log = time.monotonic()
            if now_log - self._last_reject_log_s > 5.0:
                self._last_reject_log_s = now_log
                self._log(f"[WARN] command rejected "
                          f"({decision.reason.value}): {decision.detail} — "
                          f"target {target!r} rad, "
                          f"budget {self._config['torque_limit_nm']} N·m, "
                          f"rate ceiling {self._config['max_velocity_rad_s']} "
                          f"rad/s; **not a single motion frame will be sent** "
                          f"for this one")
            return

        try:
            adapter.set_target(decision.target_position_rad,
                               decision.torque_limit_nm,
                               decision.max_velocity_rad_s)
        except AdapterLatched as exc:
            # ★★ The execution layer stopped **by design** — this is not a
            #    contract violation, it is a correct safety stop. It must come
            #    before the generic except, otherwise a correct stop gets reported
            #    as an internal bug.
            self._latched = True
            self._persistent_fault = int(exc.fault_code)
            self._last_error = shm_bridge.DAEMON_LATCHED_SAFE_STOP
            self._log(f"[ERROR] the execution layer has latched a safe stop "
                      f"(fault_code={int(exc.fault_code)}) — {exc.detail}. "
                      f"This process will not recover on its own; a human must "
                      f"investigate and restart it.")
            return
        except Exception as exc:  # noqa: BLE001 - exceptions must not escape
            # The gate let it through but the execution layer rejected it ⇒
            # a **contract violation** (a programming error), not an ordinary
            # out-of-range value.
            self._persistent_fault = int(FaultCode.INTERNAL)
            self._last_error = shm_bridge.DAEMON_HOLDING_BAD_COMMAND
            self._log(f"[ERROR] the execution layer rejected a command that had "
                      f"already passed the gate: {exc!r} — this is an internal "
                      f"inconsistency; check whether safety_gate and the execution "
                      f"layer take their limits from the same source.")
            return

        # ★ Clear the transient one only. The persistent fault register is **not
        #   touched by a single byte** here — this line is the rule "a persistent
        #   fault must not clear merely because the next command is valid".
        self._command_fault = 0
        self._applied_cycle = self._cycle
        self._last_error = shm_bridge.DAEMON_OK

    def _assemble_state(self, now: float) -> shm_bridge.LitegripState:
        """Assemble the most recent sample plus the health flags into one state
        frame."""
        state = shm_bridge.LitegripState()
        sample = self._last_state

        if sample is None:
            # No usable data source — do not pass zeros off as real values: the
            # connected/communication semantics are expressed by the two fields
            # below, while position and velocity stay 0 and have to be read
            # together with fault_code.
            state.position[0] = 0.0
            state.velocity[0] = 0.0
            state.effort[0] = 0.0
            state.enabled = 0.0
        else:
            state.position[0] = sample.position_rad
            state.velocity[0] = sample.velocity_rad_s
            state.effort[0] = sample.torque_nm
            state.temperature_mos[0] = sample.temperature_mos_c
            state.temperature_coil[0] = sample.temperature_coil_c
            state.error_code[0] = float(sample.error_code)
            state.enabled = 1.0 if sample.enabled else 0.0
            state.stopped = 1.0 if sample.stopped else 0.0
            # ── Persistent fault register: sampling is the only observation ──
            #    Clearing is allowed only once health is observed — that is its
            #    **only** clearing path.
            self._persistent_fault = int(sample.fault_code)

        # Precedence: **persistent faults** outrank **transient command-rejection
        # events**. Once the hardware reports an error it must be seen — a stale
        # "command was rejected" must not mask it.
        fault_code = (self._persistent_fault if self._persistent_fault != 0
                      else self._command_fault)
        state.fault_code[0] = float(fault_code)
        state.faulted = 1.0 if fault_code != 0 else 0.0

        state.feedback_age_s[0] = -1.0 if sample is None else 0.0
        state.feedback_received[0] = float(self._cycle)
        state.stamp_s = now
        state.heartbeat_s = now
        state.connected = 1.0 if self._adapter is not None else 0.0
        state.dry_run = 1.0 if self._config["dry_run"] else 0.0
        state.latched = 1.0 if self._latched else 0.0
        state.cycle_count = float(self._cycle)
        state.applied_command_cycle = float(self._applied_cycle)
        state.command_age_s = (now - self._last_command_stamp
                               if self._last_command_stamp > 0.0 else -1.0)
        state.last_error = float(self._last_error)
        return state

    def spawn(self) -> bool:
        """Set the whole thing up (red line → adapter → shared memory).
        Returns False on failure."""
        self._limits = self._build_limits()
        self._adapter = self._build_adapter()
        try:
            # ★ create=True: the segment belongs to this process (the plugin
            # attaches with create=False).
            self._channel = shm_bridge.ShmChannel(
                self._config["shm_name"], create=True)
        except shm_bridge.ShmError as exc:
            self._log(f"[ERROR] failed to open/create the shared memory segment: "
                      f"{exc}")
            return False

        mode = "dry-run (mock data, no hardware)" if self._config["dry_run"] else (
            "hardware" if self._adapter is not None
            else "**refusing to drive** (see the ERROR above)")
        self._log(f"[INFO] daemon ready: mode={mode}, "
                  f"shm={self._config['shm_name']}, "
                  f"period={1.0 / float(self._config['control_rate_hz']) * 1e3:.0f}ms, "
                  f"can={self._config['channel']}")
        return True

    def spin(self) -> None:
        period = 1.0 / float(self._config["control_rate_hz"])
        while not self._closing:
            start = time.monotonic()
            self._cycle += 1

            command = self._channel.read_command()
            if command is not None:
                self._handle_command(command, start)

            if self._adapter is not None:
                try:
                    sample = self._adapter.snapshot()
                    self._last_state = sample
                except Exception as exc:  # noqa: BLE001 - must surface as visible state

                    self._persistent_fault = int(FaultCode.NO_FEEDBACK)
                    self._last_error = shm_bridge.DAEMON_HOLDING_FEEDBACK_STALE
                    self._log(f"[ERROR] sampling failed: {exc!r}")

            try:
                self._channel.publish_state(self._assemble_state(start))
            except shm_bridge.ShmError as exc:
                self._log(f"[ERROR] failed to publish state: {exc}")
                break

            remaining = period - (time.monotonic() - start)
            if remaining > 0.0:
                time.sleep(remaining)

    def shutdown(self) -> None:
        """Tear down: hold position for a few seconds first, then close the
        adapter and remove the segment.

        ⚠ Closing the adapter **disables the gripper** (see the module
        docstring) — do not stop this process while it is holding something.
        """
        hold_s = float(self._config.get("exit_hold_s") or 0.0)
        if hold_s > 0.0 and self._channel is not None:
            self._last_error = shm_bridge.DAEMON_SHUTTING_DOWN
            deadline = time.monotonic() + hold_s
            while time.monotonic() < deadline:
                try:
                    self._channel.publish_state(
                        self._assemble_state(time.monotonic()))
                except shm_bridge.ShmError:
                    break
                time.sleep(0.05)

        if self._adapter is not None:
            closer = getattr(self._adapter, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception as exc:  # noqa: BLE001 - must not crash the process

                    self._log(f"[ERROR] failed to close the execution layer: "
                              f"{exc!r} — whether the hardware is disabled is "
                              f"**unconfirmed**")

        if self._channel is not None:
            try:
                self._channel.unlink()
            except shm_bridge.ShmError as exc:
                self._log(f"[WARN] failed to unlink the shared memory segment: "
                          f"{exc}")
            self._channel.close()
            self._channel = None

        self._log(f"[INFO] daemon teardown complete: {self._cycle} cycles total, "
                  f"last sample: {self._last_state!r}")


def _is_finite(value: float) -> bool:
    import math

    return math.isfinite(value)


def _optional(config: Dict[str, Any], key: str) -> Optional[float]:
    """Turn the sentinel (< 0) into ``None``; the **only** place allowed to do so."""
    value = float(config[key])
    return None if value < 0.0 else value


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        config = resolve_config(args)
    except (RuntimeError, OSError) as exc:
        print(f"[ERROR] argument/configuration error: {exc}", file=sys.stderr)
        return 2

    log = lambda text: print(f"{time.strftime('%H:%M:%S')} {text}", flush=True)
    daemon = GripperDaemon(config, log=log)

    def _on_signal(signum, _frame):  # noqa: ANN001
        log(f"[INFO] received signal {signum}, preparing to exit")
        daemon._closing = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    if not daemon.spawn():
        return 1
    try:
        daemon.spin()
    except KeyboardInterrupt:  # belt and braces: clean exit even without handlers

        pass
    finally:
        daemon.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
