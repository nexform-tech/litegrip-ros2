"""sdk_adapter — the real LiteGrip SDK adapter layer.

This module is the **only** code in this package that touches hardware.
``driver_adapter`` only handles "reading the limits from the SDK, read-only"
and dry_run's fake feedback; the real gripper's lifecycle (``connect`` /
``enable`` / sending frames / reading state / shutdown) is all gathered here.

Why a separate layer is mandatory
---------------------------------
In a ROS 2 single-threaded executor, subscription callbacks and timer callbacks
**share one and the same thread**. Calling any blocking motion interface from a
callback stalls the whole node until the motion ends — during which it cannot
receive new commands, publish status, or respond to an emergency stop. So this
layer puts the hardware on **its own background thread**, and the callbacks do
only two things: validate + write a target slot.


Requirement 6 verdict: the SDK **does** have a safe non-blocking position
control interface
--------------------------------------------------------------------------
Conclusion after checking the source line by line (not copied from the report;
the report is only an index):

========================================  ====================================
Interface                                 Nature
========================================  ====================================
``LiteGrip.send_mit_frame(q,kp,kd,dq,tau)`` ★ **Non-blocking**: sends **one**
``gripper.py:725`` → ``LiteGripCAN``        frame and returns (no ``duration``
``.send_mit_motion()`` ``can_bus.py:1078``  loop, no ``wait_for_ready``, no sleep)
``→ _guard_motion()`` ``can_bus.py:201``    ★ **Full red-line check**: both the
``→ _control_mit_gated()`` ``:324``         measured and target position must be
``→ control_mit()`` ``controller.py:239``   inside the red lines; a violation raises
``→ transport.send()`` ``transport.py:220`` ``LimitViolation`` / ``SafetyFault``,
                                            **never silently clamped**
========================================  ====================================

Therefore there is **no need** to report an API gap, and **no need** to fall
back to a blocking interface. But three rules must hold together; drop one and
this verdict does not stand:

1. it may only be called from the **background thread** (requirement 8);
2. **never** call the blocking interfaces of §5.1 (:data:`FORBIDDEN_MOTION_APIS`);
3. a frame may only be sent after **real feedback** has been obtained —
   ``_guard_motion`` requires ``motor.rx_count > 0``, so motion frames are
   rejected outright as long as no feedback frame has ever arrived. And DM
   motors use **poll-style** feedback (no outgoing frame, no reply frame), so
   keepalive frames must keep flowing even while "not driving". See
   :meth:`SdkHardwareAdapter._send_keepalive`.


Thread model
------------
::

    ROS subscription callback ──set_target()─▶ [target slot]  ◀──read/cycle── background
    ROS timer callback        ──snapshot()───▶ [sample cache] ◀──write/cycle─ background

Neither arrow **touches the SDK**: the callback side only takes a lock once and
reads or writes one immutable object, so its cost is independent of the
hardware. Every SDK call happens on the background thread. Samples are immutable
:class:`~litegrip_ros2_control.driver_adapter.Sample` objects
(``frozen=True``), so a snapshot is **always** self-consistent — there is no
"read half of it, another thread rewrote the other half".


Lifecycle (requirement 10)
--------------------------
Everything is gathered in the background thread's
:meth:`SdkHardwareAdapter._run`, in strict order::

    import SDK → construct LiteGrip
      → probe refresh_status interface [missing ⇒ INTERNAL, refuse to power up]
      → connect() [exactly once]
      → disable() probe frame [give the driver a reason to answer]
      → wait for one **fresh** feedback frame [none ⇒ NO_FEEDBACK, do not enable]
      → get_state(wait=False) read cache → **check** → enable() only if it passes
        → control loop → finally: disable() → disconnect()

End of every control-loop cycle ("asking" and "collecting" must be separate)::

    has target → send_mit_frame(position frame)   → reply read next cycle by poll()
    no target  → stop() [zero-torque keepalive]   → SDK reads that reply internally
                 → refresh_status() [0xCC refresh] → reply read next cycle by poll()
                                                       ↑ without this line, when idle
                                                         poll() never sees a new frame
                                                         ⇒ fake NO_FEEDBACK after 0.5 s

★ ``connect()`` has exactly one call site, :meth:`_connect_once`; ``enable()``
has exactly one, :meth:`_enable_once`. **Never enable unless the initial state
check passes** — that is "powering a motor without knowing where the mechanism
is", the one mistake this layer must never make.

★★ Why the extra "ask first, then wait" steps in between: DM4310 feedback is
**poll-style** — no outgoing frame, no reply frame; and when it has never
received a frame, ``get_state`` returns ``position_rad=0.0``, the **cached
initial value** (not a measurement), with a timestamp that is still "just now",
so it looks perfectly normal. See :meth:`_await_fresh_frame` and
:meth:`_check_initial_state`.


★ Safety stop is **latched**
----------------------------
Once a safety stop is entered, the background thread **leaves the loop**,
followed by ``disable()`` + ``disconnect()``, and this adapter layer **never
recovers on its own**. To move the gripper again you can only restart the node.

The design follows requirement 13: "a persistent fault must not be cleared
automatically by the next legal command". A stronger guarantee than "clear it"
is "never recover at all" — there is no automatic recovery path in the code, so
a missing clear somewhere is impossible. The price is manual intervention, and
that is deliberate:

* **Transient** (a single command turned away by the gate) →
  ``FaultCode.COMMAND_REJECTED``, owned by ``gripper_node``'s command path and
  cleared by the next legal command;
* **Persistent** (anything this adapter layer observes) →
  ``FaultCode.HARDWARE_SAFE_STOP`` and friends, written by the background
  thread and **read-only for the command path**, and this layer never recovers
  automatically.

★ Once latched, :meth:`SdkHardwareAdapter.set_target` **raises
:class:`~litegrip_ros2_control.driver_adapter.AdapterLatched`** and no longer
accepts a target. That is not pedantry: once the loop has exited the target slot
has no consumer, so writing to it only makes ``gripper_node`` print "command
queued (executed by the background thread)" — while the background thread is
long gone. **Logging that says "running" when motion stopped long ago** is the
most dangerous kind of false report in a bridge like this. The raised exception
**must be a separate type**, so that the node can tell it apart from the "the
gate let it through, but the execution layer refused it" contract violation and
does not report a correct safety stop as an internal bug.

``communication_ok`` is filled in from the **actual** communication state
(requirement 13): a frame was read ⇒ ``True``, even if that frame reports a
motor fault — a broken motor does not mean a broken bus.


No SDK import at the top level (requirement 3)
----------------------------------------------
``import litegrip`` is only allowed in two places:

* inside the background thread body (via ``sdk_factory``);
* the fake SDK factory injected by tests.

Never at module top level. So "dry_run does not import the SDK" holds
**structurally**, without relying on the caller's discipline —
``test_sdk_adapter.py`` pins it down with a ``sys.modules`` snapshot.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from .driver_adapter import (
    STOPPED_EPS_RAD_S,
    AdapterLatched,
    Sample,
    TrajectoryLimiter,
    motor_fault_code,
)
from .safety_gate import MOTOR_FAULT_BASE, FaultCode, GateLimits

__all__ = [
    "FORBIDDEN_MOTION_APIS",
    "FORBIDDEN_STREAMING_APIS",
    "FAULT_OVERTEMP_MOS",
    "FAULT_OVERTEMP_COIL",
    "FAULT_EMERGENCY_STOP",
    "AdapterLatched",
    "HardwareConfig",
    "SdkHardwareAdapter",
]


# ═══════════════════════════════════════════════════════════════════════
# List of forbidden interfaces — both documentation and a test-scanned list
# ═══════════════════════════════════════════════════════════════════════

#: The §5.1 "blocking" motion interfaces. **Calling one in a ROS callback
#: stalls the single-threaded executor**; calling one on the background thread
#: is banned just the same — they **sleep** for the whole ``duration=``, with
#: no chance to check for an emergency-stop request or for feedback health,
#: which hands the safety responsibility to the SDK's internal loop.
#: ``recover_to_safe_zone`` is the only channel that moves from outside the red
#: lines inward, but it is a duration-based stream as well and carries "bypass
#: the normal gate" semantics, so this layer does not use it.
FORBIDDEN_MOTION_APIS: Tuple[str, ...] = (
    "open", "close", "move_to", "goto_rad", "goto", "grasp", "home",
    "move_at_speed", "move_at_speed_rad", "set_force", "hand_guided",
    "wait_for_ready", "recover_to_safe_zone",
)

#: The §5.2 ones that are labelled "single-frame send" but are actually
#: **duration-based streams**. They also sleep for the whole ``duration_s``,
#: so they are banned at the same level as §5.1.
#: (The ``LiteGrip`` layer does not expose them — only ``LiteGripCAN`` does.
#:  They are listed here so that the shortcut "bypass LiteGrip and talk to
#:  LiteGripCAN directly" is covered by the list too.)
FORBIDDEN_STREAMING_APIS: Tuple[str, ...] = (
    "send_mit_motion_stream", "send_recovery_stream",
    "enter_zero_gravity", "hand_guided",
)

#: Method names that **do not exist** in the SDK. Writing these by mistake
#: raises AttributeError at runtime — listing them statically makes "we are not
#: guessing at the API" checkable.
#: (The most commonly guessed one is ``read_state()``; the correct entry point
#: is ``get_state()``.)
FORBIDDEN_INEXISTENT_APIS: Tuple[str, ...] = ("read_state",)

#: Smallest distance, in rad, for deciding that "this command **still has
#: travel left**".
#:
#: ★ Why not compare against ``0.0`` directly: ``q_target − position`` is a
#:   subtraction of two floats, so the result can be pure noise like ``1e-17``
#:   rather than a clean 0. Treating that as "still has travel" would judge a
#:   command that has already arrived as not arrived.
#: ★ Why ``1e-9``: it is six orders of magnitude below one **count** of the
#:   DM4310 (about ``3e-4 rad``, see ``dm4310-mit-quantization``) — it cannot
#:   erase any stretch of real travel, and it is immune to floating-point
#:   noise. **Not comparing at the same order of magnitude as the quantization
#:   step** is deliberate: the criterion measures "is there any distance", not
#:   "is it worth one count".
_COMMAND_TRAVEL_EPS_RAD = 1e-9


# ═══════════════════════════════════════════════════════════════════════
# This adapter layer's own fault codes (all above MOTOR_FAULT_BASE)
# ═══════════════════════════════════════════════════════════════════════

#: This layer observed a MOS temperature above ``temperature_limit_c``.
#:
#: ★ Deliberately reusing the SDK's own ``ErrorCode.MOS_OT = 0xB`` /
#: ``COIL_OT = 0xC`` values instead of inventing another number: 0xB/0xC are
#: **the manual's numbers for over-temperature faults**, and a driver that
#: really reports over-temperature answers with these very values. Sharing one
#: number means the subscriber rule "0x8B means MOS over-temperature" **does
#: not** fork depending on who noticed it first.
#:
#: ⚠ A literal is written here instead of ``ErrorCode.MOS_OT``: the module top
#: level must not import the SDK (requirement 3). One test pins these two
#: numbers against the real SDK's ``ErrorCode``.
FAULT_OVERTEMP_MOS = MOTOR_FAULT_BASE + 0xB
FAULT_OVERTEMP_COIL = MOTOR_FAULT_BASE + 0xC

#: A ROS-side emergency-stop request was received, and ``emergency_stop()``
#: has been executed on the motor.
#:
#: ``0xFD`` (the number of the driver's disable command) is used instead of
#: picking another bit: the meaning is exactly "a disable has already been
#: issued", and ``MOTOR_FAULT_BASE + 0xFD`` collides with no ERR code in the
#: manual (the ERR field is 4 bit, max 0xF, so 0xFD is beyond what that field
#: can encode).
FAULT_EMERGENCY_STOP = MOTOR_FAULT_BASE + 0xFD


# ═══════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class HardwareConfig:
    """Every adjustable knob of the real-hardware path. **Validated at
    construction time**, so an illegal value cannot be built.

    The validation here only covers "the value ranges this layer itself uses".
    The **safety ceilings** of ``kp`` / ``kd`` / ``tau`` (``kp_max=200``,
    ``kd_max=5``, ``tau_max_nm``) are authoritatively decided by the SDK's
    ``SafetyLimits.guard_motion_frame``, which raises ``LimitViolation`` — this
    layer **does not copy** those constants. A copy means that the day the SDK
    tightens one, this side still holds the old value, and so "this layer lets
    it through, the SDK refuses it", or worse "this layer thinks it capped it,
    when it did not". The criterion lives in exactly one place.
    """

    #: CAN channel. Requirement 11: Classic CAN, bitrate 1000000 (bittiming is
    #: configured on the system side by ``ip link``, this layer does not touch
    #: it — see ``scripts/can0_up.sh``).
    channel: str = "can0"
    #: Gripper CAN ID (requirement 11: ``0x08``).
    can_id: int = 0x08
    #: Host/feedback ID (requirement 11: ``0x18``).
    mst_id: int = 0x18
    #: Classic CAN. Any value other than ``False`` is rejected — the protocol
    #: baseline must not drift.
    canfd_mode: bool = False

    #: Position stiffness — an **upper bound**, not the final value. Every
    #: frame re-allocates it from this command's torque budget and the
    #: **worst-case** position error bound, see
    #: :meth:`SdkHardwareAdapter._gains_for_budget`.
    kp: float = 20.0
    #: Velocity damping — likewise an **upper bound**. ``dq_target`` is always
    #: 0, so this term only produces a damping torque **opposite to the
    #: direction of motion** (it always brakes, it never pushes the axis out by
    #: itself). ★ **Damping first** when allocating the budget: if the budget
    #: is short, ``kd`` is kept and ``kp`` is cut; only if even ``kd`` does not
    #: fit is it cut itself. See :meth:`_gains_for_budget` for the reasoning.
    kd: float = 0.5

    # ── Worst-case bounds (Stage 5.3) ───────────────────────────────
    #
    # The ``kp`` / ``kd`` in the frame are handed to the **driver**: it
    # **continuously** computes ``kp·(q_target − q_act) + kd·(0 − q̇)`` from
    # them, and ``q_act`` is measured by the driver **itself in real time**,
    # not the sample this layer happened to take. So between two ROS control
    # periods (``period_s`` = 20 ms), both the error and the velocity can
    # differ from the sampling instant.
    #
    # ⇒ If ``torque_limit_nm`` is to really mean "the total torque budget", the
    #   gain scaling must not use the **current sample's** ``|Δq|`` / ``|q̇|``;
    #   it must use the **worst value that can occur in this window**. It is
    #   most obvious at ``e = 0``: at that instant the position term really
    #   spends no budget, yet a millisecond later the driver multiplies the
    #   same ``kp`` by an error that is not 0.
    #
    #: Worst-case position error bound ``error_bound`` (rad). **Defaults to
    #: ``None``.**
    #:
    #: With ``None`` it is derived from the **red-line width**:
    #: ``red_max_rad − red_min_rad``. That is a **provable hard bound**, not an
    #: estimate — the SDK's ``guard_motion_frame`` forces both ``q_target`` and
    #: the **measured** ``q_act`` to lie inside the red lines (see
    #: ``require_act_within_red`` in ``safety_limits.py``), so the absolute
    #: value of their difference cannot exceed the width of the interval.
    #: Derivation ≠ fabrication.
    #:
    #: ★ An explicit value can only **tighten** it: what is actually used is
    #:   ``min(given value, red-line width)``. A value larger than the red-line
    #:   width does not loosen this bound.
    max_position_error_rad: Optional[float] = None

    #: Worst-case feedback velocity bound ``velocity_bound`` (rad/s).
    #: **Defaults to ``None``.**
    #:
    #: ★★ **``None`` is fail-closed: the real-hardware path refuses to send
    #: any motion frame.** Not out of caution, but because **this bound has no
    #: source anywhere in the existing code** — audit conclusion (Stage 5.3):
    #:
    #: * for the measured ``dq_act``, ``guard_motion_frame`` **only checks
    #:   finiteness**
    #:   (``if dq_act is None or not isinstance(...) or not math.isfinite(...)``),
    #:   **with no magnitude limit at all**;
    #: * ``protocol.DM4310_DQ_MAX_RAD_S = 30.0`` is the **mapping range of the
    #:   12-bit ``dq`` field**, not a protection threshold — it says whether the
    #:   encoding can represent a value, not whether that value is allowed. And
    #:   it acts on ``dq_target``, which is not the same thing as this layer's
    #:   ``q̇_act``;
    #: * ``TemporaryParams.recovery_dq_max = 0.5`` lives in the **recovery
    #:   channel** only, and constrains ``dq_target`` only;
    #: * ``v_allow()`` is the ``dq_target`` gate of the deceleration zone, and
    #:   likewise does not constrain ``q̇_act``;
    #: * the ``safety_limits.json`` baseline contains no velocity field at all.
    #:
    #: Using any of the above as ``velocity_bound`` would be **fabrication**:
    #: 30.0 would blow the bound up to something meaningless, and 0.5 is a
    #: number from another channel. So no default is given here — it must come
    #: from **calibration** (measuring on the real gripper the maximum speed it
    #: can reach within a ``period_s`` window); until then this layer **sends no
    #: motion frame** and writes the reason into the status and the log.
    max_feedback_velocity_rad_s: Optional[float] = None

    #: Background control-loop rate. DM motors need a continuous MIT frame
    #: stream to keep moving; the SDK's own streaming interface uses 200 Hz
    #: (``interval_s=0.005``). This layer defaults to 50 Hz — plenty for
    #: holding a position, and it leaves the main loop room to check the
    #: emergency stop and the feedback.
    #: ★ Raising it is not safer, it only moves closer to the SDK's own
    #: throttling rate.
    #: ★ It also defines the **window length** ``period_s`` that both gains
    #: have to cover: the driver keeps using the previous frame's ``kp`` /
    #: ``kd`` for that long.
    control_rate_hz: float = 50.0

    #: Feedback timeout. No **new** feedback frame for this long means the
    #: communication is judged lost. 0.5 s is far larger than the 50 Hz loop
    #: period, so one scheduling hiccup cannot trip it, and far smaller than
    #: the order of magnitude of the DM driver's own watchdog timeout, so it is
    #: noticed before the motor drops out of the enabled state.
    feedback_timeout_s: float = 0.5

    #: Temperature ceiling (MOS and coil are each checked once, in ℃). **This
    #: layer's threshold**, not the SDK's — what the SDK reports is the
    #: driver's over-temperature fault (0xB/0xC), which already happens after
    #: its own protection. This one is "stop before the driver trips by
    #: itself".
    temperature_limit_c: int = 80

    #: How many consecutive frames that **fail to go out** trigger a safety
    #: stop. A single dropped frame (TX buffer full) is transient and does not
    #: latch; N of them in a row mean the link really is in trouble.
    #:
    #: ⚠ A dropped motion frame is **not a no-op**: the driver received
    #: nothing and keeps following the previous command. So the count is
    #: "consecutive", not "cumulative" — one successful frame in between
    #: resets it to zero.
    max_consecutive_send_failures: int = 5

    #: Near-zero velocity criterion (rad/s) used to set ``stopped``. Only
    #: **strictly below** it counts as stopped.
    #:
    #: ★ The default points at ``driver_adapter.STOPPED_EPS_RAD_S`` — the one
    #: place that threshold is defined. ``MockAdapter`` uses the same constant,
    #: so dry_run and the real hardware give the same answer to "what counts as
    #: stopped". **No literal is written here**: once two ``1e-3`` values drift
    #: apart, the symptom is "the same motionless mechanism reports different
    #: stopped values on the two paths", and that kind of difference is
    #: extremely hard to notice — the two paths do not even run in the same
    #: process.
    stopped_eps_rad_s: float = STOPPED_EPS_RAD_S

    #: Upper bound (seconds) for ``close()`` waiting on the background thread
    #: to finish. On timeout it only logs and stops waiting — being stuck in
    #: ``join`` would leave the node unable to stop even with Ctrl-C.
    join_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        if not isinstance(self.channel, str) or not self.channel.strip():
            raise ValueError(f"channel must be a non-empty string: {self.channel!r}")
        for name in ("can_id", "mst_id"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an int: {value!r}")
            if not 0 <= value <= 0x7FF:
                raise ValueError(
                    f"{name}=0x{value:X} outside 11-bit standard frame range [0, 0x7FF]"
                )
        if self.canfd_mode is not False:
            raise ValueError(
                f"canfd_mode={self.canfd_mode!r} — this bridge runs Classic CAN "
                "only, CAN FD must not be enabled"
            )
        if isinstance(self.kp, bool) or not isinstance(self.kp, (int, float)):
            raise ValueError(f"kp is not a number: {self.kp!r}")
        if not math.isfinite(self.kp) or self.kp <= 0.0:
            raise ValueError(f"kp must be a positive finite number: {self.kp!r}")
        if isinstance(self.kd, bool) or not isinstance(self.kd, (int, float)):
            raise ValueError(f"kd is not a number: {self.kd!r}")
        if not math.isfinite(self.kd) or self.kd < 0.0:
            raise ValueError(f"kd must be a non-negative finite number: {self.kd!r}")
        if not math.isfinite(self.control_rate_hz) or self.control_rate_hz <= 0.0:
            raise ValueError(
                f"control_rate_hz must be a positive finite number: "
                f"{self.control_rate_hz!r}")
        if not math.isfinite(self.feedback_timeout_s) \
                or self.feedback_timeout_s <= 0.0:
            raise ValueError(
                f"feedback_timeout_s must be a positive finite number: "
                f"{self.feedback_timeout_s!r}")
        if isinstance(self.temperature_limit_c, bool) \
                or not isinstance(self.temperature_limit_c, int) \
                or self.temperature_limit_c <= 0:
            raise ValueError(
                f"temperature_limit_c must be a positive integer: "
                f"{self.temperature_limit_c!r}")
        if self.max_consecutive_send_failures < 1:
            raise ValueError(
                "max_consecutive_send_failures must be at least 1: "
                f"{self.max_consecutive_send_failures!r}")
        if isinstance(self.stopped_eps_rad_s, bool) \
                or not isinstance(self.stopped_eps_rad_s, (int, float)) \
                or not math.isfinite(self.stopped_eps_rad_s) \
                or self.stopped_eps_rad_s <= 0.0:
            raise ValueError(
                f"stopped_eps_rad_s must be a positive finite number: "
                f"{self.stopped_eps_rad_s!r}"
                " — 0 would make every velocity count as \"stopped\", and a "
                "negative value would be true forever")
        # ★ The two worst-case bounds: **None is allowed** (None has a
        #   well-defined meaning, see the field comments), but once a value is
        #   given it must be a positive finite number — a bound of 0 or NaN
        #   would reduce the worst-case budget to empty words, and failing
        #   construction is better than silently letting it do nothing.
        for name in ("max_position_error_rad", "max_feedback_velocity_rad_s"):
            value = getattr(self, name)
            if value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} is not a number: {value!r}")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be a positive finite number (or None): {value!r}")

    @property
    def period_s(self) -> float:
        return 1.0 / float(self.control_rate_hz)


# ═══════════════════════════════════════════════════════════════════════
# Single-instance guard (requirement 9)
# ═══════════════════════════════════════════════════════════════════════
#
# can0 is an **exclusive** resource. Two LiteGrip instances each opening a
# SocketCAN socket bound to the same channel does not end in "each minding its
# own business": both sides receive the other's feedback frames and both sides
# send command frames onto the bus — one motor driven by two control loops at
# the same time, each of them believing it is right.
#
# So "there can only be one" is made a **construction-time constraint**: asking
# for a second one raises. It sits in the constructor rather than at every call
# site, because call sites multiply and this one does not.

_LIVE_LOCK = threading.Lock()
_LIVE_ADAPTER: Optional["SdkHardwareAdapter"] = None


def _claim_single_instance(adapter: "SdkHardwareAdapter") -> None:
    global _LIVE_ADAPTER
    with _LIVE_LOCK:
        if _LIVE_ADAPTER is not None and _LIVE_ADAPTER is not adapter:
            raise RuntimeError(
                "A real-hardware adapter layer is already running in this "
                "process — can0 is an exclusive resource, so only one SDK "
                "instance may exist at a time (one process may hold only one "
                "SocketCAN socket). close() that one first, or switch to "
                "dry_run."
            )
        _LIVE_ADAPTER = adapter


def _release_single_instance(adapter: "SdkHardwareAdapter") -> None:
    global _LIVE_ADAPTER
    with _LIVE_LOCK:
        if _LIVE_ADAPTER is adapter:
            _LIVE_ADAPTER = None


class _StdlibLogger:
    """Fallback for when the ``logger`` argument is left out. Good enough;
    it does not pull in rclpy."""

    def __init__(self, name: str) -> None:
        import logging  # noqa: PLC0415 - fallback path, not worth hoisting

        self._log = logging.getLogger(name)

    def info(self, message: str) -> None:
        self._log.info(message)

    def warn(self, message: str) -> None:
        self._log.warning(message)

    def error(self, message: str) -> None:
        self._log.error(message)


def _load_sdk():
    """★ Deferred import — only ever called from the background thread.

    This is also how "dry_run does not import the SDK" is implemented: dry_run
    never constructs this class, so this function is never called.
    """
    import litegrip  # noqa: PLC0415 - a top-level import would break requirement 3

    return litegrip


class SdkHardwareAdapter:
    """The real-hardware adapter layer. **It alone touches hardware**, and only
    from its own background thread.

    :param limits: Gate limits. ``None`` **refuses construction** — without
        red lines there is no real-hardware path (fail-closed). dry_run's mock
        allows ``limits=None``, the real path does not: those are two
        different things.
    :param config: :class:`HardwareConfig`; the default is used when omitted.
    :param sdk_factory: Callable returning the SDK module. Defaults to
        :func:`_load_sdk` (the real ``import litegrip``). Tests inject a fake
        SDK, so the whole real-hardware path can be exercised **without any
        CAN socket**.
    :param logger: Object with the three methods ``info`` / ``warn`` /
        ``error``; defaults to the standard library ``logging``. The node
        passes rclpy's logger in.
    """

    def __init__(
        self,
        limits: Optional[GateLimits],
        config: Optional[HardwareConfig] = None,
        sdk_factory: Optional[Callable[[], object]] = None,
        logger: Optional[object] = None,
    ) -> None:
        if limits is None:
            raise ValueError(
                "The real-hardware adapter layer must be given limits — no "
                "real-hardware path may exist without red lines "
                "(fail-closed). For a closed loop without hardware, use "
                "dry_run's MockAdapter."
            )
        self._limits = limits
        self._config = config if config is not None else HardwareConfig()
        self._sdk_factory = sdk_factory if sdk_factory is not None else _load_sdk
        self._log = logger if logger is not None else _StdlibLogger(__name__)

        # ── Shared between threads (all of it goes through this one lock) ──
        self._lock = threading.Lock()
        #: Target slot: ``(q_target, torque_limit_nm, max_velocity_rad_s)``,
        #: ``None`` = no command has been received yet.
        #: ★ The three values **live and die together**: ``q_target`` is the
        #: destination, ``max_velocity_rad_s`` is the speed cap for going
        #: there, ``torque_limit_nm`` is the torque budget for the trip.
        #: Splitting them into two slots would make "the target changed but the
        #: speed is still from the previous command" possible — a combination
        #: nobody ever commanded.
        self._pending: Optional[Tuple[float, float, float]] = None
        self._estop_requested = False
        self._sample = Sample(
            position_rad=0.0,
            velocity_rad_s=0.0,
            torque_nm=0.0,
            error=True,
            stopped=True,
            enabled=False,
            communication_ok=False,
            fault_code=int(FaultCode.NO_FEEDBACK),
        )
        self._detail = "adapter layer not started yet"
        self._fault = int(FaultCode.NO_FEEDBACK)

        # ── State touched only by the background thread ─────────────
        self._thread: Optional[threading.Thread] = None
        self._closing = threading.Event()
        self._last_rx = 0.0
        self._send_failures = 0
        self._claimed = False
        #: ★ Rate-limited trajectory. **Touched only by the background
        #: thread**, same as ``_last_rx``. It remembers "the position actually
        #: sent in the previous frame"; the next frame may advance at most
        #: ``max_velocity_rad_s × elapsed`` from there.
        self._trajectory = TrajectoryLimiter()
        #: Instant the previous motion frame was **sent successfully**
        #: (``time.monotonic()``). ``None`` = no motion frame has been sent
        #: successfully yet. ★ Timed from "frame sent successfully", not from
        #: "entered the loop": the driver received nothing for a frame that
        #: failed to go out, it is still executing the earlier command, and
        #: only ``elapsed`` measured from back then answers "how long has the
        #: driver been walking on the old command".
        self._last_motion_tx: Optional[float] = None
        #: ★★ The command whose "gains actually obtained by this command" has
        #: already been reported. **Touched only by the background thread.**
        #:
        #: Deduplicated on the **target tuple** rather than on a counter: two
        #: identical commands in a row would get identical gains anyway, so
        #: reporting again would only copy the same line into the log. Changing
        #: the target (including A→B→A) still re-reports — the second A is not
        #: equal to the remembered B.
        self._reported_target: Optional[Tuple[float, float, float]] = None
        #: ★ Whether a safety stop has been latched. **Written only by the
        #: background thread's fault path**, read-only on the command path.
        #: Once set, :meth:`set_target` always raises :class:`AdapterLatched`
        #: — the loop has exited, so writing anything into the target slot
        #: would only make the node print "command queued (executed by the
        #: background thread)" while the background thread is long gone.
        #: This field is protected by ``self._lock``.
        self._latched = False
        #: Safety exception types raised by the SDK (read from the SDK module
        #: at construction time), for isinstance checks.
        self._safety_exc: Tuple[type, ...] = ()
        #: If the background thread exits on an unexpected exception, the
        #: exception object is kept here for diagnosis. It is **not
        #: swallowed** — it is also recorded into ``_fault`` and logged at
        #: ERROR level.
        self.thread_exception: Optional[BaseException] = None

    # ── Read-only views ─────────────────────────────────────────────
    def limits(self) -> Optional[GateLimits]:
        """The ``limits()`` required by the ``GripperAdapter`` protocol.

        ★ Made a **method** rather than a property to match
        ``MockAdapter.limits()`` — the adapter contract
        (``driver_adapter.GripperAdapter``) specifies a method, and both
        implementations must be interchangeable by the same calling code.
        A property would "look nicer" here, but it would make the real and the
        mock path diverge in how they are called, which is exactly what the
        adapter layer exists to remove.
        """
        return self._limits

    @property
    def config(self) -> HardwareConfig:
        return self._config

    @property
    def is_running(self) -> bool:
        thread = self._thread
        return bool(thread is not None and thread.is_alive())

    @property
    def detail(self) -> str:
        """One-sentence description of the most recent status. For diagnostic
        output only."""
        with self._lock:
            return self._detail

    # ── Callback-side interface: none of it touches the SDK ─────────
    def set_target(
        self,
        position_rad: float,
        torque_limit_nm: float,
        max_velocity_rad_s: float,
    ) -> None:
        """Write the target slot. **Returns as soon as it is uploaded**, with
        no hardware operation whatsoever.

        ``torque_limit_nm`` is **the maximum total control torque allowed this
        time (the budget)**, not a feedforward torque — it is never sent out
        as ``tau`` directly; instead every frame **allocates** it between
        ``kp`` and ``kd`` according to the current position error and feedback
        velocity, see :meth:`_gains_for_budget` and :meth:`_send_motion`.

        ``max_velocity_rad_s`` is **the trajectory speed cap for this position
        move** (rad/s) — ``position_rad`` is the **destination**, not the
        value sent out in the next frame. What is actually sent each frame is
        the point the rate-limited trajectory currently sits at, see the
        "rate limiting" part of :meth:`_send_motion`.
        ★ It is **not** the ``dq`` of the MIT frame (that field is always 0.0),
        and **not** :attr:`HardwareConfig.max_feedback_velocity_rad_s` (the
        worst-case bound on measured velocity). See ``GripperCommand.msg`` for
        how the three relate.

        ★ The order of the checks is **latched → value range → write slot**,
        all three done **under the same lock**. Latched comes first because
        once it holds, judgements like "target out of range" mean nothing: if
        the out-of-range case reported ValueError first, the node would take it
        for a contract violation (``INTERNAL``), and the real cause (the
        hardware stopped by design) would be buried.

        :raises AdapterLatched: This layer has latched a safety stop. It is
            **not the same kind of thing** as the ``ValueError`` below — the
            former is "stopped by design", the latter is the contract
            violation "the gate let it through, but the execution layer
            refused it". Callers must handle the two separately, hence two
            types.
        :raises ValueError: The target does not satisfy this layer's
            preconditions. That is a **contract violation** (the upstream gate
            should already have blocked it), so it is an exception and not a
            return value — ignoring it silently would make "the command was
            swallowed" look exactly like "the command is running".
            The caller (``gripper_node``) is responsible for turning it into a
            visible fault code, so the exception does not travel into the
            executor.
        """
        q = float(position_rad)
        tau = float(torque_limit_nm)
        v_max = float(max_velocity_rad_s)
        with self._lock:
            if self._latched:
                raise self._latched_error()
            self._validate_target(q, tau, v_max)
            self._pending = (q, tau, v_max)
        self._log.info(
            f"target queued: {q:.6f} rad, torque budget {tau} N·m, "
            f"trajectory speed cap {v_max} rad/s")

    def _validate_target(self, q: float, tau: float, v_max: float) -> None:
        """Value-range validation of the target. Read-only, **takes no lock**
        (the caller already holds ``self._lock``)."""
        if not math.isfinite(q) or not math.isfinite(tau) \
                or not math.isfinite(v_max):
            raise ValueError(
                f"target must be finite: q={q!r}, tau={tau!r}, v_max={v_max!r}")
        if tau <= 0.0:
            raise ValueError(f"torque limit must be positive: {tau!r}")
        if v_max <= 0.0:
            raise ValueError(
                f"trajectory speed cap must be positive: {v_max!r} — 0 means "
                f"the target position can never be pushed, and that "
                f"trajectory never reaches its destination"
            )
        limits = self._limits
        assert limits is not None  # guaranteed at construction time
        if not limits.red_min_rad <= q <= limits.red_max_rad:
            raise ValueError(
                f"target {q:.6f} rad is outside the red lines "
                f"[{limits.red_min_rad}, {limits.red_max_rad}]"
            )
        if tau > limits.torque_limit_nm:
            raise ValueError(
                f"torque limit {tau} N·m exceeds the effective cap "
                f"{limits.torque_limit_nm} N·m"
            )
        # ★ Isomorphic to the torque check. ★ The comparison is against
        #   ``limits.max_velocity_rad_s`` (the command trajectory cap), **not**
        #   ``config.max_feedback_velocity_rad_s`` (the measured-velocity
        #   bound) — using the latter to gate a command would quietly turn "I
        #   assert the axis will not spin this fast" into "I command it to
        #   spin this fast".
        if v_max > limits.max_velocity_rad_s:
            raise ValueError(
                f"trajectory speed cap {v_max} rad/s exceeds the effective "
                f"cap {limits.max_velocity_rad_s} rad/s"
            )

    def _latched_error(self) -> AdapterLatched:
        """Build the "already latched" exception. **The caller must already
        hold ``self._lock``.**"""
        return AdapterLatched(
            self._fault,
            f"the execution layer has latched a safety stop "
            f"(fault_code={self._fault}) — it will not execute any new "
            f"target: {self._detail}",
        )

    def request_emergency_stop(self) -> None:
        """Request an emergency stop. **Only sets a flag**; the actual
        ``emergency_stop()`` is executed by the background thread on its next
        cycle.

        The reason is the same as for every other command: the callback
        thread must not wait on hardware. Setting the flag is one locked
        assignment and returns immediately no matter how congested the bus is
        at that moment.
        """
        with self._lock:
            self._estop_requested = True
        self._log.warn(
            "emergency-stop request received — will be executed by the "
            "background thread on its next cycle")

    def snapshot(self) -> Sample:
        """Take the most recent sample. **Non-blocking, does not touch the
        SDK**, safe for the callback thread.

        What comes back is an immutable :class:`Sample`, so it cannot be
        changed by the background thread after you have it — there is no
        in-between state of "read half of it and it was rewritten".
        """
        with self._lock:
            return self._sample

    # ── Lifecycle ───────────────────────────────────────────────────
    def start(self) -> None:
        """Start the background thread. **Beyond being idempotent it also
        claims exclusivity**: the second ``start()`` raises."""
        if self._thread is not None:
            raise RuntimeError("the adapter layer has already been started")
        _claim_single_instance(self)
        self._claimed = True
        try:
            thread = threading.Thread(
                target=self._run, name="litegrip-sdk-adapter", daemon=True)
            thread.start()
        except BaseException:
            # If the thread did not start, the exclusive claim must be given
            # back, otherwise this process can never open a real-hardware path
            # again (and the reason was "the thread failed to start", not
            # "someone else is already using it").
            _release_single_instance(self)
            self._claimed = False
            raise
        self._thread = thread

    def close(self, timeout_s: Optional[float] = None) -> None:
        """Request shutdown and wait for the background thread to exit.
        **Safe to call repeatedly.**

        ``disable()`` / ``disconnect()`` are executed by the background
        thread's ``finally``, **exactly once** (the thread has only one exit).
        This method only: sets the closing flag → ``join`` → returns the
        exclusive claim. So repeated calls do not disable twice.

        ``join`` has a timeout (default
        :attr:`HardwareConfig.join_timeout_s`): on timeout it logs ERROR and
        returns, **it does not wait forever** — being stuck here would leave
        the node unable to stop even with Ctrl-C, which is worse than leaving
        one thread behind.
        """
        self._closing.set()
        thread = self._thread
        if thread is None:
            if self._claimed:
                _release_single_instance(self)
                self._claimed = False
            return
        if thread is not threading.current_thread():
            timeout = (
                self._config.join_timeout_s if timeout_s is None
                else float(timeout_s)
            )
            thread.join(timeout=timeout)
            if thread.is_alive():
                self._log.error(
                    f"background thread did not exit within {timeout:.1f}s — "
                    f"not waiting any longer. It may still hold can0."
                )
        self._thread = None
        if self._claimed:
            _release_single_instance(self)
            self._claimed = False

    # ── Background thread ───────────────────────────────────────────
    def _run(self) -> None:
        """Thread body. **The whole lifecycle lives here, in strict
        order.**"""
        gripper = None
        enabled = False
        try:
            try:
                sdk = self._sdk_factory()
            except Exception as exc:  # noqa: BLE001 - must become a visible fault
                self._record_safe_stop(
                    FaultCode.INTERNAL,
                    f"cannot import the LiteGrip SDK: {exc!r}", comm_ok=False)
                return

            # ★ The safety exception types are taken from the SDK module that
            #   has **already been imported**. That way isinstance can be used
            #   in the except clauses (no comparing class-name strings), while
            #   the SDK still need not be imported at module top level.
            self._safety_exc = tuple(
                t for t in (
                    getattr(sdk, "SafetyFault", None),
                    getattr(sdk, "LimitViolation", None),
                ) if isinstance(t, type)
            )

            ctor = getattr(sdk, "LiteGrip", None)
            if ctor is None:
                self._record_safe_stop(
                    FaultCode.INTERNAL,
                    "the SDK module has no LiteGrip — the interface does not "
                    "match, refusing to continue",
                    comm_ok=False)
                return

            cfg = self._config
            try:
                gripper = ctor(
                    channel=cfg.channel,
                    can_id=cfg.can_id,
                    mst_id=cfg.mst_id,
                    canfd_mode=cfg.canfd_mode,
                )
            except Exception as exc:  # noqa: BLE001
                self._record_safe_stop(
                    FaultCode.INTERNAL,
                    f"constructing LiteGrip failed: {exc!r}", comm_ok=False)
                return

            if not self._require_refresh_capability(gripper):
                return
            if not self._connect_once(gripper):
                return
            if not self._check_initial_state(gripper):
                return
            if not self._enable_once(gripper):
                return
            enabled = True
            self._log.info(
                f"real-hardware path is ready: {cfg.channel} "
                f"can_id=0x{cfg.can_id:02X} "
                f"mst_id=0x{cfg.mst_id:02X}, control rate "
                f"{cfg.control_rate_hz} Hz"
            )
            self._loop(gripper)
        except BaseException as exc:  # noqa: BLE001 - the thread must not die silently
            self.thread_exception = exc
            self._record_safe_stop(
                FaultCode.INTERNAL,
                f"background thread exited on an exception: {exc!r}",
                comm_ok=False)
            if not isinstance(exc, Exception):
                # KeyboardInterrupt / SystemExit and the like: record it and
                # re-raise as before, so "someone is shutting the process
                # down" is not disguised as an ordinary fault.
                raise
        finally:
            # ★ The one and only teardown point — disable() / disconnect() can
            #   only happen here, and each _run passes through it once.
            self._teardown(gripper, enabled)

    def _require_refresh_capability(self, gripper: object) -> bool:
        """The SDK must provide ``refresh_status()``, otherwise **no power-up**.

        ★★ Why this is a **startup-time** hard requirement rather than a
        runtime degradation:

        DM4310 feedback is **poll-style** — on a quiet bus the driver does not
        speak up on its own. When idle this layer keeps the communication alive
        with the zero-torque frame of :meth:`_send_keepalive`, but the reply to
        that frame is read away by ``update_state()`` inside the SDK's own
        ``LiteGrip.stop()`` (see the notes on :meth:`_send_keepalive`). So
        **while idle this layer's ``poll()`` never sees a new frame**,
        ``_last_rx`` stops advancing, and the moment ``feedback_timeout_s``
        expires it stops — which is exactly what causes "enable succeeds, then
        NO_FEEDBACK 0.5 seconds later" on the real hardware.

        Adding one ``0xCC`` status refresh
        (:meth:`_request_status_refresh`) separates "asking" from "collecting"
        and lets the reply frame land in this layer's ``poll()``. **Without
        this interface, this layer cannot maintain verifiable feedback while
        idle** — and then "communication is fine" is a sentence that cannot be
        tested.

        ★ So the choice here is to **refuse to power up**, not to "enable
        first and let it time out on its own": the latter would stop the node
        0.5 seconds after power-up, and the stop reason (NO_FEEDBACK) looks
        like a bus fault — the whole troubleshooting direction would be wrong.
        Better to say up front what is missing.

        ★ The criterion is **callable on the instance**, not the name existing
        on the class: when the SDK migration copy and the upstream repository
        disagree on version, it is the instance that breaks.
        """
        if callable(getattr(gripper, "refresh_status", None)):
            return True
        self._record_safe_stop(
            FaultCode.INTERNAL,
            "the SDK has no status-refresh interface "
            "(LiteGrip.refresh_status) — while idle it cannot request fresh "
            "feedback, and this layer would immediately degrade into a stream "
            "of fake NO_FEEDBACK. **Refusing to power up**. The SDK migration "
            "copy inside the workspace needs to provide that interface (add a "
            "thin wrapper in LiteGripCAN.refresh_status in "
            " protocols/can_bus.py and in LiteGrip.refresh_status in "
            "gripper.py; the underlying MotorController.refresh_status / "
            "pack_refresh_frame have existed all along).",
            comm_ok=False,
        )
        return False

    def _connect_once(self, gripper: object) -> bool:
        """The **only** call site of ``connect()``."""
        try:
            ok = gripper.connect()
        except Exception as exc:  # noqa: BLE001
            self._record_safe_stop(
                FaultCode.INTERNAL, f"connect() raised: {exc!r}", comm_ok=False)
            return False
        if not ok:
            self._record_safe_stop(
                FaultCode.INTERNAL,
                "connect() returned False — no CAN connection was "
                "established, so nothing further is executed",
                comm_ok=False)
            return False
        self._log.info("connect() succeeded (the only time in this process)")
        return True

    def _elicit_one_frame(self, gripper: object) -> None:
        """Ask one question first, so the driver has a reason to answer.

        ★★ Why asking first is **mandatory**: DM4310 feedback is **poll-style**
        — the driver only returns a status frame after it has received a frame
        from this host. ``connect()`` itself **sends no frame**, so right after
        connecting there is not a single host frame on the bus, the driver has
        nothing to answer, and no amount of waiting will produce a feedback
        frame. Proven by capture
        (``test_snapshots/litegrip-p1-torque-plus001-candump.log``): every
        ``008#FFFFFFFFFFFD`` frame (the disable command sent by the host) is
        **immediately followed** by an ``018#087B6F7FF7FE1B1A`` frame (the
        status frame the driver returns).

        ★ Why ``disable()`` is used as this question: it is the **safest**
        command on this bus — the motor is disabled at this point anyway, and a
        disable command cannot make anything move; whereas ``stop()`` (the
        zero-torque frame) returns False outright when not enabled and sends no
        frame at all, so it cannot be used to "ask". The teardown path calls
        ``disable()`` anyway, so this is not a new kind of operation on the
        motor.

        ★ Failing to send **is not treated as fatal here**: this method only
        has to produce one "question"; the verdict comes from the later
        "did we get a fresh frame" check. Reporting a single TX failure as a
        fault of its own would misreport "the bus is actually up, this one
        frame just did not fit in" as a configuration problem.
        """
        try:
            gripper.disable()
        except Exception as exc:  # noqa: BLE001
            self._log.warn(
                f"the pre-enable probe frame (disable) did not go out: "
                f"{exc!r} — continuing to wait for feedback; if none arrives, "
                f"it is handled as NO_FEEDBACK")

    def _await_fresh_frame(self, gripper: object) -> bool:
        """Wait for **at least one newly decoded** feedback frame. Returns
        ``False`` if none arrives.

        ★★ Why ``get_state(wait=True)`` cannot simply be trusted:

        ``LiteGrip.get_state(wait=True)`` is internally
        ``update_state(timeout_s=0.05)`` → read the cache → build a
        ``GripperState``. It **throws away** the return value of
        ``update_state``, and the initial value of ``MotorState._position`` is
        ``0.0`` (``can/motor.py``). So the situation "the wait timed out and
        the cache was never filled" returns a
        ``GripperState(position_rad=0.0, error_code=0, t_mos=0, t_coil=0,
        timestamp=time.time())`` — a snapshot where **every field looks
        normal and the timestamp is still just now**, yet not one number in it
        was measured. The SDK itself writes this down in the docs of
        ``LiteGripCAN.get_rx_count``:

            the values read for position/velocity/torque are the initial
            value 0 when no frame has been received, looking like "position
            is at 0 rad" — that is **fake**. Any logic that "makes a decision
            from the current measured position" must check this counter
            first.

        ★ The criterion here uses the return value of ``poll()``, not that
        counter: ``poll()`` returns ``True`` if and only if
        ``MotorController.poll`` **has just decoded a frame and called**
        ``update_from_status`` (which also increments ``rx_count``). So "poll
        returned True" means "the cache has just been overwritten with real
        data", which is stronger than "the counter is not 0" — the latter only
        says that frames were received **in the past**.

        ★ The budget reuses ``feedback_timeout_s``: its meaning is already "how
        long a silent bus counts as disconnected", the same quantity as "how
        long to wait at most before power-up". Not inventing another parameter
        also avoids a new "this timeout was not configured" state of missing
        configuration.
        """
        budget_s = float(self._config.feedback_timeout_s)
        deadline = time.monotonic() + budget_s
        while True:
            try:
                got = bool(gripper.poll(0.0))
            except Exception as exc:  # noqa: BLE001
                # The SDK's feedback watchdog raises a safety exception here
                # ⇒ a frame really **was** received (its content did not
                # qualify). Handled as "received but unusable", consistent
                # with the matching case inside the loop.
                self._record_safe_stop(
                    FaultCode.UNSAFE_INITIAL_STATE,
                    f"the SDK raised while waiting for feedback before "
                    f"enabling: {exc!r} — a frame was received but its content "
                    f"did not qualify, refusing to enable",
                    comm_ok=True,
                )
                return False
            if got:
                return True
            if time.monotonic() >= deadline:
                return False
            self._closing.wait(0.005)

    def _check_initial_state(self, gripper: object) -> bool:
        """Read **and check** the initial state. **Never enable() unless it
        passes** (requirement 10).

        The order is **ask first, then wait, and read last**; none of the three
        steps may be skipped:

        1. :meth:`_elicit_one_frame` — send one frame out so the driver has a
           reason to answer (poll-style feedback: no outgoing frame, never a
           reply frame);
        2. :meth:`_await_fresh_frame` — wait until ``poll()`` really decodes a
           new frame; **if none arrives, report ``NO_FEEDBACK`` and refuse to
           enable**, never taking the cached initial value for a position;
        3. read the state and clear the four gates.

        ★★ Step 3 reads the cache **that real data has just overwritten**, so
        the four gates judge "where the mechanism is now". The old version went
        straight to ``get_state(wait=True)``; with no frame received yet it got
        back ``position_rad=0.0``, the **initial value** — not a measurement,
        yet it would be taken as "the axis is at 0 rad" for the power-up
        decision.

        ★ That 0.0 happens to be caught by the red lines
        (``0.0 > red_max_rad = -0.01``), so the old version **looked**
        fail-closed too. But what caught it was a **position-value** criterion,
        unrelated to "is there data at all": the moment the red lines are not a
        fully negative interval, this fake 0.0 sails through all four gates
        into ``enable()``. More importantly, the diagnosis would say "position
        0.000000 rad is outside the red lines", sending people to inspect the
        mechanism, when the real problem is that **no feedback was received at
        all**.

        The four gates (after step 2 has confirmed a fresh frame):

        1. the position is finite (never enable when the encoder gives
           NaN/inf);
        2. the position is inside the **red lines**. The SDK allows enabling
           from "outside the red lines but within the mechanical range" and
           entering its recovery flow — this bridge **does not** use that
           path: it needs duration-based streaming interfaces like
           ``recover_to_safe_zone``, and the entire design premise of this
           layer is that they are never called. Stopping outside the red lines
           leaves it to a human.
        3. the motor reports no error (``error_code`` ∉ {0, 1} means an error;
           0 = not enabled, 1 = enabled);
        4. the temperature is below this layer's threshold.

        ``communication_ok`` is filled in from the **actual** situation
        (requirement 13): a frame was read ⇒ ``True``, even if what was read is
        judged unsafe — a motor with a fault does not mean a broken bus. Only
        "could not read" makes it ``communication_ok=False``.
        """
        # ── ① ask first ─────────────────────────────────────────────
        self._elicit_one_frame(gripper)

        # ── ② then wait for one **new** frame ───────────────────────
        if not self._await_fresh_frame(gripper):
            self._record_safe_stop(
                FaultCode.NO_FEEDBACK,
                f"no fresh feedback frame arrived within "
                f"{self._config.feedback_timeout_s}s before enabling — "
                f"**refusing to enable**. ★ The position_rad read at this "
                f"point is the cached initial value 0.0, not a measurement: "
                f"using it to judge \"may we power up\" means making a power-up "
                f"decision from a position that was never measured. Possible "
                f"causes: driver not powered / CAN not connected / wrong "
                f"bitrate / broken wire.",
                comm_ok=False,
            )
            return False

        # ── ③ read the cache (real frames have just overwritten it) ──
        try:
            state = gripper.get_state(wait=False)
        except Exception as exc:  # noqa: BLE001
            self._record_safe_stop(
                FaultCode.UNSAFE_INITIAL_STATE,
                f"failed to read the initial state before enabling: {exc!r}",
                comm_ok=False)
            return False

        limits = self._limits
        assert limits is not None
        problems = []

        try:
            position = float(state.position_rad)
        except Exception as exc:  # noqa: BLE001
            problems.append(f"position_rad is not a number ({exc!r})")
            position = float("nan")
        if not math.isfinite(position):
            problems.append(f"position is not finite ({position!r})")
        elif not limits.red_min_rad <= position <= limits.red_max_rad:
            problems.append(
                f"position {position:.6f} rad is outside the red lines "
                f"[{limits.red_min_rad}, {limits.red_max_rad}]"
            )

        try:
            error_code = int(state.error_code)
        except Exception:  # noqa: BLE001
            error_code = -1
        fault = motor_fault_code(error_code)
        if fault != 0:
            problems.append(
                f"the motor reports an error, error_code=0x{error_code:X}"
                f" ({'unknown code' if error_code < 0 else 'not 0/1'})"
            )

        temps = self._read_temperatures(state)
        if temps is not None:
            t_mos, t_coil = temps
            limit_c = self._config.temperature_limit_c
            if t_mos >= limit_c:
                problems.append(
                    f"MOS temperature {t_mos}℃ ≥ threshold {limit_c}℃")
            if t_coil >= limit_c:
                problems.append(
                    f"coil temperature {t_coil}℃ ≥ threshold {limit_c}℃")

        if problems:
            self._record_safe_stop(
                FaultCode.UNSAFE_INITIAL_STATE,
                "initial state check failed, **refusing to enable**: "
                + "; ".join(problems),
                comm_ok=True,
                position_rad=position if math.isfinite(position) else None,
            )
            return False

        # ★ It passed. Record the first frame — from here on
        #   ``communication_ok`` has something to stand on.
        self._last_rx = time.monotonic()
        with self._lock:
            self._detail = (
                f"initial state check passed: position {position:.6f} rad, "
                f"error_code=0x{error_code:X}"
            )
        return True

    def _enable_once(self, gripper: object) -> bool:
        """The **only** call site of ``enable()``. Reached only after the
        initial state check has passed."""
        try:
            ok = gripper.enable()
        except Exception as exc:  # noqa: BLE001
            self._record_safe_stop(
                FaultCode.UNSAFE_INITIAL_STATE,
                f"enable() raised: {exc!r}", comm_ok=True)
            return False
        if not ok:
            self._record_safe_stop(
                FaultCode.UNSAFE_INITIAL_STATE,
                "enable() returned False — the motor did not enter the "
                "enabled state. (``is_enabled`` is equivalent to "
                "error_code == 1; a True return is also **not** the same as "
                "really being enabled, that layer has its own evidence in the "
                "high 4 bits of ERR.)",
                comm_ok=True)
            return False
        self._log.info("enable() succeeded")
        return True

    def _loop(self, gripper: object) -> None:
        """Control loop. **Returning means teardown** (normal shutdown or a
        safety stop has been latched)."""
        period = self._config.period_s
        while not self._closing.is_set():
            started = time.monotonic()
            if not self._cycle(gripper):
                return
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                self._closing.wait(remaining)

    def _cycle(self, gripper: object) -> bool:
        """One control cycle. **Returning False = leave the loop.**

        The order is deliberate:

        1. emergency-stop request — highest priority, ahead of reading state
           and sending frames;
        2. read one feedback frame (non-blocking);
        3. feedback-timeout check (communication lost);
        4. only with a new frame, run the health checks (motor fault /
           over-temperature);
        5. send a frame: a position frame if there is a target, a keepalive
           frame if not.
        """
        # ① Emergency stop — before everything else.
        if self._estop_is_pending():
            self._do_emergency_stop(gripper)
            return False

        # ② Read feedback. poll() is non-blocking (timeout_s=0).
        try:
            got = bool(gripper.poll(0.0))
        except Exception as exc:  # noqa: BLE001
            # ★ When poll() raises a safety exception, the SDK has already
            #   sent a zero-torque frame for us
            #   (the watchdog branch of LiteGripCAN._check_feedback).
            #   comm_ok is filled in from the facts: reaching this point means
            #   frames **were** received.
            safety = isinstance(exc, self._safety_exc) if self._safety_exc else False
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP if safety
                else FaultCode.NO_FEEDBACK,
                f"poll() failed "
                f"({'SDK safety exception' if safety else 'non-safety exception'}): "
                f"{exc!r}",
                comm_ok=safety,
            )
            return False

        now = time.monotonic()
        if got:
            self._last_rx = now
            # ★ This does **not** clear ``_send_failures``.
            #   That counts "how many frames in a row **failed to go out**",
            #   so only a **successful send** breaks the run. The earlier
            #   version cleared it on "feedback received", and since the
            #   feedback is poll-style and arrives almost every cycle, the
            #   counter was zeroed every cycle and
            #   ``max_consecutive_send_failures`` could never be reached —
            #   that protection became dead code, and losing every frame still
            #   only ever reported "1/N". The reset lives in the success branch
            #   of :meth:`_send_motion`.

        if now - self._last_rx > self._config.feedback_timeout_s:
            # ③ Communication lost. **Send no more motion frames** — without
            #    the current state, anything sent would be blind. Best-effort
            #    send of one zero-torque frame (it may already be impossible to
            #    send, and a failure is not escalated, because the bus was
            #    already down).
            self._best_effort_stop(gripper)
            self._record_safe_stop(
                FaultCode.NO_FEEDBACK,
                f"feedback timeout {now - self._last_rx:.3f}s > "
                f"{self._config.feedback_timeout_s}s — communication lost, "
                f"stopped sending motion frames",
                comm_ok=False,
            )
            return False

        if got:
            # ④ A new frame → update the cache and run the health checks.
            try:
                state = gripper.get_state(wait=False)
            except Exception as exc:  # noqa: BLE001
                self._record_safe_stop(
                    FaultCode.NO_FEEDBACK,
                    f"get_state(wait=False) failed: {exc!r}", comm_ok=False)
                return False
            self._update_from_state(state)
            if self._current_fault() != 0:
                # The health checks already flagged a problem (motor fault /
                # over-temperature).
                self._do_safe_stop(gripper)
                return False

        # ⑤ Send a frame.
        target = self._take_target()
        if target is None:
            # No command has ever been received — **do not drive**, just keep
            # the communication alive. A frame must be sent: DM motors have
            # poll-style feedback, so without an outgoing frame no feedback
            # frame comes back, and "not driving" would degrade into "unable to
            # see".
            return self._send_keepalive(gripper)
        return self._send_motion(gripper, target)

    # ── Sending frames ──────────────────────────────────────────────
    def _send_motion(self, gripper: object, target: Tuple[float, float]) -> bool:
        """Send one position-control frame. — The **only** call in this layer
        that can produce motion.

        It uses ``LiteGrip.send_mit_frame()`` (non-blocking single frame, see
        the verdict in the module docstring). Internally that goes through the
        ``_guard_motion`` full red-line check: a **measured position** or a
        target position crossing the red lines always raises
        ``LimitViolation``, it never silently clamps. This layer does not
        repeat those checks — the criterion lives in exactly one place.

        The parameter mapping is **one for one** (``GripperCommand`` →
        ``send_mit_frame``)::

            q_target              → q    target position
            _gains_for_budget[0]  → kp   stiffness (after **worst-case**
                                         budget allocation)
            _gains_for_budget[1]  → kd   damping (after **worst-case** budget
                                         allocation)
            0.0                   → dq   target velocity always 0
            0.0                   → tau  feedforward torque always 0 ★

        ★ **``torque_limit_nm`` is not here.** It is neither ``tau`` nor does
        it appear in any field of the frame — it is only used by
        :meth:`_gains_for_budget` to **allocate** ``kp`` and ``kd``. That is
        the core of the command message's semantics (see
        ``GripperCommand.msg``): ``torque_limit_nm = 2.0`` means "this trip
        exerts at most 2 N·m", **not** "continuously output 2 N·m". The latter
        would turn the gripper into a 2 N·m constant-force pusher — a
        completely different and more dangerous action.

        There is a second, independent reason why ``tau`` is always passed as
        ``0.0``: the SDK's send-side gate
        (``require_zero_feedforward_unless_calibrated``, ``can_bus.py:388``)
        **only lets through a feedforward torque strictly equal to 0.0** while
        the force calibration is incomplete, and a non-zero value raises before
        the frame goes out. So even if the budget were meant to be sent as
        ``tau``, it could not be — but the real reason is the semantics above:
        **it is not a torque to begin with**.

        ★ **How the torque is bounded (three layers, none of them optional)**

        ====  ================================  ==========================
        Layer Mechanism                         Nature
        ====  ================================  ==========================
        ROS   ``safety_gate.check_command``     ① **Request level**: a
              vs ``limits.torque_limit_nm``     ``torque_limit_nm`` over the
              (= min(params, SDK tau_max, 3.5)) effective cap rejects the whole
                                                command — no clamp, no pass
        Bridge :meth:`_gains_for_budget`        ② **Look-ahead**: allocates the
                                                two gains from the **worst-case**
                                                bounds so that
                                                ``kp·e_b + kd·v_b ≤ budget``
                                                holds for the **whole control
                                                cycle** (damping first)
        SDK   ``guard_motion_frame``            ③ **Retrospective**: ``kp ≤ 200``,
              (``safety_limits.py:1235``)       ``kd ≤ 5.0``, ``|tau_ff| ≤
                                                tau_max``; and a **measured**
                                                ``tau_act`` already over the limit
                                                while this frame still pushes the
                                                same way → reject
        ====  ================================  ==========================

        ② is the only **look-ahead** layer — it keeps the torque the command
        computes from exceeding the budget in the first place. ③ is not a
        clamp but a **rejection**, and it works from the **already measured**
        torque, so it is an after-the-fact interception. So the criterion for
        "was the budget taken seriously" lives in ②, not in ③.
        ③ must still be kept: ② rests on the **assertion** "``|q̇| ≤ v_b``",
        and reality can overturn that assertion (a violent external push, a
        mechanically jammed axis); only the measured ③ can see that part.
        ② has a check on the assertion itself too — the moment the measured
        velocity crosses ``v_b``, it stops.

        ⚠ **The damping term is inside the budget as well**, and it is
        allocated **first**. The actual output is approximately
        ``kp·Δq − kd·q̇``; :meth:`_gains_for_budget` guarantees
        ``kp·e_b + kd·v_b ≤ budget``, and ``|Δq| ≤ e_b``, ``|q̇| ≤ v_b``, so by
        the triangle inequality that **implies** ``|kp·Δq − kd·q̇| ≤ budget``.
        The sum of absolute values is used instead of the sum itself because
        the sign relationship between ``Δq`` and ``q̇`` changes with the
        situation (same sign while approaching, opposite after overshoot),
        and a sign-agnostic bound holds in every situation — the "conservative"
        place to land.

        ⚠ **Worst-case bounds are used, not the current sample.** The ``kp`` /
        ``kd`` in the frame are used **continuously** by the driver throughout
        the whole control cycle, and its ``q_act`` is measured by itself in
        real time. Scaling by the ``|Δq|`` / ``|q̇|`` of the sampling instant
        only guarantees "not over budget at that instant", not "not over budget
        for this cycle". ``e = 0`` is the clearest counter-example: at that
        instant the position term really spends no budget, but a millisecond
        later the driver multiplies the same ``kp`` by an error that is not 0.

        ⚠ All three of the following cases **refuse to send and latch**
        (``HARDWARE_SAFE_STOP``):

        * **The worst-case bounds cannot be derived** (``comm_ok=False``) — see
          :meth:`_worst_case_bounds`. The only one that can currently be
          missing is the velocity bound; it has no source in the existing code
          and must be explicitly configured after calibration. **No default is
          invented for it.**
        * **The velocity feedback is unusable**, and it is **not assumed to be
          0** (``comm_ok=False``). Treating it as 0 would make the
          "worst case" look generous and would open up the drive force exactly
          while the axis may be moving fast.
        * **The measured velocity crosses ``v_b``** (``comm_ok=True`` —
          communication is fine, it is the **assertion** that is wrong). The
          premise of the whole budget derivation has been falsified, so
          sending more frames would be computing torque from a false premise.
        """
        q_target, torque_budget, max_velocity = target

        # ★★ Requirements 4/8: **no worst-case bounds, no motion frame**.
        #    This one comes before reading the feedback — the bounds are a
        #    property of the configuration and the red lines, independent of
        #    this frame's sample, and without them whatever is sampled later
        #    is wasted arithmetic.
        #
        #    The only one that can currently be missing is the velocity bound
        #    (the position error bound can be derived from the red-line width).
        #    The real-hardware path **stops right here** until that value has
        #    been calibrated out; that is the intent: without a velocity bound
        #    the sentence "the worst case stays within budget" cannot be
        #    written, and sending a torque you cannot even prove is worse than
        #    not sending one.
        bounds = self._worst_case_bounds()
        if bounds is None:
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP,
                self._missing_bounds_detail(),
                comm_ok=False,
            )
            return False
        error_bound, velocity_bound = bounds

        position, velocity = self._last_motion_state()

        # ★★ With unusable feedback it is **fail-closed**, and the unknown
        #    velocity is **never taken as 0**. Treating them as 0 would make
        #    the "worst case" look generous while we actually have no idea
        #    whether the axis is moving — switching the brakes off at exactly
        #    the moment they are needed most.
        #
        #    ``comm_ok=False``: the frame was received, but its **content is
        #    unusable**. That matches the outcome of "let the SDK raise" — the
        #    SDK's ``guard_motion_frame`` raises ``SafetyFault`` for an invalid
        #    ``dq_act`` ("no valid feedback… motion frames forbidden"), and
        #    this layer's ``SafetyFault`` branch also fills in
        #    ``comm_ok=False``. The two paths must produce the same field
        #    value, otherwise the same event reports a different
        #    ``communication_ok`` depending on who noticed it first.
        if not (math.isfinite(position) and math.isfinite(velocity)):
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP,
                f"feedback unusable (position={position!r}, "
                f"velocity={velocity!r}) — the total torque budget cannot be "
                f"computed, refusing to send a motion frame. "
                f"★ The unknown velocity is not taken as 0: that would hand "
                f"the budget meant for damping entirely to the position term, "
                f"opening up the drive force exactly while the axis may be "
                f"moving fast.",
                comm_ok=False,
            )
            return False

        # ★★ When the measured velocity breaks the declared worst-case bound,
        #    **the safety argument has already failed**. ``velocity_bound`` is
        #    not "the largest value we observed", it is "the value we
        #    **assert** will not be exceeded": the whole budget derivation
        #    rests on ``|q̇| ≤ v_b``. Once the measurement exceeds it, that
        #    assertion is false — sending more frames would be computing
        #    torque from a premise that has been falsified. So it refuses and
        #    latches, rather than "letting this one slide".
        #    ★ Not changed to ``max(v_b, measured)``: that would let
        #      observation rewrite the assertion, the bound would drift upward
        #      with every spike, and the budget would lose its meaning along
        #      with it.
        if abs(velocity) > velocity_bound:
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP,
                f"measured velocity {abs(velocity):.4f} rad/s exceeds the "
                f"configured worst-case bound {velocity_bound} rad/s — the "
                f"safety argument of the torque budget no longer holds, "
                f"refusing to send a motion frame. Please recalibrate "
                f"hardware_max_feedback_velocity_rad_s and restart.",
                comm_ok=True,
            )
            return False

        # ★★ Rate-limited trajectory: ``q_target`` is the **destination**, and
        #    what goes out in this frame is the point the trajectory is
        #    **currently at**. There is only one rule — advance at most
        #    ``max_velocity × elapsed`` toward the target per cycle
        #    (``elapsed`` = how long it has been since the previous motion
        #    frame that was **sent successfully**).
        #
        #    ★ Why rate limiting is mandatory: without it ``q_target`` is a
        #    **step**. The output torque of the position loop is proportional
        #    to the error, so a command jumping straight from −0.81 rad to
        #    −0.01 rad would make the driver push at full error for the whole
        #    cycle — the torque budget does still cover it (that is exactly
        #    how the ``kp·e_b`` term is computed), but the mechanism slams into
        #    its target at maximum acceleration, and whatever the gripper is
        #    holding slams along with it.
        #
        #    ★★ ``max_velocity`` is **not** the velocity setpoint of the MIT
        #    frame. The ``dq`` in the frame is always ``0.0`` (see the
        #    ``send_mit_frame`` arguments below); this value is **not written
        #    into the frame at all**. Sending the speed cap as ``dq`` would
        #    mean something completely different: "charge ahead at this speed",
        #    with no destination, still pushing after the target is reached;
        #    the rate-limited trajectory means "advance at most this much per
        #    cycle, and stop once the target is reached".
        now = time.monotonic()
        seeded = False
        if not self._trajectory.started:
            # ★ First frame: only **pin the trajectory point to the measured
            #   position**, without **advancing** this cycle. The step is
            #   exactly 0 — it never jumps over from 0, and it never walks the
            #   first v×period segment right away. The first frame happens at
            #   the moment of "maximum position uncertainty" (just powered up,
            #   just received the first command), and the most conservative
            #   action then is to **send the current measured position**.
            #   The mock follows the same rule (see
            #   driver_adapter.MockAdapter.snapshot); if the two disagreed,
            #   dry_run could not validate the real robot.
            self._trajectory.set_reference(position)
            self._last_motion_tx = now
            seeded = True
        previous_reference = self._trajectory.reference_rad
        assert previous_reference is not None      # guaranteed by started
        if seeded:
            q_frame = previous_reference
        else:
            # ★★ ``elapsed`` must be **clamped to the control period** before
            #    it is multiplied.
            #
            #    Why clamp: one frame is one advance, and the normal gap
            #    between two frames is **one** control period. A longer gap
            #    means no frame went out at all in between (idle, dropped
            #    frame, thread held up), and during that time the axis was
            #    **holding still under the previous command** — it was not
            #    "travel". Without the clamp, "idle for ten seconds, then one
            #    more command" would cash those ten seconds in as a single
            #    v×10s of travel, making the first frame one huge jump — the
            #    exact thing this feature exists to eliminate.
            #
            #    ★ The clamp does **not** loosen the requirement-3 criterion:
            #    ``Δq ≤ v×min(elapsed, period) ≤ v×elapsed``. Clamping only
            #    makes it more conservative, never faster. The price is that
            #    a slowed-down loop makes the trajectory lag slightly behind
            #    the nominal rate (better slow than fast).
            elapsed = now - self._last_motion_tx
            if elapsed > self._config.period_s:
                elapsed = self._config.period_s
            q_frame = self._trajectory.advance(q_target, max_velocity, elapsed)

        kp, kd = self._gains_for_budget(
            self._config.kp,
            self._config.kd,
            torque_budget,
            error_bound,
            velocity_bound,
        )
        # ★★ Report the gains actually obtained **once per command**. See
        #    :meth:`_report_effective_gains`: this path sets no fault code, so
        #    without a log line, "command accepted + position not moving +
        #    fault_code=0" would leave nothing but guesswork.
        if target != self._reported_target:
            self._reported_target = target
            self._report_effective_gains(
                target, kp, kd, error_bound, velocity_bound, position)

        try:
            sent = gripper.send_mit_frame(q_frame, kp, kd, 0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            safety = isinstance(exc, self._safety_exc) if self._safety_exc else False
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP if safety else FaultCode.NO_FEEDBACK,
                f"send_mit_frame raised — "
                f"{'safety exception, not sent' if safety else 'non-safety exception'}"
                f": {exc!r}",
                comm_ok=not safety,
            )
            return False

        if not sent:
            # ★ A dropped frame is **not a no-op**: the driver received
            #   nothing and keeps going under the previous command. A single
            #   frame is a transient (TX buffer full); only N consecutive
            #   frames latch.
            #
            # ★★ The trajectory point must be **put back**. The ``advance``
            #    above already pushed it to ``q_frame``, yet this frame never
            #    went out — the driver is still executing the earlier command.
            #    Without the rollback, the invariant "trajectory point = the
            #    position the driver has confirmed receiving" breaks, and it
            #    breaks **with no log at all**: the trajectory would run ahead
            #    of the commands actually sent, and every later frame would
            #    carry that error. The rollback and the non-rewinding of
            #    ``_last_motion_tx`` go together: ``elapsed`` is counted from
            #    the moment the previous frame was **successfully** sent, so
            #    the dropped-frame interval gets **partially** made up — how
            #    much is decided by the period clamp above: a gap of only one
            #    period is made up in full, a longer gap only by one period's
            #    worth (that frame should not have gone farther anyway).
            self._trajectory.set_reference(previous_reference)
            self._send_failures += 1
            self._log.error(
                f"position frame not sent ({self._send_failures}/"
                f"{self._config.max_consecutive_send_failures} consecutive "
                f"failures) — frame dropped, the motor is still holding "
                f"under the previous command; trajectory point rolled back "
                f"to {previous_reference:.6f} rad"
            )
            if self._send_failures >= self._config.max_consecutive_send_failures:
                self._record_safe_stop(
                    FaultCode.HARDWARE_SAFE_STOP,
                    f"{self._send_failures} consecutive frames could not be "
                    f"sent — the link is no longer trustworthy",
                    comm_ok=True,
                )

                return False
        else:
            # ★ Only a **successful** send breaks the "consecutive failures"
            #   run — this and the comment in ``_cycle`` are two halves of the
            #   same fact. It lives here rather than "reset on any feedback"
            #   because what is being counted is send continuity, not feedback
            #   continuity.
            self._send_failures = 0
            # ★ The timestamp is updated only when the frame **went out**.
            #   It is left alone on a drop, so the next frame's ``elapsed``
            #   naturally includes the dropped interval.
            self._last_motion_tx = now
        return True

    def _send_keepalive(self, gripper: object) -> bool:
        """Keepalive frame when there is no target: ``stop()`` (zero torque,
        non-latching).

        ★ Why ``stop()`` and not "send nothing": DM motors have **poll-style**
        feedback, so with no frame going out no feedback frame ever comes back
        — which would reduce ``communication_ok`` to an unverifiable
        assertion. The ``send_zero_torque`` behind ``stop()`` is the SDK's
        "only exit for a zero-torque frame"; ``kp=kd=dq=tau=0`` disables the
        ``q`` field, so it is **exempt from the red-line position check** (it
        is equally valid when the position is outside the red line), and it
        **does not latch a fault** — see the table in the SDK for how it
        differs from ``emergency_stop()``.

        ★ Handling ``stop()`` returning False: the SDK documentation defines
        False as "**did not go out** (not connected / not enabled / dropped
        because the send buffer was full). The motor very likely still holds
        the previous torque command. This **must** be handled as a hardware
        emergency stop". Since the SDK defines it that way itself, this layer
        goes **straight** to a safe stop instead of counting a few fails
        first — a keepalive frame that cannot be sent means we may already
        be both blind and unable to control the device.

        ★★ After ``stop()`` there must also be a status-refresh frame
        (:meth:`_request_status_refresh`). ``stop()`` alone is not enough:
        the frame it sends does elicit a reply, but the SDK reads that reply
        itself and this layer's ``poll()`` never sees it. Without the refresh
        frame, this "keepalive" path could only prove that we are **still
        sending**, not that we **can still see** — and the latter is exactly
        what ``feedback_timeout_s`` exists for.
        """
        try:
            ok = gripper.stop()
        except Exception as exc:  # noqa: BLE001
            safety = isinstance(exc, self._safety_exc) if self._safety_exc else False
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP if safety else FaultCode.NO_FEEDBACK,
                f"stop() raised: {exc!r}", comm_ok=not safety)
            return False
        if ok is False:
            self._record_safe_stop(
                FaultCode.HARDWARE_SAFE_STOP,
                "keepalive stop() returned False — the SDK requires handling "
                "this as a hardware emergency stop (the motor very likely "
                "still holds the previous torque command)",
                comm_ok=False,
            )
            return False

        # ★★ The order **must not be reversed**: ``stop()`` first, then
        #    ``refresh_status()``. See :meth:`_request_status_refresh` for the
        #    reason — ``stop()`` reads away the reply to its own frame
        #    internally, so only a refresh frame issued **after** it leaves
        #    its reply for the next cycle's ``poll()``.
        self._request_status_refresh(gripper)
        return True

    def _request_status_refresh(self, gripper: object) -> None:
        """Send an extra ``0xCC`` status refresh while idle — this layer's
        "ask" half.

        ★★ Why this frame is mandatory (the root cause of the real-robot
        failure)::

            While idle, this layer sends only one ``stop()`` frame per cycle
            (the zero-torque keepalive). The driver does reply to it, **but
            that reply is read away by the SDK itself** —
            ``LiteGrip.stop()`` is implemented as::

                ok = self._can.send_zero_torque("stop")
                self._can.update_state(timeout_s=0.02)   # ← reply swallowed here
                return bool(ok)

            Meanwhile the ``poll(0.0)`` at the top of this layer's cycle is
            **non-blocking**: by the time it looks into the socket, the buffer
            has already been drained by ``update_state()``. So ``got`` is
            always ``False`` and ``_last_rx`` stops advancing — as soon as
            ``feedback_timeout_s`` (0.5 s by default) elapses, it raises
            NO_FEEDBACK.

            The real-robot symptom matches exactly: **enable succeeds → the
            node stops with NO_FEEDBACK about 0.5 s later**.

        ★ The fix is not "make ``stop()`` stop reading the reply" (that would
          change an SDK semantic many call sites rely on), but to **ask one
          more question**: nobody ever reads the reply to the ``0xCC`` refresh
          frame, so it sits quietly in the buffer waiting for this layer's
          ``poll(0.0)`` next cycle.

        ★ Why this frame is safe, and does not violate "a refresh must not be
          a motion / enable / disable interface":

        * On the wire it is 4 bytes ``[can_id_lo, can_id_hi, 0xCC, 0x00]``
          sent to the broadcast ID ``0x7FF``
          (:func:`~litegrip.can.protocol.pack_refresh_frame`) — **no
          position, no kp/kd, no feed-forward torque**; the driver just
          replies with one status frame and its output does not change by a
          single byte;
        * It goes through ``MotorController.refresh_status`` →
          ``transport.send``, **never through** ``send_mit_motion`` /
          ``send_zero_torque``, and never touches ``control_mit``;
        * It does not call ``enable()`` / ``disable()``;
        * The SDK itself uses this same interface inside ``disable()`` to ask
          for a reply on a quiet bus, noting that it "does not change the
          motor output and is independent of CTRL_MODE".

        ★ Sent once per cycle (i.e. at the ``hardware_rate_hz`` tick). **No
          new configuration knob**: the refresh tick is naturally the control
          loop tick; following it is the simplest option and the least likely
          to produce a half-broken state such as "the refresh is slower than
          control, so feedback drops out again".

        ★ How failures are handled:

        * **Returns False** (dropped because the send buffer was full) → warn
          only. What was lost is a "question", and the consequence is no
          fresh feedback this cycle — but that already **has an owner**: the
          ``feedback_timeout_s`` criterion turns "cannot see" into a safe stop
          as usual. Layering another fault on top here would only blur
          "was it the refresh or the feedback that was dropped".
        * **Raises** → latch, per this layer's convention (every SDK call in
          this file that raises latches), because an exception is not a
          comprehensible dropped frame but an unforeseen behaviour.
        """
        try:
            sent = gripper.refresh_status()
        except Exception as exc:  # noqa: BLE001
            self._record_safe_stop(
                FaultCode.NO_FEEDBACK,
                f"status refresh (0xCC) raised: {exc!r} — cannot request "
                f"fresh feedback while idle, refusing to keep waiting blind",
                comm_ok=False,
            )
            return
        if not sent:
            self._log.warn(
                "status refresh (0xCC) was not sent (send buffer full?) — no "
                "fresh feedback may arrive this cycle; if this persists, "
                "feedback_timeout_s will trigger a safe stop as usual"
            )

    def _best_effort_stop(self, gripper: object) -> None:
        """Best-effort braking on communication loss. **Failures do not
        escalate** — the bus is already down."""
        try:
            gripper.stop()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"stop() also failed after communication loss "
                            f"(expected): {exc!r}")

    def _do_safe_stop(self, gripper: object) -> None:
        """Stop action when the health check finds a problem:
        ``emergency_stop()``.

        ``emergency_stop()`` is chosen over ``stop()`` because it **disables
        first** (0xFD) and latches a fault on the SDK side — and "motor fault
        / overtemperature" should indeed stop accepting any motion command
        before a human intervenes. ``stop()`` only sends one zero-torque
        frame and leaves the motor enabled; that is far too light for the
        case of "the driver has already reported a fault".

        ⚠ **Its return value is not inspected.** The signature of
        ``LiteGrip.emergency_stop()`` is ``-> None`` (``gripper.py:579``),
        even though the body actually does ``return ok``; SDK_API_REPORT §4
        specifically warns that this layer's return value is inconsistent
        with the ``LiteGripCAN`` layer's. Writing
        ``if gripper.emergency_stop():`` is **always false** when read by the
        signature — which would silently swallow "disabling was never
        confirmed". So this call site only calls, never branches; whether the
        disable was confirmed is expressed by the ``disable()`` call and
        ``communication_ok`` below.
        """
        self._log.error(
            f"performing safe stop emergency_stop(): {self._diagnosis()}"
        )
        try:
            gripper.emergency_stop()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"emergency_stop() raised: {exc!r}")
        with self._lock:
            sample = self._sample
            # ★ This method does **not** go through
            #   :meth:`_record_safe_stop` (that one rewrites the sample's
            #   velocity/torque, whereas the fault code has already been
            #   written by :meth:`_update_from_state`), so the latch must be
            #   set separately here — otherwise the "stop due to motor fault"
            #   path would be the only fault path that still lets commands
            #   into the slot.
            self._latched = True
            # The emergency stop has cut the output → no longer claim any
            # velocity/torque. The position keeps its last known value
            # (consistent with MockAdapter's fault semantics: frozen, not
            # zeroed).
            self._sample = Sample(
                position_rad=sample.position_rad,
                velocity_rad_s=0.0,
                torque_nm=0.0,
                error=True,
                stopped=True,
                enabled=False,
                communication_ok=sample.communication_ok,
                fault_code=self._fault,
            )

    def _do_emergency_stop(self, gripper: object) -> None:
        """Emergency stop requested from the ROS side. Same action as
        :meth:`_do_safe_stop`, different cause."""
        self._log.warn("performing the emergency stop requested by ROS")
        try:
            gripper.emergency_stop()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"emergency_stop() raised: {exc!r}")
        with self._lock:
            sample = self._sample
        self._record_safe_stop(
            FAULT_EMERGENCY_STOP,
            "emergency stop requested by ROS: emergency_stop() was executed",
            comm_ok=sample.communication_ok,
        )

    def _teardown(self, gripper: Optional[object], enabled: bool) -> None:
        """Thread teardown. **Runs exactly once per _run**, so disable and
        disconnect each happen once."""
        if gripper is None:
            return
        if enabled:
            try:
                gripper.disable()
            except Exception as exc:  # noqa: BLE001
                self._log.error(f"teardown disable() failed: {exc!r}")
        try:
            if bool(getattr(gripper, "is_connected", False)):
                gripper.disconnect()
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"teardown disconnect() failed: {exc!r}")
        self._log.info("hardware path torn down (disable + disconnect)")

    # ── State mapping ───────────────────────────────────────────────
    def _update_from_state(self, state: object) -> None:
        """Map one ``GripperState`` frame into a :class:`Sample`.

        Not a single field semantic may be substituted (see
        ``msg/GripperState.msg`` and the module docstring of
        ``driver_adapter``):

        * ``enabled`` comes from ``is_enabled`` — which is equivalent to
          ``error_code == 1``;
        * ``error`` **never** comes from ``error_code``. The SDK's ``1`` means
          "enabled", and stuffing it into ``error`` would turn a normal
          enable into a reported error. Normalization goes through
          :func:`~litegrip_ros2_control.driver_adapter.motor_fault_code`.
        """
        position = float(state.position_rad)
        velocity = float(state.velocity_rad_s)
        torque = float(state.torque_nm)
        error_code = int(state.error_code)
        enabled = bool(getattr(state, "is_enabled", error_code == 1))

        fault = motor_fault_code(error_code)
        detail = f"sampling normal (error_code=0x{error_code:X})"
        # ★ Temperatures are read only once: they feed both the overtemperature
        #   criterion and the reporting (the two temperature fields of Sample).
        #   The original implementation read them here and threw them away
        #   after the check, so the shared-memory temperatures were always 0 —
        #   a diagnostic that is always 0 is worse than none (whoever reads it
        #   will believe the temperature really is fine).
        temps = self._read_temperatures(state)
        if fault == 0:
            if temps is not None:
                t_mos, t_coil = temps
                limit_c = self._config.temperature_limit_c
                if t_mos >= limit_c:
                    fault = FAULT_OVERTEMP_MOS
                    detail = f"MOS temperature {t_mos}℃ ≥ limit {limit_c}℃"
                elif t_coil >= limit_c:
                    fault = FAULT_OVERTEMP_COIL
                    detail = f"coil temperature {t_coil}℃ ≥ limit {limit_c}℃"
        else:
            detail = f"motor reports error error_code=0x{error_code:X}"
            # An error report cannot coexist with the enabled state
            # (is_enabled is exactly error_code == 1).
            enabled = False

        if fault == 0 and not enabled:
            # Not enabled, but not a fault either — for example the driver's
            # own watchdog timed out and dropped the enable. This is not "the
            # motor is broken", but the higher layer **must** see it: this
            # layer believes it is driving an enabled motor, when in fact the
            # motor is already out of control. NO_FEEDBACK would be the wrong
            # bucket (communication is fine), so UNSAFE_INITIAL_STATE is used
            # to express "the current state does not allow further driving".
            fault = int(FaultCode.UNSAFE_INITIAL_STATE)
            detail = (
                f"motor is no longer enabled (error_code=0x{error_code:X}) — "
                "the output may have been cut by the driver's own watchdog"
            )

        sample = Sample(
            position_rad=position,
            velocity_rad_s=velocity,
            torque_nm=torque,
            error=(fault != 0),
            stopped=abs(velocity) < self._config.stopped_eps_rad_s,
            enabled=enabled,
            communication_ok=True,
            fault_code=int(fault),
            # An unreadable temperature is reported as 0 — but that means
            # "unknown", not "cool", so it is only valid when read together
            # with communication_ok / fault_code.
            temperature_mos_c=float(temps[0]) if temps is not None else 0.0,
            temperature_coil_c=float(temps[1]) if temps is not None else 0.0,
            error_code=error_code,
        )
        with self._lock:
            self._fault = int(fault)
            self._detail = detail
            self._sample = sample

    def _read_temperatures(self, state: object) -> Optional[Tuple[int, int]]:
        """Read the ``(MOS, coil)`` temperatures. Returns ``None`` when they
        cannot be read.

        Unreadable is **not treated as 0℃**: taking "unknown" for "cool" is
        the most classic false-negative pattern. When this returns ``None``
        the caller skips the temperature criterion and lets the other
        criteria cover for it.
        """
        try:
            return int(state.temperature_mos), int(state.temperature_coil)
        except Exception:  # noqa: BLE001 - missing field or bad type = unreadable
            return None

    def _missing_bounds_detail(self) -> str:
        """The sentence written to the status and the log when a bound is
        missing. **It must name which one.**

        ★ Naming it is mandatory: a bare "a parameter is missing" gives no
        clue what to do next. This lists, item by item, what is missing, what
        happens without it, and why this layer will not invent a default for
        it — those three things are the only useful information when a human
        is troubleshooting.
        """
        missing = []
        if self._config.max_feedback_velocity_rad_s is None:
            missing.append(
                "max_feedback_velocity_rad_s (worst-case feedback velocity "
                "bound, rad/s) — the SDK only checks the measured dq_act for "
                "finiteness, with no magnitude cap of any kind; the range "
                "constant 30.0 is the encoding range of a 12-bit field, not a "
                "protection threshold, and the 0.5 of the recovery channel "
                "belongs to a different channel. This layer **will not invent "
                "a value for it**"
            )
        reason = "; ".join(missing) if missing else "non-positive or non-finite bound"
        return (
            f"missing worst-case bounds, refusing to send motion frames: "
            f"{reason}. Without a velocity bound there is no way to state "
            f"'total torque over the whole control period ≤ budget', so the "
            f"real-hardware path stays stopped until that value is calibrated. "
            f"After calibration, state it explicitly as "
            f"hardware_max_feedback_velocity_rad_s."
        )

    def _worst_case_bounds(self) -> Optional[Tuple[float, float]]:
        """Obtain ``(error_bound, velocity_bound)``. **Returns ``None`` when
        they cannot be obtained.**

        :meth:`_send_motion` fails closed on ``None`` in every case (it will
        not invent a motion frame). Splitting the two into "derivable" and
        "not derivable" is the only honest thing this layer can do here:

        * **Position error bound** — derivable. The SDK's
          ``guard_motion_frame`` forces ``q_target`` and the **measured**
          ``q_act`` to **both** lie inside the red line
          (``require_act_within_red`` in ``safety_limits.py``), therefore
          ``|q_target − q_act| ≤ red_max_rad − red_min_rad``. This is a hard
          bound derived from the **red line in force**, not an estimate. When
          the configuration supplies a tighter value, the tighter one wins
          (**it can only be tightened**).
        * **Feedback velocity bound** — **not derivable**. See the comment on
          :attr:`HardwareConfig.max_feedback_velocity_rad_s`: the SDK only
          checks the measured ``dq_act`` for finiteness, the range constant
          30.0 is an encoding range rather than a protection threshold, and
          the 0.5 of the recovery channel belongs to a different channel. No
          configured value ⇒ ``None`` ⇒ fail-closed.
        """
        limits = self._limits
        if limits is None:                       # guaranteed at construction, defensive
            return None

        width = float(limits.red_max_rad) - float(limits.red_min_rad)
        if not math.isfinite(width) or width <= 0.0:
            return None
        configured_error = self._config.max_position_error_rad
        error_bound = width if configured_error is None \
            else min(float(configured_error), width)

        velocity_bound = self._config.max_feedback_velocity_rad_s
        if velocity_bound is None:
            return None
        velocity_bound = float(velocity_bound)
        if not math.isfinite(velocity_bound) or velocity_bound <= 0.0:
            return None
        if not math.isfinite(error_bound) or error_bound <= 0.0:
            return None
        return error_bound, velocity_bound

    @staticmethod
    def _gains_for_budget(
        kp_config: float,
        kd_config: float,
        torque_budget_nm: float,
        error_bound: float,
        velocity_bound: float,
    ) -> Tuple[float, float]:
        """Allocate the **total torque budget** between the position term
        and the damping term, returning ``(kp, kd)``.

        This version computes the **worst-case budget**, not this frame's budget
        ---------------------------------------------------------------------------
        Stage 5.2 scaled the gains by the **current sample's** ``|Δq|`` /
        ``|q̇|``. That is not sufficient, because the ``kp`` / ``kd`` inside a
        frame are handed to the **driver**: it uses those two numbers to
        **continuously** evaluate ``kp·(q_target − q_act) + kd·(0 − q̇)``,
        where ``q_act`` is what the driver **measures itself in real time**.
        Within one control period ``period_s`` (50 Hz → 20 ms), both the error
        and the velocity can differ from what they were at the sampling
        instant.

        ``e = 0`` is the clearest counterexample: at the sampling instant the
        position term really does spend no budget, but a millisecond later the
        driver multiplies the **same** ``kp`` by a non-zero error. So the
        scaling must use the **worst values** over that window::

            e_b = error_bound       worst-case position error (non-zero — requirement 6)
            v_b = velocity_bound    worst-case feedback velocity

        Constraint and allocation
        -------------------------
        With ``tau_ff = 0`` and ``dq_target = 0`` the output torque is
        ``τ = kp·Δq − kd·q̇``. By the triangle inequality, a **sign-agnostic**
        sufficient condition (that is, a conservative one) is::

            kp·e_b + kd·v_b ≤ budget          ★ worst-case total-budget criterion

        ::

            kd = min(kd_config, budget / v_b)         ← ① damping loaded first
            remaining = budget − kd·v_b               ← ② what is left
            kp = min(kp_config, remaining / e_b)      ← ③ position term takes the rest

        Hence ``kp·e_b ≤ remaining`` and ``kd·v_b ≤ budget``, and the two sum
        **exactly** back within the budget. Three corollaries:

        * When the damping term **on its own** fills the whole budget
          (``kd_config·v_b ≥ budget``), ``remaining = 0`` ⇒ ``kp = 0`` — only
          braking, no drive. When the axis may be moving fast, **the position
          loop yields to the damping loop**;
        * When not even ``kd_config`` fits (``budget/v_b < kd_config``), ``kd``
          itself is reduced as well — the budget outranks any individual gain;
        * Both ``kp`` and ``kd`` are **independent of the current sample**, so
          they are constants identical from frame to frame. This is not a
          defect but the point: a bound that must cover the whole control
          period cannot wobble with the sampling.

        ★ **Why the ``e = 0`` branch of Stage 5.2 cannot be carried over.**
        That version specified "when ``e = 0``, ``kp`` takes ``kp_config``",
        with the rationale that "the position term consumes no budget this
        frame". That rationale holds under the **current-cycle** semantics but
        not under the **worst-case** semantics: taking ``kp_config = 20.0`` in
        full together with ``e_b = 1.23`` means the position term **can**
        produce 24.6 N·m — far above the 3.5 limit. Requirement 6 says exactly
        this: even with ``e = 0`` at the current sample, the scaling must use a
        non-zero ``e_b``. The requirements of the two versions conflict
        outright; this one follows Stage 5.3 and keeps this record.

        A ``velocity_bound`` of 0 would in theory mean ``budget/0``; but
        :meth:`_worst_case_bounds` rejects a non-positive ``v_b``
        (fail-closed), so execution never gets there —
        :meth:`_largest_gain_within`'s ``rate <= 0`` branch is only a
        backstop.

        ★ This is **not** a substitute for the SDK's own judgement. The SDK's
        ``guard_motion_frame`` uses the **measured** torque ``tau_act`` as the
        authoritative criterion (``kp·Δq`` is a poor estimate: starting from
        the far end, the position error is the entire travel). Here the
        **command itself** is kept within budget for the whole control period;
        the two are complementary rather than duplicative.
        """
        budget = float(torque_budget_nm)

        # ★★ Both call sites use **keyword arguments** without exception.
        #    The two calls have identical signatures and differ only in the
        #    arguments, so swapping them would be a change that raises nothing
        #    and merely yields silently wrong gains — keywords turn it into
        #    something you can see at a glance in the diff, and they pin down
        #    the non-crossing rule that "kp eats only kp_config, kd eats only
        #    kd_config".
        kd = SdkHardwareAdapter._largest_gain_within(
            desired=kd_config, limit=budget, rate=velocity_bound)
        # ★ The remainder is computed from the share the damping term
        #   **actually** ate (``kd·v_b``), not estimated as
        #   ``min(kd_config·v_b, budget)`` — the former is what really has to
        #   be subtracted.
        remaining = budget - kd * velocity_bound
        kp = SdkHardwareAdapter._largest_gain_within(
            desired=kp_config, limit=remaining, rate=error_bound)
        return kp, kd

    @staticmethod
    def _largest_gain_within(desired: float, limit: float, rate: float) -> float:
        """Take the largest gain satisfying ``gain · rate ≤ limit``.

        :param desired: the gain from the configuration (the value it would
            have with no budget compression).
        :param limit: the torque allowance available to this term (N·m),
            which must be ≥ 0.
        :param rate: how much torque one unit of gain produces — ``|Δq|`` for
            the position term, ``|q̇|`` for the damping term.

        ★ **Floating-point note.** The pure-mathematical solution is
        ``limit / rate``, but the floating-point division multiplied back can
        exceed ``limit`` by one ulp. No compensation is added for that here:
        the torque field of the MIT frame is 12-bit fixed point
        (``dm4310-mit-quantization``: mapped onto a ±10 N·m scale), so one
        code is about ``4.9e-3 N·m`` — **thirteen orders of magnitude** larger
        than the ulp noise (about ``1e-16 N·m``). That noise cannot even cross
        one quantization step, let alone become real torque. So the criterion
        is written to hold "in the real-number sense", the tests use a
        tolerance of the same magnitude, and nothing chases ulps in the code.
        """
        desired = float(desired)
        rate = float(rate)
        limit = max(float(limit), 0.0)   # subtraction can yield -0.0 or a tiny negative
        if not math.isfinite(rate):
            # Backstop: an invalid velocity reading gets **no** gain
            # (fail-closed). The normal path never gets here — _send_motion
            # already intercepts it first.
            return 0.0
        if rate <= 0.0:
            # This term cannot produce torque (error/velocity is 0), so it
            # spends no budget → take the configured value.
            return desired
        return min(desired, limit / rate)

    def _report_effective_gains(
        self,
        target: Tuple[float, float, float],
        kp: float,
        kd: float,
        error_bound: float,
        velocity_bound: float,
        position: float,
    ) -> None:
        """Report once the stiffness/damping this command **actually got**.
        **One report per command.**

        ★★ Why this line must exist
        ---------------------------
        When the budget falls short, :meth:`_gains_for_budget` compresses
        ``kp`` very small — in the extreme, all the way to ``0``. And this
        path **produces no fault code at all**: the frame is legal, it goes
        out, the driver receives it, bus communication is fine. So the
        operator sees a set of mutually contradictory observations —

            "command accepted / no fault at all / position not moving"

        — while the one number that would explain them all at once (how
        stiff this command actually is) **is not printed anywhere**. This is
        exactly what was reported on the real robot on 2026-09-22: the
        command ``target=-0.60 rad, torque_limit=1.0 N·m`` was accepted, and
        some 2 minutes later the position was still at ``-0.807 rad`` with a
        feedback torque of ``0.06~0.07 N·m``. That is exactly ``kp × error``
        (``0.309 × 0.207``) — the position loop was pushing, just **too
        weakly** to move the mechanism. For the offline reproduction see
        ``test_a_stuck_axis_still_gets_the_full_trajectory``.

        What is reported, and why those two numbers
        -------------------------------------------
        * The **ratio** of ``kp`` to ``hardware_kp`` — so that a stiffness
          that has dropped by an order of magnitude is obvious at a glance,
          rather than "the budget is a bit tight";
        * The torque the position term would produce **when the axis does not
          move at all**: ``kp × this command's travel`` — the only number the
          operator can directly compare against "the gripper's own
          resistance": **if it is smaller than that, the axis certainly will
          not move**.
          ★ Using **travel** rather than the **current error**: this command
          is reported only here, and this single report happens to land on
          the **first frame** — whose reference point is pinned to the
          measured position, so the current error is identically 0 and any
          "maximum output" computed from it would always be 0.0000, exactly
          inverting the very thing that needed reporting. Travel does not
          depend on which frame the report lands on: while the axis stays
          put, the error grows monotonically with the advancing reference
          point until it reaches the full travel, so ``kp × travel`` is this
          command's steady-state output, and also its ceiling in the worst
          case of "the axis does not move".

        How the level is chosen (one line per command, no 50 Hz spam)
        -------------------------------------------------------------
        * ``error`` — ``kp`` was compressed to **0** and there is **still
          travel to cover**: the frame that goes out has nothing but damping
          (braking), the position loop contributes no force at all, and this
          command **will not move no matter how long you wait**. A command
          that has already arrived does not count (the position term is not
          needed there);
        * ``warn`` — ``kp`` is below half the configured value;
        * ``info`` — everything else. Still reported, just not shouted: even
          when the compression is mild, "was the configured kp actually sent
          out?" is a question only this line can answer.
        """
        q_target, torque_budget, _ = target
        kp_config = float(self._config.kp)
        kd_config = float(self._config.kd)
        travel = abs(q_target - position)
        stiffness_torque = abs(kp * travel)
        ratio = kp / kp_config if kp_config > 0.0 else 1.0

        head = (
            f"actual control gains for this command: kp={kp:.6f} N·m/rad"
            f"(configured hardware_kp={kp_config}, compressed to {ratio:.2%} "
            f"by the torque budget), kd={kd:.4f} (configured {kd_config}). "
            f"★ On this command's travel ({travel:.4f} rad) the position term "
            f"produces **at most** {stiffness_torque:.4f} N·m — while the "
            f"axis does not move, the error grows to fill the whole travel, "
            f"and that is its steady-state output. "
        )
        why = (
            f"the allocation rule is kp·e_b + kd·v_b ≤ budget: the budget "
            f"here is {torque_budget} N·m, e_b={error_bound} rad, "
            f"v_b={velocity_bound} rad/s, of which the damping term alone "
            f"takes {kd * velocity_bound:.4f} N·m. "
        )
        ceiling = None if self._limits is None else \
            float(self._limits.torque_limit_nm)
        fix = (
            f"★ If that torque above is smaller than the gripper's own "
            f"resistance, the axis will **stay put**, and it **will not "
            f"report any fault code** (the frame is legal, communication is "
            f"fine) — that is not a fault, the budget is simply not enough. "
            f"To get more position stiffness: raise torque_limit_nm in the "
            f"command"
            + (f" (effective ceiling this time: {ceiling} N·m)"
               if ceiling is not None else "")
            + f", or reduce hardware_kd / hardware_max_feedback_velocity_rad_s"
            f" (the damping term is exactly kd × v_b). "
        )

        if kp <= 0.0 and travel > _COMMAND_TRAVEL_EPS_RAD:
            self._log.error(
                head + why
                + "★ The kp of this command was compressed to **0** — the "
                "frame that goes out has nothing but damping (braking), the "
                "position loop contributes no force at all, and **this "
                "command will not move no matter how long you wait**." + fix
            )
        elif ratio < 0.5:
            self._log.warn(head + why + fix)
        else:
            self._log.info(head + why)

    def _last_motion_state(self) -> Tuple[float, float]:
        """Take the position and velocity of **the same frame** under one
        lock, returning ``(position, velocity)``.

        ★ They must come from the same frame: two separate reads would yield
        samples from two different cycles, and computing a budget from "the
        position of frame A paired with the velocity of frame B" produces a
        bound that holds for **neither frame**. Both fields come from the
        same immutable :class:`Sample`, so reading them in one go is
        self-consistent.
        """
        with self._lock:
            sample = self._sample
        return sample.position_rad, sample.velocity_rad_s

    def _take_target(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            return self._pending

    def _estop_is_pending(self) -> bool:
        with self._lock:
            return self._estop_requested

    def _current_fault(self) -> int:
        with self._lock:
            return self._fault

    def _diagnosis(self) -> str:
        with self._lock:
            return self._detail

    # ── Recording a safe stop ───────────────────────────────────────
    def _record_safe_stop(
        self,
        fault: int,
        detail: str,
        *,
        comm_ok: bool,
        position_rad: Optional[float] = None,
    ) -> None:
        """Record one safe stop. **Once called, this adapter layer never
        recovers on its own.**

        The three ``Sample`` fields are filled per the "safe stop convention"
        (requirement 13):

        * ``stopped=True`` — the bridging layer has stopped advancing motion;
        * ``enabled=False`` — no longer claim the motor is enabled (especially
          not when the feedback cannot confirm it; :class:`Sample`'s
          construction-time invariants also block that);
        * ``communication_ok`` is filled with the value **actually** passed
          in;
        * ``fault_code != 0``, hence ``error=True`` (guaranteed by ``Sample``).

        ``position_rad`` defaults to keeping the last known value —
        consistent with ``MockAdapter``'s fault semantics (frozen at the last
        known position before the fault, not zeroed). Zero would be
        misread as "the axis returned to 0", and 0 is a legal position that
        may be very far away.
        """
        with self._lock:
            previous = self._sample
            self._fault = int(fault)
            self._detail = detail
            # ★ Set **under the same lock acquisition** as the fault code —
            #   otherwise a window appears where "the fault code has already
            #   been published, yet commands can still enter the slot", and
            #   that is exactly the false report audit point 7 sets out to
            #   eliminate.
            self._latched = True
            self._sample = Sample(
                position_rad=(
                    previous.position_rad if position_rad is None
                    else float(position_rad)
                ),
                velocity_rad_s=0.0,
                torque_nm=0.0,
                error=True,
                stopped=True,
                enabled=False,
                communication_ok=bool(comm_ok),
                fault_code=int(fault),
            )
        self._log.error(
            f"safe stop latched (fault_code={int(fault)}): {detail}")
