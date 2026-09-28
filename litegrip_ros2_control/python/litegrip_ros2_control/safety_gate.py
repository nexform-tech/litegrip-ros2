"""safety_gate — the last gate before a frame is sent.

Purpose
-------
The LiteGrip SDK ships its own red line system (``litegrip.safety_limits``;
tighten only, never loosen). This gate **neither replaces it nor loosens
it** — it is one more layer stacked on top of the SDK, the "should the ROS 2
side send this frame at all" decision, covering what the SDK cannot.

This module is **pure logic**: it does not import rclpy, does not import
litegrip, and opens no CAN socket. The limits are passed in by the caller as
:class:`GateLimits` — that way it can be unit-tested offline and it is
guaranteed that "the red lines have exactly one source of truth, the SDK"
(this module contains no hard-coded red line).

Stage 3 status
--------------
Only **command validation** is implemented. The real send gate (throttling,
concurrency, watchdog, authorization freshness) is only needed for the
real-hardware path (Stage 4); the contracts not yet implemented are listed at
the bottom of this file.

Staying consistent with the SDK's boundary conventions
------------------------------------------------------
* **Red lines are closed intervals**: ``safety_limits.py:1145`` states
  explicitly that when ``q_act`` is exactly ``red_min_rad`` or
  ``red_max_rad`` it is **not rejected**, and the criterion at ``:1258`` is
  ``q < red_min or q > red_max``. This gate uses the same criterion — being
  off by one endpoint is a silent disagreement, and such bugs only surface on
  real hardware.
* **Reject, do not clamp**: the SDK's ``send_mit_motion`` raises
  ``LimitViolation`` / ``SafetyFault`` when out of range; it does not
  silently return False, and certainly does not "fix" the value and send it.
  This gate likewise only makes the pass/reject choice.
* **Strict numeric validation**: the SDK's ``normalize_scalar`` rejects NaN /
  ±inf / non-numeric values, and **explicitly rejects bool** (in Python
  ``bool`` is a subclass of ``int``, so ``True`` would pass itself off as
  ``1.0``). This gate does the same.
* **The two ceilings have their own hard limits and are unrelated to each
  other**: ``TORQUE_LIMIT_CEILING_NM`` governs the torque budget, while
  ``MAX_COMMAND_VELOCITY_CEILING_RAD_S`` governs the **command trajectory**
  velocity. ★ The latter is **not** ``max_feedback_velocity_rad_s`` in
  ``HardwareConfig`` (the worst-case bound on measured velocity) — this module
  cannot see that quantity at all, and should not: this module is pure logic,
  it imports no SDK and touches no adapter configuration. Mixing the two
  turns "I assert the axis cannot turn that fast" into "I command it to turn
  that fast".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional

__all__ = [
    "TORQUE_LIMIT_CEILING_NM",
    "MAX_COMMAND_VELOCITY_CEILING_RAD_S",
    "MOTOR_FAULT_BASE",
    "FaultCode",
    "GateLimits",
    "RejectReason",
    "GateDecision",
    "check_command",
]


#: ROS 2 side torque hard ceiling, unit N·m.
#:
#: This is a **read-only ceiling**: :class:`GateLimits` refuses to construct a
#: wider value, and node parameters can only make it **smaller**, never
#: larger. It is the same discipline as the SDK's "red lines may only be
#: tightened, never loosened".
#:
#: ══════════════════════════════════════════════════════════════════════
#: 2026-09-21 baseline change: 0.25 → 3.5 (N·m)
#: ══════════════════════════════════════════════════════════════════════
#:
#: **Basis**: ``/home/yd/下载/DM-J4310-2EC V1.2 减速电机使用说明书 V1.3
#: 2026-09-08.pdf`` (SHA256 ``c2600a1f…85e65bcd``), page 6, the
#: "characteristic parameters" table — **rated torque 3.5 N·m (output side)**,
#: peak torque 12.5 N·m, gear ratio 10:1. The **rated** value is used rather
#: than the peak: the peak would need duty-cycle and temperature-rise
#: constraints, and this project has no data of that kind. The full evidence
#: write-up (SHA256, how "output side" was determined, why the 10 N·m mapped
#: range is not used) lives in ``safety_limits.PACKAGE_TAU_MAX_NM`` on the SDK
#: side — it is not copied here; a second copy would inevitably drift.
#:
#: ★ **The relationship between this quantity and the SDK has changed, and
#: that has to be stated clearly.** The old value 0.25 was "tighten once more
#: within the SDK's 1.0 N·m", taking its legitimacy from being **tighter than
#: the SDK**; the new value 3.5 is **equal** to the SDK baseline, and is no
#: longer tighter than it. In other words, this constant is now an
#: **independent mirror of the same evidence**, not an extra tightening. Its
#: legitimacy comes from that evidence itself, not from "being smaller than
#: the other one".
#:
#: The effect of the two being equal is guaranteed by the three-way minimum in
#: :func:`driver_adapter.read_limits_from_sdk`,
#: ``min(requested value, SDK tau_max_nm, this constant)`` — whichever layer
#: is lowered, the effective value follows it down. **Lowering any one layer
#: takes effect immediately; raising any one has no effect.**
#:
#: ⚠ The loosening direction (0.25 → 3.5) must be backed by evidence, and the
#: evidence is above. Rollback path: switch the node's ``safety_baseline``
#: parameter to ``"0.25"``; this constant does not need to change.
TORQUE_LIMIT_CEILING_NM = 3.5


#: ROS 2 side **command trajectory velocity** hard ceiling, unit rad/s.
#:
#: Same discipline as :data:`TORQUE_LIMIT_CEILING_NM`: a **read-only
#: ceiling**; :class:`GateLimits` refuses to construct a faster value, and
#: node parameters can only make it **smaller**.
#:
#: ══════════════════════════════════════════════════════════════════════
#: Basis: the SDK's own "stopping distance" velocity model
#: ``SafetyLimits.v_allow`` (``sdk/litegrip/safety_limits.py:1112``)
#: ══════════════════════════════════════════════════════════════════════
#:
#: That docstring writes the model as::
#:
#:     stopping distance = v²/(2·a) + v·t_comm + reserve  ≤  limit
#:     v_allow  = −a·t_comm + sqrt( (a·t_comm)² + 2a·(limit − reserve) )
#:
#: where ``limit = min(the margin on that side, the distance from the current
#: position to the red line on that side)``. **The farther from the red line,
#: the faster it may go**, and the ceiling is the value determined by that
#: "margin" term:
#:
#:   · open-side margin 0.032793 rad → **0.436591… rad/s**
#:   · closed-side margin 0.061308 rad → 0.657020… rad/s
#:
#: The SDK's own docstring goes on to say "**the smaller one is the global
#: v_max**" — this constant is that smaller one, **rounded down to 3 decimal
#: places** (rounding may only be stricter, never wider):
#: ``floor(0.436591… × 1000) / 1000 = 0.436``.
#:
#: ★ Why use the SDK's model rather than picking another number: this model
#:   answers exactly "how fast may the command trajectory advance while the
#:   stopping distance still lands inside the margin outside the red line" —
#:   which **corresponds word for word** to the semantics of
#:   `max_velocity_rad_s`. The red line span is 1.23 rad, and 0.436 rad/s
#:   covers it in about 2.8 s, which matches how a gripper feels.
#:
#: ★ A literal is written here rather than importing the SDK: module level
#:   must not import ``litegrip`` (this module is pure logic). One test
#:   computes ``v_allow`` from the **real SDK** on the fly to pin this number
#:   down — the same treatment as ``FAULT_OVERTEMP_MOS`` (write a literal +
#:   pin it with the true value), so any change to the baseline or to
#:   ``a_max_rad_s2`` / ``t_comm_s`` turns the test red at once.
#:
#: ⚠ The ``a=5.0 rad/s²`` and ``t_comm=0.02 s`` in this model are marked in
#:   the SDK as **provisional experimental parameters not yet measured on
#:   real hardware**. If they are optimistic, the safe speed on real hardware
#:   will be lower than this constant — so this constant is **a ceiling, not
#:   a target**: the real-hardware path should use the node parameter
#:   ``max_velocity_rad_s`` to press it down to a value confirmed by
#:   measurement.
#:
#: ⚠ The loosening direction must be backed by new evidence. Rollback path:
#:   lower the node parameter ``max_velocity_rad_s``; this constant does not
#:   need to change.
MAX_COMMAND_VELOCITY_CEILING_RAD_S = 0.436


class FaultCode(IntEnum):
    """**Bridge-layer** fault codes. The values match ``fault_code`` in
    ``msg/GripperState.msg``.

    Note that this is **a different thing** from the ``error_code`` in SDK
    feedback: ``error_code`` is a motor status word (where 1 means enabled),
    while this enum is the bridge layer's own faults.

    ★ Motor faults occupy **another segment** (``> MOTOR_FAULT_BASE``), see
    :data:`MOTOR_FAULT_BASE`. The two segments are separate because, mixed
    into one, motor error 8 and bridge-layer 8 would collide on the same
    number and the subscriber would have no way to tell them apart.
    """

    NONE = 0
    #: The command was rejected by this gate (out of range / non-finite /
    #: torque above the ceiling / no usable limits).
    COMMAND_REJECTED = 1
    #: No feedback available: communication loss / SDK not importable /
    #: an exception during sampling.
    NO_FEEDBACK = 2
    #: Internal bridge-layer error.
    INTERNAL = 3
    #: ★ The real-hardware adapter layer **refuses to enter the enabled
    #: state**: the initial state check failed (position outside the red
    #: lines / temperature over the limit / motor error / no feedback).
    #:
    #: Why this is separate from ``INTERNAL``: ``INTERNAL`` means "the bridge
    #: layer itself has a bug", whereas this one means "the bridge layer is
    #: working correctly; the hardware's current state does not permit
    #: enabling". Mixing the two would make a **correct** refusal look like a
    #: code defect, and operations would go looking at the code instead of at
    #: the mechanism.
    UNSAFE_INITIAL_STATE = 4
    #: ★ The real-hardware adapter layer **has executed a safe stop** and
    #: latched it (emergency stop / feedback out of range / repeated frame
    #: send failures / overtemperature / motor fault / communication loss).
    #:
    #: ★★ The difference from ``COMMAND_REJECTED`` is "persistent vs
    #: transient": this one **is only ever written by the background thread's
    #: observations**, and the command path can neither write nor clear it;
    #: ``COMMAND_REJECTED`` is the event "the previous command was invalid"
    #: and clears as soon as the next valid command arrives.
    HARDWARE_SAFE_STOP = 5


#: Start of the motor fault code segment. Motor faults are reported as
#: ``MOTOR_FAULT_BASE + the raw error_code``.
#:
#: 100 rather than 4 was chosen to leave room for the bridge-layer code
#: segment — adding bridge-layer fault codes later will not risk colliding
#: with the motor segment. The raw code can be recovered losslessly:
#: ``raw code = fault_code - 100``.
MOTOR_FAULT_BASE = 100


class RejectReason(str, Enum):
    """Rejection reasons. **Every rejection must leave a diagnosable
    reason**, not just False.
    """

    ACCEPTED = "accepted"
    #: Limits unavailable — reject by default (fail-closed).
    NO_LIMITS = "no_limits"
    #: Not a finite real number (NaN / ±inf / non-numeric / bool).
    NOT_FINITE = "not_finite"
    #: The target position is outside the red lines.
    POSITION_OUT_OF_RANGE = "position_out_of_range"
    #: The torque limit is not positive.
    TORQUE_NOT_POSITIVE = "torque_not_positive"
    #: The torque limit exceeds the hard ceiling.
    TORQUE_ABOVE_CEILING = "torque_above_ceiling"
    #: ★ The trajectory velocity limit is not positive (including 0 — 0 means
    #: "never reaches the target", which is not "slow", it is "this command
    #: contradicts itself").
    VELOCITY_NOT_POSITIVE = "velocity_not_positive"
    #: ★ The trajectory velocity limit exceeds the limit in effect this time.
    VELOCITY_ABOVE_CEILING = "velocity_above_ceiling"
    #: The limits that were passed in are themselves invalid (they should have
    #: been rejected at construction; this is a programming error).
    BAD_LIMITS = "bad_limits"


@dataclass(frozen=True)
class GateLimits:
    """The set of limits the gate uses. **May only be tighter than the SDK,
    never wider.**

    :param red_min_rad: the open side of the red lines (the smaller end of
        the range).
    :param red_max_rad: the closed side of the red lines (the larger end of
        the range).
    :param torque_limit_nm: torque limit, ``0 < x <= TORQUE_LIMIT_CEILING_NM``.
    :param max_velocity_rad_s: **command trajectory** velocity limit (rad/s),
        ``0 < x <= MAX_COMMAND_VELOCITY_CEILING_RAD_S``.
        ★ This is "how far the target position may advance per second"; it is
        **not** the velocity setpoint in an MIT frame, and **not**
        ``HardwareConfig.max_feedback_velocity_rad_s`` (the worst-case bound on
        measured velocity) — the semantics of those two quantities are in
        ``GripperCommand.msg``.
    :param source: where this set of limits came from; used for diagnostic
        output only.
    """

    red_min_rad: float
    red_max_rad: float
    torque_limit_nm: float
    max_velocity_rad_s: float
    source: str = "unspecified"

    def __post_init__(self) -> None:
        for name in (
            "red_min_rad", "red_max_rad", "torque_limit_nm",
            "max_velocity_rad_s",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"GateLimits.{name} is not a number: {value!r}")
            if not math.isfinite(value):
                raise ValueError(
                    f"GateLimits.{name} is not a finite number: {value!r}")

        if not self.red_min_rad < self.red_max_rad:
            raise ValueError(
                f"GateLimits red line interval is empty or reversed: "
                f"[{self.red_min_rad}, {self.red_max_rad}]"
            )

        # ★ Tighten only: limits wider than the hard ceiling **cannot be
        #   constructed**. This lives at construction rather than at the check
        #   site so that "loosening" is impossible at the type level, instead
        #   of relying on every call site to remember to compare.
        if self.torque_limit_nm <= 0.0:
            raise ValueError(
                f"GateLimits.torque_limit_nm must be positive: "
                f"{self.torque_limit_nm!r}"
            )
        if self.torque_limit_nm > TORQUE_LIMIT_CEILING_NM:
            raise ValueError(
                f"GateLimits.torque_limit_nm={self.torque_limit_nm} exceeds "
                f"the hard ceiling {TORQUE_LIMIT_CEILING_NM} N·m — the torque "
                f"limit may only be tightened, never loosened."
            )

        # ★ The trajectory velocity limit follows the **same** "tighten only"
        #   discipline, word for word symmetric.
        #   The two are written out side by side rather than factored into a
        #   loop: each error message has to name its own quantity and unit
        #   (N·m / rad/s), and factoring it out would mean passing a pile of
        #   message arguments, which is actually harder to read.
        if self.max_velocity_rad_s <= 0.0:
            raise ValueError(
                "GateLimits.max_velocity_rad_s must be positive: "
                f"{self.max_velocity_rad_s!r} — 0 is not"
                "\"the slowest\", it is \"this trajectory never reaches the "
                "target\""
            )
        if self.max_velocity_rad_s > MAX_COMMAND_VELOCITY_CEILING_RAD_S:
            raise ValueError(
                f"GateLimits.max_velocity_rad_s={self.max_velocity_rad_s} "
                f"exceeds the hard ceiling {MAX_COMMAND_VELOCITY_CEILING_RAD_S} "
                f"rad/s — the trajectory velocity limit may only be tightened, "
                f"never loosened."
            )


@dataclass(frozen=True)
class GateDecision:
    """The gate's verdict.

    ``accepted`` only means "this frame's intent is itself legal"; it **does
    not mean a frame has been or will be sent**.
    """

    accepted: bool
    reason: RejectReason
    detail: str
    #: The original value when validation passed; None when it did not — never
    #: leave a "fixed-up" value for the caller to misuse.
    target_position_rad: Optional[float] = None
    torque_limit_nm: Optional[float] = None
    #: As above. ★ The three values are **all present or all absent**: the gate
    #: lets through the **whole command**; there is no intermediate state such
    #: as "position passes, velocity clamped to the ceiling".
    max_velocity_rad_s: Optional[float] = None

    @property
    def fault_code(self) -> FaultCode:
        return FaultCode.NONE if self.accepted else FaultCode.COMMAND_REJECTED


def _as_finite_float(value: object, name: str) -> tuple[Optional[float], str]:
    """Strict numeric validation, aligned with the SDK's ``normalize_scalar``.

    Returns ``(value, error description)``; when the value is not None the
    error description is the empty string.

    ★ **Explicitly rejects bool**. In Python ``isinstance(True, int)`` is
    true, so without a special case ``True`` would pass itself off as ``1.0``
    and slip all the way into the red line checks — and an input like
    ``torque_limit_nm=True`` can only come from buggy code or dirty data;
    letting it through would disguise "the input is broken" as "the input is
    legal".
    """
    if isinstance(value, bool):
        return None, f"{name} is a bool ({value!r}) — bool is not a legal numeric input"
    if not isinstance(value, (int, float)):
        return None, f"{name} is not a number: {value!r} (type {type(value).__name__})"
    number = float(value)
    if not math.isfinite(number):
        return None, f"{name} is not a finite number: {value!r}"
    return number, ""


def check_command(
    target_position_rad: object,
    torque_limit_nm: object,
    max_velocity_rad_s: object,
    limits: Optional[GateLimits],
) -> GateDecision:
    """Validate one motion command. **Reject by default**: any branch not
    covered returns a rejection.

    This is a pure function — it sends no frame, changes no state and reads no
    clock.

    :param target_position_rad: target position (rad).
    :param torque_limit_nm: the maximum torque budget allowed this time (N·m).
    :param max_velocity_rad_s: the **trajectory velocity limit** for this
        position motion (rad/s).
        ★ All three values are **required**, with no defaults: if any one of
        them cannot be worked out, the command should not pass. Especially
        ``max_velocity_rad_s`` — giving it a default would mean deciding "how
        fast the gripper may move" on the caller's behalf, and that is exactly
        why this message exists.
    :param limits: the limits currently in effect. ``None`` means the limits
        are unavailable → reject everything.

    ★ The order of the criteria: **finiteness first, then range, then
    ceilings**. Each of the three command values forms its own independent
    gate (position / torque / velocity), and failing any one rejects the whole
    command — the unit of rejection is the **command**, not "this item". So a
    rejected command **has no effect in any part**, position included.
    """
    # ── gate 0: limits unavailable, reject everything ────────────────
    #    "cannot get the limits" must never degrade into "no validation" —
    #    that turns fail-closed into fail-open, the most typical silent
    #    failure in the whole safety logic.
    if limits is None:
        return GateDecision(
            accepted=False,
            reason=RejectReason.NO_LIMITS,
            detail=(
                "limits unavailable (no red lines retrieved from the SDK) — "
                "reject by default, no command is let through"
            ),
        )
    if not isinstance(limits, GateLimits):
        return GateDecision(
            accepted=False,
            reason=RejectReason.BAD_LIMITS,
            detail=f"wrong type for limits: {type(limits).__name__}, expected "
                   f"GateLimits",
        )

    # ── gate 1: the target position must be a finite real number ─────
    target, err = _as_finite_float(target_position_rad, "target_position_rad")
    if target is None:
        return GateDecision(False, RejectReason.NOT_FINITE, err)

    # ── gate 2: the torque limit must be a finite real number ────────
    torque, err = _as_finite_float(torque_limit_nm, "torque_limit_nm")
    if torque is None:
        return GateDecision(False, RejectReason.NOT_FINITE, err)

    # ── gate 2': the trajectory velocity limit must be a finite real ─
    #    A gate **fully parallel** to torque. NaN / ±inf / non-numeric / bool
    #    are all stopped here — NaN in particular: every comparison
    #    ``x <= 0`` and ``x > ceiling`` is False for NaN, so failing to stop it
    #    here would let it **silently slip through every velocity criterion**.
    velocity, err = _as_finite_float(max_velocity_rad_s, "max_velocity_rad_s")
    if velocity is None:
        return GateDecision(False, RejectReason.NOT_FINITE, err)

    # ── gate 3: red lines (closed interval, endpoints included) ──────
    if target < limits.red_min_rad or target > limits.red_max_rad:
        return GateDecision(
            accepted=False,
            reason=RejectReason.POSITION_OUT_OF_RANGE,
            detail=(
                f"target position {target:+.6f} rad is outside the red lines "
                f"[{limits.red_min_rad}, {limits.red_max_rad}] — command "
                f"rejected, not clamped (source: {limits.source})"
            ),
        )

    # ── gate 4: the torque limit must be positive ────────────────────
    if torque <= 0.0:
        return GateDecision(
            accepted=False,
            reason=RejectReason.TORQUE_NOT_POSITIVE,
            detail=f"torque_limit_nm={torque!r} is not positive — command "
                   f"rejected",
        )

    # ── gate 5: the torque limit must not exceed the ceiling in effect ─
    #    Note that this compares against limits.torque_limit_nm, and
    #    GateLimits already guarantees at construction that it is <=
    #    TORQUE_LIMIT_CEILING_NM — the two together are what equals "does not
    #    exceed the hard ceiling".
    if torque > limits.torque_limit_nm:
        return GateDecision(
            accepted=False,
            reason=RejectReason.TORQUE_ABOVE_CEILING,
            detail=(
                f"torque_limit_nm={torque} exceeds the ceiling in effect "
                f"{limits.torque_limit_nm} N·m — command rejected, not clamped "
                f"(source: {limits.source})"
            ),
        )

    # ── gate 4': the trajectory velocity limit must be positive ──────
    #    0 is not "the slowest speed allowed" but "the target position can
    #    never be pushed" — a self-contradictory command. A negative number is
    #    even less meaningful (the trajectory pushes **toward** the target;
    #    direction is decided by the position difference, not by the sign of
    #    the velocity). Both are rejected, and **no absolute value is taken**:
    #    using abs() to turn -0.4 into 0.4 would be silent clamping, and this
    #    gate's discipline is "reject, do not clamp".
    if velocity <= 0.0:
        return GateDecision(
            accepted=False,
            reason=RejectReason.VELOCITY_NOT_POSITIVE,
            detail=(
                f"max_velocity_rad_s={velocity!r} is not positive — the whole "
                f"command is rejected (position and torque do not take effect "
                f"either). 0 means the trajectory never reaches the target, "
                f"not \"the slowest\"."
            ),
        )

    # ── gate 5': the trajectory velocity limit must not exceed the
    #    ceiling in effect this time ─────────────────────────────────
    #    Isomorphic to the torque gate: it compares against
    #    limits.max_velocity_rad_s, and GateLimits already guarantees at
    #    construction that it is <= MAX_COMMAND_VELOCITY_CEILING_RAD_S — the
    #    two together are what equals "does not exceed the hard ceiling".
    #    ★ What is compared is **this field**, and it has nothing whatever to
    #    do with HardwareConfig.max_feedback_velocity_rad_s: that quantity
    #    lives on the worst-case budget side (sdk_adapter), and this module
    #    neither sees it nor should see it.
    if velocity > limits.max_velocity_rad_s:
        return GateDecision(
            accepted=False,
            reason=RejectReason.VELOCITY_ABOVE_CEILING,
            detail=(
                f"max_velocity_rad_s={velocity} exceeds the ceiling in effect "
                f"this time {limits.max_velocity_rad_s} rad/s — the whole "
                f"command is rejected, not clamped (source: {limits.source})"
            ),
        )

    return GateDecision(
        accepted=True,
        reason=RejectReason.ACCEPTED,
        detail=(
            f"passed: target={target:+.6f} rad ∈ "
            f"[{limits.red_min_rad}, {limits.red_max_rad}], "
            f"torque_limit={torque} N·m ≤ {limits.torque_limit_nm} N·m, "
            f"max_velocity={velocity} rad/s ≤ {limits.max_velocity_rad_s} rad/s"
        ),
        target_position_rad=target,
        torque_limit_nm=torque,
        max_velocity_rad_s=velocity,
    )


# ─────────────────────────────────────────────────────────────────────
# Contracts not yet implemented (only needed for the real-hardware path /
# Stage 4; written down now so they are not forgotten)
# ─────────────────────────────────────────────────────────────────────
#
# 1. **Mode short-circuit**: when dry_run is true, any intent to send a frame is
#    rejected outright, and that check comes before every other check. Stage 3
#    sends no frames at all, so that branch does not exist yet.
# 2. **Authorization and freshness**: who permitted this motion, and until when.
#    The SDK does not know "which ROS client requested this particular action".
# 3. **Throttling**: an upper bound on the frame send rate. The SDK's
#    send_mit_motion is a single-frame exit point and does not rate-limit; the
#    ROS 2 side's timers/services may be far faster than the CAN bus.
# 4. **Concurrency**: only one motion intent may be in flight at a time. When
#    two services request motion simultaneously, one must be rejected rather
#    than the frames alternating.
# 5. **Watchdog**: sending should stop once the ROS 2 client drops off. The
#    SDK's watchdog watches "how long since a feedback frame arrived", not
#    "how long since a ROS command arrived".
# 6. **Refuse to send during a fault**: while a persistent fault (motor /
#    communication) is latched, new motion commands should be rejected rather
#    than "noted down to move once the fault clears". Stage 3 **still accepts**
#    commands during a fault (under dry_run it only records them, it does not
#    send frames), so this one is not implemented — but it is a different
#    matter from "a persistent fault is not cleared by a valid command", which
#    is already done in gripper_node.
#
# Plus two permanent contracts (unchanged from Stage 1, never to be loosened):
# * Do not recompute or clamp the red lines — the SDK is the only source of
#   truth for them.
# * Do not catch and swallow LimitViolation / SafetyFault.
