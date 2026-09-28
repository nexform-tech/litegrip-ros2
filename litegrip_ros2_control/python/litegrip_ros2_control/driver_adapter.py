"""driver_adapter — the single wrapping layer around the LiteGrip SDK.

This module is the **only** seam between the ROS 2 world and the LiteGrip
SDK. Nodes do not ``import litegrip`` directly; every SDK interaction goes
through here.

Stage 3.1 status
----------------
* :func:`read_limits_from_sdk` — read-only retrieval of the red lines, torque
  limit and command-trajectory velocity limit currently in effect in the SDK.
  It only ``import``s, and **never calls ``connect()``**.
* :class:`TrajectoryLimiter` — pure logic that turns a "target position" into
  a **rate-limited trajectory**. The real-hardware and mock paths share it, so
  "trajectory rate limiting" has exactly one implementation.
* :class:`MockAdapter` — deterministic fake feedback for dry_run; it can
  simulate motor faults, communication loss and emergency stop.
* :class:`SdkAdapter` — the real-hardware adapter layer, but it **never calls
  ``connect()`` itself**: it only accepts an **already-connected**
  ``LiteGrip`` instance.

Field semantics (Stage 3.1 correction)
--------------------------------------
Each field of :class:`Sample` **means exactly one thing**; fields are not
allowed to stand in for one another:

===============  ==========================================
Field            Meaning
===============  ==========================================
``error``        whether an error exists (motor or bridge) — **bool**
``enabled``      **only** whether the motor is enabled
``stopped``      **only** whether the last sample's velocity is near zero
``communication_ok``  **only** whether communication is healthy
``fault_code``   0 = no fault; non-zero = bridge-layer or motor fault
===============  ==========================================

★ The criterion for ``stopped`` is **exactly one**:
``abs(velocity_rad_s) < STOPPED_EPS_RAD_S`` (strictly less than). The
near-zero threshold :data:`STOPPED_EPS_RAD_S` is defined in this module and
**nowhere else**; both adapters take it from here — two separate ``1e-3``
literals would drift sooner or later, and the symptom of that drift is "the
same mechanism reports a different stopped on the two paths".

★ **Why ``velocity_rad_s = −0.0073`` reports ``stopped=false``**: 0.0073 >
1e-3, so the criterion does not hold. This is not a bug, it is the criterion
working faithfully — the minus sign only indicates the direction of motion
(toward the open side); the magnitude is what the criterion tests.
★ Conversely, ``stopped=true`` **only** means "the velocity in the last sample
was near zero"; it does **not** mean "the axis is confirmed locked". External
forces, gravity and inertia can all move the axis between two samples, and
slow creep below the threshold is likewise "not stopped". To tell these apart
you have only the accompanying fields (``enabled`` / ``communication_ok`` /
``fault_code``).

★ **Never write the SDK's enable status code into ``error``.** In the SDK's
``error_code``, ``1`` means **enabled**; passing it into ``error`` as a
boolean turns "normally enabled" into ``error=true``. That conversion is
handled in one place, :func:`motor_fault_code`: ``error_code ∈ {0, 1}``
always maps to "no fault".

Three **construction-time invariants** (see :meth:`Sample.__post_init__`) make
contradiction between the fields impossible — rather than relying on tests to
find contradictions after the fact, make the contradictions unconstructible.

★ Deferred imports
------------------
``import litegrip`` always goes **inside a function body** and must never be
hoisted to module level.

⚠ Read-only ≠ zero side effects
-------------------------------
On real hardware even a pure status read requires ``connect()`` first, and
that call claims can0 and may send handshake/probe frames. That is why
:class:`SdkAdapter` requires the caller to establish the connection first and
then inject it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Protocol

from .safety_gate import (
    MAX_COMMAND_VELOCITY_CEILING_RAD_S,
    MOTOR_FAULT_BASE,
    TORQUE_LIMIT_CEILING_NM,
    FaultCode,
    GateLimits,
)

__all__ = [
    "STOPPED_EPS_RAD_S",
    "Sample",
    "GripperAdapter",
    "TrajectoryLimiter",
    "MockAdapter",
    "SdkAdapter",
    "SdkUnavailable",
    "AdapterLatched",
    "read_limits_from_sdk",
    "motor_fault_code",
    "MOTOR_FAULT_BASE",
]


#: The near-zero criterion for ``stopped`` (rad/s); **strictly less than**
#: counts as stopped.
#:
#: ★ This is the **only** place in the package where that threshold is
#: defined. Both adapters (mock and real hardware) and the default of
#: ``sdk_adapter.HardwareConfig.stopped_eps_rad_s`` point at it. Two separate
#: ``1e-3`` literals would diverge sooner or later, and the symptom of that
#: divergence is very hard to spot: the same motionless mechanism reports a
#: different ``stopped`` under dry_run and on real hardware.
#:
#: Why 1e-3: the velocity in a DM4310 feedback frame is a 12-bit quantized
#: value (the mapped range of ``DQ_MAX_RAD_S=30``), so one LSB of that
#: quantization is about 30/2047 ≈ 0.0147 rad/s — **an order of magnitude
#: larger than this threshold**. So this threshold is not "how slow a motion
#: can we still resolve" but the dividing line between "treat observation
#: noise as motion" and "treat it as standstill"; on real hardware a single
#: frame of quiet jitter can exceed it. To use it to judge "is the axis really
#: at rest", you must confirm over several frames, never from a single frame.
STOPPED_EPS_RAD_S = 1e-3


class SdkUnavailable(RuntimeError):
    """The SDK cannot be imported or is unavailable. **Do not swallow it** —
    the caller must handle it explicitly.
    """


class AdapterLatched(RuntimeError):
    """The execution layer has **latched a safe stop** — it will not execute
    any new target.

    Why this must be a **separate type** (rather than reusing ``ValueError``
    / a bare ``RuntimeError``): ``gripper_node._on_command`` must be able to
    tell two things apart —

    * "the execution layer stopped **by design**" (motor fault / communication
      loss / overtemperature / emergency stop / repeated send failures) — this
      is **not** a code defect and must be handled as a hardware condition;
    * "the gate let it through, but the execution layer rejected it on range
      grounds" — this one **is a contract violation**, an internal
      inconsistency in the bridge layer, and must be handled as a code defect
      (``FaultCode.INTERNAL``).

    Lumped into one type, a **correct** safe stop gets reported as an internal
    bridge-layer bug, and operations go looking at the code instead of at the
    mechanism. This is the same reasoning as ``UNSAFE_INITIAL_STATE`` not
    being folded into ``INTERNAL`` in ``FaultCode``.

    ★ This class lives in ``driver_adapter`` rather than ``sdk_adapter``:
    ``gripper_node`` needs it at **module level** for its ``except``, whereas
    ``sdk_adapter`` may only be imported locally on the real-hardware path
    (requirement 3: dry_run must not import it).
    """

    def __init__(self, fault_code: int, detail: str) -> None:
        super().__init__(detail)
        #: The fault code in effect at latch time. **Always non-zero** —
        #: latching is only ever written by fault paths.
        self.fault_code = int(fault_code)
        self.detail = detail


def motor_fault_code(sdk_error_code: object) -> int:
    """Normalize the SDK's ``error_code`` into a **fault code**.

    SDK semantics (see ``GripperState`` in ``litegrip/models.py``) ::

        error_code == 0  → not enabled, **not** an error
        error_code == 1  → enabled, **not** an error
        anything else    → motor error

    So ``{0, 1}`` always returns ``0`` (no fault); anything else returns
    ``MOTOR_FAULT_BASE + the raw code``.

    ★ This is the **only** conversion point between "SDK enable status code"
    and "error". ``1`` (enabled) can never turn into a fault — a positive case
    in ``test_state_semantics.py`` pins that down.

    :returns: ``0`` means no motor fault; otherwise ``100 + the raw
        error_code``.
    """
    try:
        code = int(sdk_error_code)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        # No valid code available — this must not be treated as "no fault",
        # since that would disguise the unknown as normal.
        return MOTOR_FAULT_BASE
    if code in (0, 1):
        return 0
    return MOTOR_FAULT_BASE + code


@dataclass(frozen=True)
class Sample:
    """One state sample.

    ⚠ This package is a port of ``litegrip_ros2_control``; ``Sample`` no
    longer corresponds to any ``.msg`` — it is now the payload on the
    "daemon → seqlock shared memory → C++ plugin state interface" path (see
    ``LitegripState`` in litegrip_shm.h).
    """

    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    #: Whether an error exists (motor or bridge layer). **Always equal to**
    #: ``fault_code != 0``.
    error: bool
    stopped: bool
    enabled: bool
    communication_ok: bool
    fault_code: int
    #: MOS / coil temperature in ℃. **New in this package**: the SDK already
    #: reads them every frame (for its overtemperature criterion); the original
    #: implementation simply did not expose them. The shared-memory state block
    #: has these two fields, so they are filled in here to keep the two state
    #: interfaces on the plugin side from sitting permanently at 0 (a
    #: diagnostic that is always 0 is worse than none).
    #: The raw ``error_code`` is carried out as well (0=disabled 1=enabled,
    #: anything else is a fault code).
    temperature_mos_c: float = 0.0
    temperature_coil_c: float = 0.0
    error_code: int = 0

    def __post_init__(self) -> None:
        """Construction-time invariants — make contradiction between fields
        impossible.

        These are not "defensive programming" but **semantic constraints**: a
        Sample that violates them describes a self-contradictory world, and
        such data must never be published.
        """
        # ① error and fault_code must agree.
        #    Written independently in two places, they would drift sooner or
        #    later — at which point a subscriber sees error=false with
        #    fault_code=100 and has no way to tell which one to believe.
        if self.error != (self.fault_code != 0):
            raise ValueError(
                f"Sample is self-contradictory: error={self.error} but "
                f"fault_code={self.fault_code} — error must always equal "
                f"(fault_code != 0)"
            )

        # ② When communication is unhealthy it **must not** claim to be
        #    enabled. Without feedback the motor state is unknown; reporting
        #    enabled=true then disguises "unknown" as "known" — the most
        #    dangerous class of false report.
        if not self.communication_ok and self.enabled:
            raise ValueError(
                "Sample is self-contradictory: communication_ok=False but "
                "enabled=True — claiming the motor is enabled while feedback "
                "is unavailable is not allowed"
            )

        # ③ The fault code must not be negative — a negative value has no
        #    meaning in the contract; seeing one means something is miscomputed.
        if self.fault_code < 0:
            raise ValueError(
                f"Sample.fault_code must not be negative: {self.fault_code}")

        if isinstance(self.error, int) and not isinstance(self.error, bool):
            raise ValueError(
                f"Sample.error must be a bool, got {type(self.error).__name__} "
                f"({self.error!r}) — the SDK's enable status code must never "
                f"be written into error"
            )


class GripperAdapter(Protocol):
    """The adapter contract. Both the mock and real-hardware paths must
    satisfy it.
    """

    def snapshot(self) -> Sample:
        """Take one state sample. **Must not block**, and never wait long
        inside a ROS callback.
        """
        ...

    def limits(self) -> Optional[GateLimits]:
        """Return the limits currently in effect; ``None`` when unavailable
        (the gate then rejects by default).
        """
        ...


# ─────────────────────────────────────────────────────────────────────
# Trajectory rate limiting — pure logic, no SDK, no clock reads
# ─────────────────────────────────────────────────────────────────────

class TrajectoryLimiter:
    """Turn a "target position" into a **rate-limited trajectory**, giving the
    position to send this cycle, cycle by cycle.

    ★ It **clamps**; it does not reject — that is the division of labor with
    ``safety_gate``, and the two are not interchangeable:

    ==================  ==========================  ====================
    Layer               Question it answers         Means
    ==================  ==========================  ====================
    ``safety_gate``     "should this command pass"  reject (whole command)
    this class          "where this cycle got to"   clamp (trajectory point)
    ==================  ==========================  ====================

    Making the gate clamp would **silently rewrite** the user's intent (an
    out-of-range target gets "fixed" and then sent); making this class reject
    would throw away a perfectly legal command because "the target is far
    away". Both ways of getting it wrong make "what did that command actually
    execute" unanswerable.

    There is only one rule::

        |Δq| = |reference_new − reference_prev| ≤ max_velocity_rad_s × elapsed_s

    ★ **The first frame starts from the measured position** (``set_reference``,
      step size 0). With no "previous trajectory point" to extrapolate from,
      using 0 as the starting point would make the first frame jump from 0 to
      the target — the most ironic way for "rate limiting" to fail, and at
      exactly the most dangerous moment (just powered up, position unknown).
    ★ **A failed frame send must put the trajectory point back** (rollback via
      ``set_reference``). The trajectory point represents "the position in the
      last frame the driver **confirmed it received**"; if a frame is lost, the
      driver is still executing the earlier command, so a trajectory point that
      advances as usual becomes a lie about "what we sent". Rollback and
      seeding share one primitive on purpose, so that "the trajectory point can
      be set directly" has exactly one entry point — two methods doing the same
      thing means one of them eventually gets forgotten when the other changes.
    ★ ``elapsed_s`` is measured by the caller on a **monotonic clock** and
      passed in. This class **does not read the clock** — if it did, there
      would be no way to verify offline "how much time actually elapsed between
      two cycles", and that is precisely the one thing this rate limit needs
      verified.
    ★ When an input is unusable (non-finite number / ``v_max ≤ 0``) it **stays
      put** and returns the current trajectory point, without raising and
      without guessing: this class is pure logic, the caller's gate has already
      guaranteed those values are legal, and getting here means there is a bug
      — holding still is the only direction that cannot cause motion.
    """

    __slots__ = ("_reference",)

    def __init__(self) -> None:
        self._reference: Optional[float] = None

    @property
    def started(self) -> bool:
        """Whether the starting point has been pinned yet. Until it is,
        :meth:`advance` refuses to work.
        """
        return self._reference is not None

    @property
    def reference_rad(self) -> Optional[float]:
        """The current trajectory point. ``None`` before seeding (**not** 0 —
        0 is a legal position).
        """
        return self._reference

    def set_reference(self, position_rad: float) -> float:
        """Put the trajectory point **directly** at *position_rad*. Two uses,
        one primitive:

        * **seeding**: the first frame calls it with the **measured position**,
          so the step size of the first frame is 0;
        * **rollback**: when a frame was not sent (dropped because the buffer
          filled up), put it back to the position from **before advancing** —
          the trajectory point may only represent a position the driver
          confirmed it received.

        ★ Not split into two methods: that would give "the trajectory point can
          be set directly" two entry points, and sooner or later one of them
          gets forgotten when the other changes — and the missed one would be
          on the rollback path, with the symptom that the trajectory runs ahead
          of the commands actually sent, with no logging at all.
        """
        value = float(position_rad)
        if not math.isfinite(value):
            raise ValueError(
                f"the trajectory point must be a finite number: "
                f"{position_rad!r} — with a non-finite trajectory point, none "
                f"of the subsequent steps can be computed"
            )
        self._reference = value
        return value

    def advance(
        self,
        target_rad: float,
        max_velocity_rad_s: float,
        elapsed_s: float,
    ) -> float:
        """Advance toward *target_rad* by at most
        ``max_velocity_rad_s × elapsed_s``.

        :returns: the trajectory point after advancing. Returns ``None`` when
            not seeded (``started is False``) — the caller must
            :meth:`start_from` first.
        """
        if self._reference is None:
            raise RuntimeError(
                "trajectory not seeded — set_reference(measured position) must "
                "be called first. Using 0 as the starting point would make the "
                "first frame jump straight to the target."
            )

        target = float(target_rad)
        velocity = float(max_velocity_rad_s)
        elapsed = float(elapsed_s)
        if not (math.isfinite(target) and math.isfinite(velocity)
                and math.isfinite(elapsed)):
            return self._reference
        if velocity <= 0.0:
            return self._reference
        if elapsed <= 0.0:
            return self._reference

        step = velocity * elapsed
        if not math.isfinite(step):
            return self._reference

        delta = target - self._reference
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        self._reference += delta
        return self._reference


# ─────────────────────────────────────────────────────────────────────
# Read-only retrieval of limits from the SDK
# ─────────────────────────────────────────────────────────────────────

def read_limits_from_sdk(
    torque_limit_nm: float = TORQUE_LIMIT_CEILING_NM,
    safety_baseline: Optional[str] = None,
    max_velocity_rad_s: float = MAX_COMMAND_VELOCITY_CEILING_RAD_S,
) -> GateLimits:
    """Read the currently effective red lines from the SDK and build the gate
    limits. **Does not call ``connect()``.**

    :param torque_limit_nm: the torque limit to use this time. It is clamped
        once more against the hard ceiling.
    :param max_velocity_rad_s: the **command-trajectory velocity limit** to use
        this time (rad/s). It is likewise clamped once more against the hard
        ceiling. The default is the hard ceiling — the same convention as
        ``torque_limit_nm``: **the default is the widest legal value; any
        tightening must be requested explicitly by the caller**.
    :param safety_baseline: versioned safety baseline (e.g. ``"3.5"`` /
        ``"0.25"``). ``None`` (the default) means "do not load explicitly,
        keep whichever one the SDK currently has in effect".

    ★ The red lines are **not redefined here** — the values come from
    ``litegrip.safety_limits``; this module contains not a single red line
    literal. That way "loosening a red line" has no second place to edit.

    ★ **Why ``safety_baseline`` is needed**: before ``connect()``,
    ``get_active_limits()`` returns the **in-package baseline** (the defaults
    of ``SafetyLimits()``), not the values in ``safety_limits.json`` — the
    JSON is installed during ``connect()`` via ``load_safety_limits()``.
    dry_run never goes through ``connect()`` at all, so the **config file is
    never read**, and "which baseline version is current" is visible only in
    code constants: the caller can neither choose one nor roll back to one.

    Loading it explicitly turns that into an **auditable choice**: the node
    parameter names the version, this function installs it with
    ``load_safety_baseline()`` and reads it back. Changing versions = changing
    one parameter value.

    ★ **fail-closed**: if a version is named but cannot be loaded (file
    missing / invalid / rewritten to be looser / the SDK has no such
    interface), it always raises :class:`SdkUnavailable` — the node then sets
    the limits to ``None`` and the gate rejects **all** commands. **Never
    silently fall back to some default ceiling**: that would make "the config
    was lost" indistinguishable from "the config is correct" for the caller,
    and the node would keep running with a torque limit nobody confirmed.

    :raises SdkUnavailable: SDK import failed, baseline load failed, or the
        limits could not be read.
    """
    # ★ Deferred import — may only happen inside the function body.
    try:
        import litegrip.safety_limits as safety_limits  # noqa: PLC0415
    except Exception as exc:  # pragma: no cover - environment-dependent
        raise SdkUnavailable(
            f"cannot import litegrip.safety_limits: {exc!r}. "
            "Put the ported package on PYTHONPATH, or set the node's sdk_path "
            "parameter."
        ) from exc

    baseline_note = "not explicitly selected"
    if safety_baseline is not None:
        loader = getattr(safety_limits, "load_safety_baseline", None)
        if loader is None:
            raise SdkUnavailable(
                f"the SDK does not support a versioned safety baseline (no "
                f"load_safety_baseline), but the caller asked for "
                f"{safety_baseline!r} — refusing to continue when the "
                f"baseline cannot be confirmed"
            )
        try:
            loader(str(safety_baseline))
        except Exception as exc:
            raise SdkUnavailable(
                f"failed to load safety baseline {safety_baseline!r}: {exc!r} "
                f"— commands will be rejected (fail-closed); no fallback to "
                f"any default ceiling"
            ) from exc
        baseline_note = f"explicitly loaded baseline {safety_baseline}"

    try:
        active = safety_limits.get_active_limits()
        red_min = float(active.red_min_rad)
        red_max = float(active.red_max_rad)
        sdk_tau_max = float(active.params.tau_max_nm)
    except Exception as exc:  # pragma: no cover - SDK-version-dependent
        raise SdkUnavailable(
            f"failed to read effective SDK limits: {exc!r}") from exc

    # ★ Minimum over three layers: requested value / SDK effective ceiling /
    #   ROS hard ceiling. Lowering any one layer takes effect immediately;
    #   raising any one has no effect — this is exactly "tighten only".
    effective = min(float(torque_limit_nm), sdk_tau_max, TORQUE_LIMIT_CEILING_NM)

    # ★ The velocity limit takes the minimum over **two** layers only: this
    #   module does not import the SDK's velocity model (``v_allow`` is a
    #   function of position, not a comparable constant), so the "SDK layer"
    #   has already been folded into the provenance of
    #   MAX_COMMAND_VELOCITY_CEILING_RAD_S.
    #   ★ ``HardwareConfig.max_feedback_velocity_rad_s`` is deliberately
    #     **not** read here — that is the worst-case bound on measured
    #     velocity, on the adapter side, and has nothing to do with the
    #     command trajectory.
    effective_velocity = min(
        float(max_velocity_rad_s), MAX_COMMAND_VELOCITY_CEILING_RAD_S)

    return GateLimits(
        red_min_rad=red_min,
        red_max_rad=red_max,
        torque_limit_nm=effective,
        max_velocity_rad_s=effective_velocity,
        source=(
            f"litegrip.safety_limits.get_active_limits() "
            f"(SDK tau_max_nm={sdk_tau_max}, "
            f"ROS hard ceiling={TORQUE_LIMIT_CEILING_NM}, "
            f"velocity hard ceiling={MAX_COMMAND_VELOCITY_CEILING_RAD_S} rad/s, "
            f"{baseline_note})"
        ),
    )


# ─────────────────────────────────────────────────────────────────────
# dry_run: deterministic mock feedback
# ─────────────────────────────────────────────────────────────────────

class MockAdapter:
    """Fake feedback for dry_run. **This is not a model of the gripper; it
    merely keeps the data flowing.**

    ⚠ You must be clear that it lies, and not mistake it for "simulation
      validation":
      * the position approaches the target as a first-order response, purely
        so that ``position_rad`` / ``velocity_rad_s`` have something to show;
        **it corresponds to no real dynamics**;
      * no friction, no gravity, no delay, no noise, no stall;
      * when things are fine, ``communication_ok`` is always ``True``, which
        only means "the mock sample succeeded" and **does not mean CAN
        communication is healthy**.

    Determinism comes from integrating over a **call count** rather than the
    wall clock — the same call sequence yields the same result on any machine,
    so tests do not fail at random.

    Fault injection (**mock only; real hardware has none**)
    -------------------------------------------------------
    :meth:`simulate_motor_fault` / :meth:`simulate_communication_loss` /
    :meth:`trigger_emergency_stop` and their matching clear methods. Their only
    reason to exist is to make "what the fields should look like under a fault"
    verifiable offline — on real hardware those states are determined by the
    hardware and cannot be injected.

    ★ Safe-stop convention (what the fields should look like under a fault)
    ----------------------------------------------------------------------
    Motor faults and communication loss both **freeze motion**: no position
    integration, velocity 0, torque 0, ``stopped=True``, ``enabled=False``.
    All of them are required — ``stopped=True`` on its own would be misread as
    "confirmed at rest".

    ⚠ Under a fault, ``stopped=True`` means "**the bridge layer stopped
    advancing motion**", **not** "the axis has been measured and confirmed to
    be at rest". On real hardware the bridge layer has no way to know whether
    the axis is still coasting on inertia after a fault. To tell those two
    apart you can only look at the accompanying fields: ``enabled=False``
    (output is cut) and ``communication_ok`` (whether feedback is still
    available).

    A fault is a **persistent state**, not an event: unless
    :meth:`clear_motor_fault` / :meth:`restore_communication` is called, the
    fault keeps being reported. That is a different matter from "a single
    command was rejected" (a transient event, see ``gripper_node``), and the
    two must not stand in for each other.
    """

    #: Duration of each step (seconds). Default corresponds to a 20 Hz rate.
    def __init__(
        self,
        limits: Optional[GateLimits],
        dt: float = 0.05,
        start_rad: Optional[float] = None,
        gain: float = 4.0,
        max_speed_rad_s: float = 1.0,
        stopped_eps_rad_s: float = STOPPED_EPS_RAD_S,
    ) -> None:
        self._limits = limits
        self._dt = float(dt)
        self._gain = float(gain)
        self._max_speed = float(max_speed_rad_s)
        self._stopped_eps = float(stopped_eps_rad_s)

        if start_rad is None:
            if limits is not None:
                start_rad = (limits.red_min_rad + limits.red_max_rad) / 2.0
            else:
                start_rad = 0.0
        self._position = float(start_rad)
        self._velocity = 0.0
        self._target: Optional[float] = None
        self._torque_limit: float = (
            limits.torque_limit_nm if limits is not None else 0.0
        )
        self._steps = 0

        #: ★ **Shares the same** :class:`TrajectoryLimiter` as the
        #: real-hardware path. It is in the mock not to "look more like real
        #: hardware" but to make ``max_velocity_rad_s`` **observable** under
        #: dry_run: the target slot holds a rate-limited trajectory and
        #: ``position_rad`` follows it, so "was the command's velocity limit
        #: taken seriously" is visible on ``/gripper/state``. Using two
        #: different timing logics for real hardware and mock would mean
        #: dry_run cannot validate the real-hardware path.
        self._trajectory = TrajectoryLimiter()
        self._target_velocity: float = 0.0

        # ── fault injection state ───────────────────────────────────
        self._motor_error_code: Optional[int] = None   # None = no motor fault
        self._comms_lost = False
        self._estop_latched = False

    # ── command recording (dry_run only records; it drives no motor) ──
    def set_target(
        self,
        target_position_rad: float,
        torque_limit_nm: float,
        max_velocity_rad_s: float,
    ) -> None:
        """Record a target that has **already passed the gate**.

        ``max_velocity_rad_s`` is the **trajectory velocity limit** for this
        motion (rad/s) — the trajectory advances according to it, and it is
        **not** written into any MIT frame field (there are no frames under
        dry_run at all).
        """
        self._target = float(target_position_rad)
        self._torque_limit = float(torque_limit_nm)
        self._target_velocity = float(max_velocity_rad_s)

    @property
    def steps(self) -> int:
        return self._steps

    @property
    def target_rad(self) -> Optional[float]:
        return self._target

    @property
    def command_reference_rad(self) -> Optional[float]:
        """The current **trajectory point** (the rate-limited position), not
        the final target.

        Together with :attr:`target_rad` it forms the two endpoints of "how
        far did the command actually get": the target is the destination, the
        trajectory point is what was actually sent this cycle. ``None`` before
        seeding.
        """
        return self._trajectory.reference_rad

    # ── fault injection (mock only) ──────────────────────────────
    def simulate_motor_fault(self, sdk_error_code: int = 8) -> None:
        """Simulate a motor error. ``sdk_error_code`` goes through the same
        normalization path as on real hardware.
        """
        self._motor_error_code = int(sdk_error_code)

    def clear_motor_fault(self) -> None:
        self._motor_error_code = None

    def simulate_communication_loss(self) -> None:
        """Simulate communication loss: feedback freezes at the last known
        position.
        """
        self._comms_lost = True

    def restore_communication(self) -> None:
        self._comms_lost = False

    def trigger_emergency_stop(self) -> None:
        """Simulate an emergency stop.

        ⚠ **This only latches the bridge layer's "stopped" state; it does not
        call the SDK's ``emergency_stop()``.** Stage 3 forbids calling any
        real-hardware control interface, so this only changes the mock's
        internal state. The real-hardware path (Stage 4) is what wires it to
        ``LiteGrip.emergency_stop()`` — note that at that point the interface
        **returns None** at the ``LiteGrip`` level (not a bool), so it must not
        be written as ``if gripper.emergency_stop():``.
        """
        self._estop_latched = True
        self._velocity = 0.0

    def clear_emergency_stop(self) -> None:
        self._estop_latched = False

    @property
    def emergency_stop_latched(self) -> bool:
        return self._estop_latched

    # ── sampling ───────────────────────────────────────────────────
    def snapshot(self) -> Sample:
        self._steps += 1

        # ── communication loss: position/velocity frozen, and **never claim
        #    to be enabled** ─────────────────────────────────────────
        if self._comms_lost:
            self._velocity = 0.0
            return Sample(
                position_rad=self._position,
                velocity_rad_s=0.0,
                torque_nm=0.0,
                error=True,
                stopped=True,
                enabled=False,
                communication_ok=False,
                fault_code=int(FaultCode.NO_FEEDBACK),
            )

        # ── motor fault: same rank as communication loss, **also frozen** ──
        #    ★ A faulted axis must not keep integrating position. If the mock
        #    reported error=true while still walking toward the target as
        #    usual, `stopped` would be false — the subscriber would see
        #    "faulted, but still moving", whereas on real hardware the driver
        #    has long since cut its output. "Safe stop" means: the bridge layer
        #    no longer advances motion, and truthfully reports that it stopped.
        #
        #    `enabled=False` matches the SDK: `is_enabled` is equivalent to
        #    ``error_code == 1``, and under a fault error_code ∉ {0,1}, so it
        #    does not count as enabled to begin with.
        if self._motor_error_code is not None:
            self._velocity = 0.0
            return Sample(
                position_rad=self._position,   # frozen at last known position
                velocity_rad_s=0.0,
                torque_nm=0.0,                 # driver cut output → no torque
                error=True,
                stopped=True,
                enabled=False,
                communication_ok=True,         # still answering; fault is its own
                fault_code=motor_fault_code(self._motor_error_code),
            )

        # ── position integration (does not move while the e-stop is latched) ──
        if self._estop_latched or self._target is None:
            self._velocity = 0.0
        else:
            # ★ Advance the **trajectory point** first, then move the position
            #   toward the trajectory point — the order must not be reversed.
            #   Reversed, the position would lunge toward the **final target**
            #   first and rate limiting would become decorative: the velocity
            #   the mock reports would be determined by ``gain`` and the
            #   distance to the target, with no relation to the command's
            #   velocity limit, so dry_run could not validate that field.
            #
            # ★ The first frame (the first sample after the command arrives)
            #   **only pins the starting point and does not advance** — the
            #   trajectory point lands exactly on the measured position, step
            #   size 0. The rule is **word for word identical** to the
            #   real-hardware adapter layer (the ``seeded`` branch of
            #   sdk_adapter._send_motion): if the two disagreed, dry_run would
            #   not be validating the real-hardware path.
            if not self._trajectory.started:
                self._trajectory.set_reference(self._position)
                reference = self._trajectory.reference_rad
            else:
                reference = self._trajectory.advance(
                    self._target, self._target_velocity, self._dt)
            assert reference is not None           # both branches settle it
            error = reference - self._position
            speed = max(-self._max_speed, min(self._max_speed, self._gain * error))
            step = speed * self._dt
            if abs(step) > abs(error):
                step = error
            self._position += step
            self._velocity = step / self._dt if self._dt > 0 else 0.0

        # ── torque ──────────────────────────────────────────────────
        #    ★ Use the **trajectory point**, not the final target, to compute
        #    the tracking error. After rate limiting the two are no longer the
        #    same thing: computed from the final target, a trajectory that has
        #    only just started ramping up would immediately report full torque
        #    (there are still 1.2 rad to go), and that number represents
        #    neither the tracking error nor any real dynamics — it would only
        #    make the dry_run output harder to read.
        if self._target is None or not self._trajectory.started:
            torque = 0.0
        else:
            torque = self._gain * (self._trajectory.reference_rad - self._position)
            if self._torque_limit > 0.0:
                torque = max(-self._torque_limit, min(self._torque_limit, torque))
        if not math.isfinite(torque):
            torque = 0.0

        # ── by now the only remaining kind of "stop" is the latched e-stop ──
        #    Communication loss and motor fault both returned early above.
        #    Those two conditions are **not checked a second time** here — two
        #    copies of a criterion inevitably drift, and the time it drifts
        #    will be on real hardware, with the symptom "reporting healthy
        #    while already faulted".
        #    ★ The criterion is **word for word identical** to the
        #    real-hardware adapter layer: the same default threshold constant
        #    (the default value of ``self._stopped_eps`` is exactly
        #    :data:`STOPPED_EPS_RAD_S`) and the same strict less-than. Either
        #    change both sides together, or change neither.
        stopped = self._estop_latched or abs(self._velocity) < self._stopped_eps

        return Sample(
            position_rad=self._position,
            velocity_rad_s=self._velocity,
            torque_nm=torque,
            error=False,
            stopped=stopped,
            enabled=True,
            communication_ok=True,
            fault_code=int(FaultCode.NONE),
        )

    def limits(self) -> Optional[GateLimits]:
        return self._limits


# ─────────────────────────────────────────────────────────────────────
# Real hardware: inject an "already connected" instance (Stage 3 does not
# construct one)
# ─────────────────────────────────────────────────────────────────────

class SdkAdapter:
    """The real-hardware adapter layer. **It never calls ``connect()``
    itself.**

    ★ A deliberate construction constraint: this class only accepts a
    ``LiteGrip`` instance that has **already been connected externally**.
    Without an instance there is nothing to connect — "this module will open
    CAN on its own initiative" is not even expressible in the type system.

    Stage 3 does not construct this class (the requirements explicitly forbid
    ``connect()``).

    ⚠ The ``wait`` argument of ``get_state()``: the default ``wait=True``
    blocks until a feedback frame arrives. **Inside a ROS timer callback you
    must pass ``wait=False``**, otherwise a single-threaded executor gets
    stalled. It is passed ``False`` by default here.
    """

    def __init__(self, gripper: object, limits: Optional[GateLimits]) -> None:
        self._gripper = gripper
        self._limits = limits

    def snapshot(self) -> Sample:
        # ★ The one correct entry point. There is **no** read_state() method in
        #   the SDK — writing gripper.read_state() raises AttributeError at
        #   runtime.
        state = self._gripper.get_state(wait=False)

        # ★ The enable state comes from is_enabled; **never** treat it as an
        #   error. The SDK's error_code == 1 means enabled, and that value may
        #   only affect enabled.
        enabled = bool(getattr(state, "is_enabled", False))

        # ★ Motor errors are normalized separately — error_code ∈ {0,1} both
        #   count as "no fault".
        fault_code = motor_fault_code(getattr(state, "error_code", 0))

        velocity = float(state.velocity_rad_s)
        return Sample(
            position_rad=float(state.position_rad),
            velocity_rad_s=velocity,
            torque_nm=float(state.torque_nm),
            error=(fault_code != 0),
            # ★ The criterion is **exactly this one**: |feedback velocity| <
            #   STOPPED_EPS_RAD_S (strictly less than). Reporting false at a
            #   measured −0.0073 rad/s is **the criterion working faithfully**:
            #   0.0073 > 1e-3 — the minus sign only means motion toward the
            #   open side, the magnitude is what the criterion tests.
            #   ⚠ This used to be the literal 1e-3, written separately from
            #     MockAdapter — the symptom of that drift was "the same
            #     motionless mechanism reports a different stopped under
            #     dry_run and on real hardware". Now both adapters take the
            #     same constant.
            stopped=abs(velocity) < STOPPED_EPS_RAD_S,
            enabled=enabled,
            communication_ok=True,
            fault_code=fault_code,
        )

    def limits(self) -> Optional[GateLimits]:
        return self._limits
