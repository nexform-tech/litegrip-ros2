"""litegrip_ros2_control — ros2_control adapter layer for the LiteGrip gripper.

Layering (same shape as ``litearm_ros2_control``)::

    litegrip_ros2_control.py   ← C++ SystemInterface plugin (memcpy + unit conversion)
      ⇅  seqlock shared memory (contract: include/litegrip_ros2_control/litegrip_shm.h)
    hw_daemon.py               ← hardware daemon (sole hardware owner here, no ROS)
      ⇅  LiteGrip SDK
    can0 (= the STM32 board's if0 gs_usb bridge) → DM4310

Python-side modules::

    hw_daemon.py       daemon: the shared memory ⇄ adapter loop + safety orchestration
    shm_bridge.py      ctypes bindings for the shared memory contract (mirrors the C
                       header and validates the layout on load)
    safety_gate.py     the one gate before a frame is sent (**pure logic**: imports
                       neither rclpy nor litegrip)
    driver_adapter.py  everything outside the SDK: the Sample carrier, trajectory
                       rate limiting, the mock adapter
    sdk_adapter.py     the **only** code in this package that touches hardware
                       (owns can0, sends MIT frames)

Design constraints (inherited from ``litegrip_ros_bridge``, not to be relaxed)
-----------------------------------------------------------------------------
1. **dry_run by default**: ``dry_run`` defaults to ``True``. Touching real
   hardware requires **both** switches to hold (``dry_run=false`` and
   ``hardware_enable=true``); there is no "one switch turns on hardware" path.
2. **Lazy SDK import**: never ``import litegrip`` at module top level. The
   dry_run path never imports the SDK, so a machine without the SDK can still
   build and run.
3. **Tighten only**: the torque ceiling and the command trajectory rate ceiling
   can only be turned down; a value above the hard ceiling is rejected and falls
   back to the hard ceiling. The safety baseline's red line must **not** appear
   as a second literal anywhere in this package.
4. **fail-closed**: a missing or invalid safety setting rejects commands outright
   and never silently falls back to a default. While
   ``max_feedback_velocity_rad_s`` is uncalibrated, the hardware path **refuses
   to send any motion frame**.
5. **A rejection means zero frames**: when the gate rejects a command, the
   execution layer's target slot was never written — that is structural (the gate
   returns before ``set_target``), not reliant on the adapter's own discipline.
6. **Two fault classes, two registers**: transient (a single command was
   rejected, cleared by the next valid command) and persistent
   (motor/communication/internal, cleared only once sampling observes health
   again). Masking a hardware fault with one ordinary command is the most
   dangerous class of error in a bridge like this.
7. **A latched safe stop does not self-recover**: once latched, this process
   sends no further motion frames and must be investigated and restarted by hand.
   Any "automatic recovery" turns a real mechanical fault into a silent repeated
   restart.

⚠ This package does not modify the LiteGrip SDK, its calibration programs, or
  the safety files; those are provided externally, and the ``sdk_path``
  parameter points at a copy carried in this workspace.
"""

__version__ = "1.0.0"
