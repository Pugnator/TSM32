# Firmware 1.0 code review

- Review date: 2026-08-25
- Baseline: `v0.7` (`dd5afe3`)
- Reviewed revision: `e170868` on `release/F103` (`v0.9-17-ge170868`)
- Diff reviewed: `v0.7..e170868`, with whitespace-only changes ignored

## Release recommendation

At reviewed revision `e170868`: **hold the 1.0 release until CR-001 through CR-003 are resolved or explicitly accepted.** The remediation branch described below addresses every finding and passes the automated gates; on-target and isolated-bus validation remain release requirements.

## Remediation status

All seven findings are addressed on branch `fix/v1.0-review-findings`:

- CR-001: DTC monitoring is read-only; production code no longer sends service `0x14`.
- CR-002: engine state requires independently fresh, CRC-validated RPM and speed signals.
- CR-003: the software lock remains set when the configured policy requires an ignition-cycle unlock.
- CR-004: the bounded retry retains arbitration monitoring.
- CR-005: the host test shim is repaired and an ordinary `make -C tests test` target was added.
- CR-006: raw accelerometer and gyro data share a proper 180-degree Z-axis rotation.
- CR-007: zero/non-finite fusion gradients are not normalized.

The original findings below are retained as the review record.

## Scope

The review covers first-party application code and custom drivers under `Core`, including the J1850 implementation/parser, TSM emulation, engine/starter/bulb/switch control, watchdog, and AHRS/MPU9250 code.

STMicroelectronics-generated and third-party files were excluded from code-quality review: CMSIS, STM32 HAL, startup/linker files, and Cube-generated peripheral/interrupt boilerplate. Generated integration files were consulted only where necessary to establish how first-party code is configured or called. Existing behavior that was unchanged from `v0.7` was not reported.

## Findings

### CR-001 — High — Production code automatically clears all IPC diagnostic history

Locations: `Core/Src/tsm.cc:224-317`, `Core/Src/j1850parser.cc:354-362`

The new ten-second diagnostic state machine sends service `0x14` to the IPC whenever either:

- source `0x61` reports any non-`P0000` DTC; or
- the security indicator remains lit even when no DTC was returned.

`ipcDtcSeen` does not identify a known, emulation-induced code; every IPC DTC takes the same path. The transmitted request is an unqualified IPC clear, so a genuine unrelated fault can be erased along with the intended `U1064`/`U1255` condition. The `silHeld` path can also clear history merely as a diagnostic experiment. This destroys evidence needed for service and can conceal a real fault from the rider or technician.

Recommendation: do not clear DTCs automatically in normal firmware operation. Make clearing an explicit service action. If an automatic workaround is unavoidable, positively identify both the module and an allow-listed code, retain/report the original code, and confirm that the protocol can clear only that code before transmitting.

### CR-002 — High — Any J1850 SOF keeps stale RPM and speed authoritative

Locations: `Core/Src/j1850vpw.cc:265-276`, `Core/Src/engine_state.cc:25-43`, `Core/Src/j1850parser.cc:165-179`, `Core/Src/bulb_ctrl.cc:161-190`

`frameCounter` increments as soon as an SOF-width pulse is detected, before a complete frame, CRC validation, or RPM/speed decoding. `Engine::handler()` treats every counter change as fresh engine telemetry. Consequently, unrelated valid traffic—or repeated SOF-like noise—can keep the engine state out of `Unknown` indefinitely while `rpms` and `kph` retain old or initial values.

While the state is not `Unknown`, the voltage fallback in `bulb_ctrl.cc` is disabled. Starter locking/unlocking can therefore be based on stale data precisely when the required ECM frames are missing.

Recommendation: timestamp validated RPM and speed updates separately in the parser, after length and CRC checks. Enter `Unknown` and enable the fallback if either required signal expires. Do not use raw bus activity as proof that engine telemetry is current.

### CR-003 — High — Engine state reports the starter unlocked while the relay remains disabled

Locations: `Core/Inc/settings.h:58`, `Core/Src/starter_ctrl.cc:8-50`, `Core/Src/engine_state.cc:93-100`

The release configuration sets `STARTER_UNLOCK_DISABLE` to `1`. After `disableStarter()` sets the private `starterDisabled` latch, every later `enableStarter()` returns without changing the relay. Nevertheless, the engine-off debounce path clears `gStarterLocked`, calls `enableStarter()`, and logs `starter UNLOCKED`.

This creates contradictory observable state: `Engine::isStarterLocked()` says false while the hardware remains locked. A stopped or stalled engine cannot be restarted through the documented state transition, and later logic cannot reliably tell whether the relay is usable.

Recommendation: choose one policy and represent it consistently. If lockout must last until ignition reset, keep `gStarterLocked` set and remove the false unlock/log. If the engine FSM is allowed to unlock after a confirmed stop, provide a deliberate API that clears the private latch and drives the relay, with tests for both policies.

### CR-004 — Medium — Arbitration-loss recovery deliberately transmits without monitoring

Location: `Core/Src/j1850vpw.cc:439-482`

After `LOST_ARB`, `j1850TxRaw()` waits for idle and retries once with `arbMonitorEnabled_ = false`. Carrier sense does not prevent another node from beginning at nearly the same time. The blind retry cannot detect that collision and may corrupt both messages on the vehicle bus.

Recommendation: retain arbitration monitoring on every attempt and retry after a bounded/randomized backoff. If the current monitor produces false losses, correct or qualify its timing instead of disabling collision detection. Expose a final transmit failure to the calling state machine.

### CR-005 — Medium — The added host unit-test target does not link

Locations: `Core/Src/switch_ctrl.cc:100`, `tests/tsm.h`, `tests/test_blinker.cc`

The test build fails because `switch_ctrl.cc` now calls `J1850VPW::onEofTimeout()`, but the host test shim does not define it:

```text
undefined reference to `J1850VPW::onEofTimeout()`
```

This prevents the only added host-side behavior tests from running before release. There are also no automated tests for the new engine-state transitions, starter latch policy, J1850 parser freshness, transmit arbitration, or DTC-clearing policy.

Recommendation: add the missing no-op/mock implementation to the test shim, make the host test an ordinary build/CI target, and add focused tests for CR-001 through CR-004.

### CR-006 — Low (configuration-dependent) — Raw IMU axis mapping is not a valid rotation

Locations: `Core/Src/ahrs/impl/mpu9250/accel.cc:57-65`, `Core/Src/ahrs/impl/mpu9250/gyro.cc:61-65`, `Core/Src/tsm.cc:100-101`

The raw sensor path negates X while leaving Y and Z unchanged, but the comment calls this a 180-degree rotation about Z. A 180-degree Z rotation negates both X and Y; negating one axis is a reflection with determinant -1. Applying that transform identically to acceleration and angular velocity can produce an inconsistent/left-handed frame and incorrect fused orientation.

The current release constructs the AHRS with DMP mode enabled (`true`), so this raw path is not active in the reviewed configuration. It becomes a defect if raw fusion is selected later.

Recommendation: document the physical sensor and body coordinate conventions, derive a proper right-handed rotation matrix, and test known static poses and signed rotations.

### CR-007 — Low (configuration-dependent) — Six-axis Madgwick update can normalize a zero gradient

Location: `Core/Src/ahrs/fusion.cc:54-91`

The new magnitude-based accelerometer check correctly accepts level samples such as `(0, 0, 1)`, but for an identity quaternion and perfectly aligned gravity the corrective gradient `s0..s3` is exactly zero. With `SPEED_MATH=0`, `FAST_INV_SQRT(0)` is `1 / sqrt(0)`; multiplying the zero gradient by infinity produces NaNs that can poison the quaternion.

This path is also inactive while DMP mode remains enabled.

Recommendation: calculate the gradient norm and apply correction only when it is finite and above a small epsilon. Add a regression test using identity orientation with `(0, 0, 1)` acceleration and zero gyro.

## Initial validation performed

- Fresh STM32F103 release build: **pass**.
- Compiler warnings in the fresh build: **none observed**.
- Image size: `text=49,240`, `data=100`, `bss=9,088` bytes (`58,428` total reported by `size`).
- Host blinker test compilation: **fail at link**, as described in CR-005.
- Hardware-in-loop, live J1850-bus, relay, watchdog-reset, and IMU-motion validation: **not performed**; no hardware was available to this review.

The current untagged build identifies itself from Git as `v0.9-17-ge170868`. Build and archive the release from the final `v1.0` tag so the embedded version metadata is reproducible.

## Remediation validation

- `make -C tests test`: **pass** — 13 blinker cases, engine-state/starter-policy regression, and AHRS transform/gradient regression.
- Fresh STM32F103 release build: **pass**, with no compiler warnings observed.
- Remediated image size: `text=49,084`, `data=100`, `bss=9,080` bytes (`58,264` total reported by `size`).
- Static check confirms application code contains no DTC clear request and no blind-arbitration retry.

## Suggested release gate

1. Resolve or explicitly accept CR-001 through CR-003.
2. Restore and run the host test target; add regression coverage for starter and J1850 freshness behavior.
3. Exercise arbitration loss and DTC handling on an isolated/simulated bus before connecting to a vehicle.
4. Run an on-target smoke test covering cold boot, watchdog recovery, engine start/stop/stall, starter relay output, indicators, and J1850 traffic.
5. Tag the exact validated commit as `v1.0`, rebuild from that tag, and archive the ELF/map/HEX plus the test record.
