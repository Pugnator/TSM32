# MPU-9250 IMU / AHRS Code Audit

Audit of the first-party MPU-9250 attitude/heading driver
([`Core/Src/ahrs/`](../Core/Src/ahrs/)). Findings are verified against the
InvenSense MPU-9250 / AK8963 register maps and the code, and each is tagged
**LIVE** (in the active runtime path) or **DORMANT** (compiled/selectable but not
on the current path).

## Active configuration

- [`tsm.cc`](../Core/Src/tsm.cc) constructs the AHRS with `useDmp = true` → the
  **DMP quaternion path** is the production path (`sampleQuant()` DMP branch →
  DMP FIFO quaternion → yaw for turn detection).
- [`ahrs_config.h`](../Core/Src/ahrs/inc/ahrs_config.h) sets
  `DISABLE_MAGNETOMETER 1` → the magnetometer and 9-DoF fusion are off.
- Therefore the **raw accel/gyro + `madgwick6DoF` path and all magnetometer /
  heading code are dormant** but shipping and selectable. They become live the
  moment `useDmp=false` or the magnetometer is re-enabled.

The DMP path itself audited clean: `DMP_PACKET_SIZE = 32` = quat(16) + accel(6) +
gyro(6) + 4-byte TAP gesture footer; the Q30→unit quaternion scaling
`(q>>16)/16384` is correct; the aerospace ZYX yaw/pitch/roll extraction is
correct.

---

## Findings

| # | Sev | Status | Path | Summary |
|---|-----|--------|------|---------|
| H1 | High | **open — needs decision** | dormant (raw/6-DoF) | Accel/gyro axis remap is a reflection (det −1), not a rotation |
| M1 | Med | open — needs decision | dormant (mag) | AK8963 Z axis not negated; mag frame not aligned to body frame |
| M2 | Med | open | dormant (heading) | `getHeadingAngle()` reads pitch/roll swapped |
| M3 | Med | **open — validate on bike** | **LIVE (DMP)** | `interruptStatus()` returns DataReady before DmpInterrupt |
| M4 | Med | open | dormant (9-DoF) | `madgwick9DoF` keeps the per-axis truthiness accel test |
| L1 | Low | **fixed** | inert | `FifoOverflow` decoded from FSYNC bit (0x08) not 0x10 |
| L3 | Low | **fixed** | inert | `enableDMP(enable)` hardcoded `useDmp_ = true` |
| L4 | Low | **fixed** | inert (`#ifdef`) | `selfTest()` `writeRegMpu` where it must `readRegMpu` |
| L5 | Info | noted | mixed | dead `gSensF`, no-op `ypr.x` wrap, split FIFO-count reads |

### H1 — Accel/gyro axis remap is an improper rotation *(High, dormant)*
[`accel.cc:63-65`](../Core/Src/ahrs/impl/mpu9250/accel.cc#L63-L65),
[`gyro.cc:63-65`](../Core/Src/ahrs/impl/mpu9250/gyro.cc#L63-L65) negate **only X**
(`x=-raw, y=+raw, z=+raw`). That matrix `diag(-1,+1,+1)` has determinant −1: it
is a reflection. The sensor frame is right-handed; the target
"forward-right-up" body frame in the comment is **left-handed**, so no proper
rotation can produce it. Angular velocity is a pseudovector — under a reflection
it must pick up an extra sign relative to a true vector (accel) — so applying the
same reflection to both makes the gyro-integration prediction and the
accelerometer gravity correction **fight each other** in the Madgwick filter:
in raw/6-DoF mode pitch and roll (lean angle) come out sign-inverted or fail to
converge, while yaw-only motion looks fine.

**Decision needed:** pick a *right-handed* body frame and apply the identical
proper rotation to accel and gyro. Options: `x=-raw_x, y=-raw_y, z=+raw_z`
(true 180° about Z → X-forward, **Y-left**, Z-up), or `x=-raw_x, y=+raw_y,
z=-raw_z` (X-forward, Y-right, **Z-down**). The choice flips a sign in downstream
turn/lean interpretation, so it needs a bike test before enabling raw mode.
No effect on the current DMP path.

### M1 — AK8963 Z not negated *(Medium, dormant)*
[`mag.cc:134-136`](../Core/Src/ahrs/impl/mpu9250/mag.cc#L134-L136) swaps X/Y
(correct) but leaves Z un-negated. The AK8963 die is rotated inside the MPU;
aligning it to the accel/gyro frame is `x=magY, y=magX, z=-magZ`. With Z's sign
wrong the tilt-compensated heading / 9-DoF reference field is corrupted. Also
the mag result is written in the un-remapped frame while accel/gyro are remapped
(H1) — the mag needs the *same* body remap once H1's convention is chosen.
Coupled to H1; resolve together when re-enabling the magnetometer.

### M2 — `getHeadingAngle()` pitch/roll swapped *(Medium, dormant)*
[`ahrs.cc:192-197`](../Core/Src/ahrs/ahrs.cc#L192-L197): `getYawPitchRoll()`
stores `ypr_.z = pitch`, `ypr_.y = roll`, but `getHeadingAngle()` reads
`pitch = pr.y` (actually roll) and `roll = pr.z` (actually pitch). The
tilt-compensation applies roll where it expects pitch. Fix: `pitch = pr.z;
roll = pr.y;`. Unambiguous; dormant (mag off).

### M3 — Interrupt source order hazard in the DMP path *(Medium, LIVE)*
[`imu.cc:182-190`](../Core/Src/ahrs/impl/mpu9250/imu.cc#L182-L190) tests
`RAW_DATA_RDY` (0x01) before `DMP_INT` (0x02); the DMP consumer
([`ahrs.cc:231`](../Core/Src/ahrs/ahrs.cc#L231)) bails unless it sees
`DmpInterrupt`. If `INT_STATUS.RAW_DATA_RDY` is ever set alongside the DMP bit,
`interruptStatus()` returns `DataReady`, the DMP branch skips the FIFO drain, and
orientation stutters until the ≥512-byte FIFO reset. The unit runs today, which
suggests the RAW bit isn't latching in DMP mode on this part — but it's fragile.
**Recommended fix:** make `interruptStatus()` mode-aware (test the DMP bit first
when `useDmp_`). This touches the working path, so validate on the bike before
shipping.

### M4 — `madgwick9DoF` per-axis truthiness test *(Medium, dormant)*
[`fusion.cc:166`](../Core/Src/ahrs/fusion.cc#L166) gates the accel/mag correction
on `if (ax && ay && az)` — under vibration any axis passing through exactly
`0.0f` drops the whole correction for that sample. This is the exact defect
already fixed in `madgwick6DoF` (squared-magnitude guard, `fusion.cc:57`). Mirror
that guard here. Dormant (9-DoF only runs with mag enabled).

### L1 / L3 / L4 — fixed (inert corrections, zero runtime change)
- **L1** `imu.cc`: `FifoOverflow` now decoded from bit4 (0x10); 0x08 is FSYNC.
  Never branched on, so behaviour is unchanged — corrected for future use.
- **L3** `dmp.cc`: `enableDMP()` now sets `useDmp_ = enable` (was hardcoded
  `true`). The single caller passes `true`, so no behaviour change.
- **L4** `selftest.cc`: the accel-average loop now `readRegMpu` (was
  `writeRegMpu`, making the accel baseline garbage). Body is behind
  `#ifdef MPU_SELFTEST` (undefined) — not compiled.

### L5 — informational
- `gSensF = 10*4912/32768` (imu_spi/i2c) is dead; the live mag scale `mRes`
  correctly uses the 16-bit range `32760`. Delete `gSensF` to avoid confusion.
- `getYawPitchRollD()` wrap only adds 360 when `x < -180`; `atan2` already returns
  `[-180,180]`, so it is a no-op (turn detection uses relative yaw, so harmless).
- `fifoDataReady()` reads `FIFO_COUNTH`/`COUNTL` in two transactions; a burst read
  is the datasheet-coherent way. Low probability at these rates.

---

## Verified correct (not bugs)

Scale factors `aMult = 2/32768` (±2 g), `gMult = 250/32768` (±250 dps), `mRes`
(16-bit AK8963); gyro `CONFIG=0x03` (DLPF 41 Hz), `SMPLRT_DIV=4` (200 Hz),
`GYRO_FS_SEL=0`, Fchoice_b cleared (`~0x03`, fixed earlier); accel `AFS_SEL=0`,
`A_DLPFCFG=5` (10.2 Hz), accel_fchoice_b=0; AK8963 mode bytes 0x16/0x0F/0x12, the
ST1-DRDY gate and ST2/HOFL overflow read, and the ASA factory formula
`((ASA-128)*0.5/128)+1`; temperature `raw/333.87 + 21`; gyro hardware-bias
procedure; Madgwick 6-DoF (deg→rad, `1/sampleFreq` with div-0 clamp, final
normalization); DMP quaternion scaling and packet size.

---

## Recommendation

The **live DMP path is sound** apart from the fragile-but-working M3 interrupt
ordering. The High/Medium findings (H1, M1, M2, M4) all live in the **dormant**
raw-fusion / magnetometer paths and will bite the moment those modes are
re-enabled. Before enabling raw or 9-DoF mode: pick the body-frame convention
(H1), align the magnetometer to it (M1), fix the heading swap (M2) and the 9-DoF
guard (M4) together, and validate lean-angle sign on the bike. M3's mode-aware
fix is worth doing but should be bench/bike-verified because it changes the
working path.
