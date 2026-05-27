# J1850-VPW support plan

Goal: stop the ECM from raising **MIL** (check engine) and **SIL** (security
lamp) whenever this aftermarket turn-signal module replaces the OEM TSM.
Preferred outcome — full handshake with the ECM so it sees a "real" TSM.
Acceptable fallback — keep clearing the offending DTCs from the cluster in a
tight loop so the lamps never latch.

Target bike: **2005 Harley-Davidson Sportster, carbureted**, Delphi ECM on a
single-wire J1850-VPW bus.

---

## 1. State of play

### 1.1 Wiring already on the board

Transceiver: **NXP MCZ33390** (single-wire J1850-VPW), confirmed populated.
Its TX input is driven by the MCU; its RX output feeds back into the MCU.
Both ends are positive-logic on the MCU side (high = bus active = +7.5 V).

| Signal | MCU pin | Peripheral | Notes |
|---|---|---|---|
| J1850 RX | **PA1** | TIM2_CH2 (input capture, IT + DMA) | Edge-timing capture, polarity flipped on every edge — see [j1850vpw.cc](Core/Src/j1850vpw.cc) |
| J1850 TX | **PA2** | GPIO push/pull, bit-banged with DWT_CYCCNT | Idles low (bus passive); hard-loop `J1850delayUS()` in [j1850vpw.cc](Core/Src/j1850vpw.cc#L27); arbitration check on RX is stubbed out |
| EOF timer | TIM3 | Periodic IT | Marks frame end after bus has been idle for the configured window |
| RX pulse capture | TIM2_CH2 | IC2 | Toggles `TIM_INPUTCHANNELPOLARITY_*` every edge |

### 1.2 What firmware does today (post phase 2)

`settings.h` now has `J1850_ENABLED = 1`, so:

- The IC interrupt and EOF timer never start.
- Nothing is parsed and nothing is transmitted.

If the flag is flipped on, what we have is:

| Capability | State |
|---|---|
| Receive raw frames | ✅ working — see `HAL_TIM_IC_CaptureCallback()` in [j1850vpw.cc](Core/Src/j1850vpw.cc) |
| Decode CRC | ✅ `crc1850()` matches the SAE J1850 CRC-8 polynomial |
| Decode VPW headers | ✅ 3-byte / 1-byte mode via the `j1850Header` union |
| Decode payloads | ⚠️ partial — `parseFrame()` extracts RPM (0x1B), SPEED (0x29), MIL (0x88), SIL (0x89), ODO (0x69). DTC fetch, DTC clear, OEM TSM heartbeat, ECM queries — none of these are parsed |
| Bit-bang transmit | ⚠️ `sendByte`/`sendFrame` exist but are never called from the main loop. Arbitration detection is commented out — TX will trample any in-flight frame |
| Source-ID table | ✅ symbolic enum already covers ECM, TSM, BLINKER, MIL, SIL, IMMO, SECURITY, IPC, etc. |
| Higher-level state machine | ❌ missing |
| Diagnostic shell over SWD | ❌ missing (logs out, reads nothing) |

Reference protocol decode (Java, but covers more headers than our parser) is
in [docs/dtc.txt](docs/dtc.txt). The interesting OEM-TSM-related frames:

| Bus header (BE, first 3+1 bytes) | Source | Destination | Meaning |
|---|---|---|---|
| `0x48 0xDA 0x40 0x39` | TSM (`0x40`) | BLINKER (`0xDA`) | Turn-signal state, low 2 bits = L/R |
| `0x68 0x88 0x10 0x03` | ECM (`0x10`) | MIL (`0x88`) | MIL on/off broadcast; `0x83` = on, `0x03` = off, `0x0E` = unknown |
| `0x68 0x89 0x10 0x03` | ECM (`0x10`) | SIL (`0x89`) | SIL on/off (same encoding as MIL) |
| `0x6C 0x00 0xF1 0x14` | tester | ECM | DTC clear request (Mode 0x14 over J1850 / KWP-on-VPW) |
| `0x6C 0xF1 0x00 0x54` | ECM | tester | DTC clear ack |
| `0x6C 0x00 0xF1 0x19` | tester | ECM | DTC read request |
| `0x6C 0xF1 0x00 0x59 …` | ECM | tester | DTC list |

What the ECM expects from the OEM TSM that we **don't** provide is the
periodic `0x48 0xDA 0x40 0x39 <state> <crc>` frame. Absence of that frame
for more than a few hundred ms is what trips the body-CAN missing-module
DTC that latches MIL+SIL on Harleys with the J1850-era ECM.

### 1.3 SWD / RTT — read path

Currently:

- TX only. `_putchar()` in [trace.cc](Core/Src/trace.cc) writes to RTT
  up-channel 0 via `SEGGER_RTT_PutChar()`.
- RX never sampled. The SEGGER driver in `Drivers/SEGGER_RTT/RTT/SEGGER_RTT.c`
  exposes `SEGGER_RTT_HasKey()`, `SEGGER_RTT_GetKey()`, `SEGGER_RTT_Read()`
  but nothing in `Core/` calls them.

Down-channel is already allocated by `SEGGER_RTT_Init()` (default config
gives one bidirectional channel). To enable host→target commands we only
need a polling pump and a tiny line editor.

---

## 2. Spec — SWD/RTT read/write console

Scope: a single text channel the host can use to dump bus traffic, send
hand-crafted J1850 frames, toggle the DTC-clear loop, and read live AHRS
state.

### 2.1 Channel layout

| Channel | Direction | Purpose | Buffer |
|---|---|---|---|
| 0 | up | existing log stream (unchanged) | `SEGGER_RTT_BUFFER_SIZE_UP` |
| 1 | up | binary J1850 trace (frame, μs timestamp, CRC ok/bad) | 1 KiB ring |
| 0 | down | ASCII command line | 128 B |

Two up-channels keeps the human-readable log from being shredded by the
binary trace dump.

### 2.2 Command grammar

Line-based, `\r` or `\n` terminated, case-insensitive, whitespace-separated.

| Command | Effect |
|---|---|
| `help` | print command list |
| `ver` | firmware build/date/git, current settings flags |
| `j1850 on` / `j1850 off` | flip `J1850_ENABLED` at runtime (RAM mirror) |
| `j1850 trace on/off` | gate the binary trace channel |
| `j1850 tx <hex bytes>` | enqueue a single frame (CRC appended by firmware) |
| `j1850 clear` | one-shot DTC clear (`6C 00 F1 14`) |
| `j1850 spoof on/off` | enable/disable the OEM-TSM impersonation loop |
| `ahrs` | current YPR, gyro bias, chip temperature, ZUPT state |
| `ahrs zupt reset` | clear `gyroBiasOnline_` |
| `reset` | software NVIC system reset |

### 2.3 Files to add

| File | Role |
|---|---|
| `Core/Inc/cli.h` | command table + `cliPoll()` prototype |
| `Core/Src/cli.cc` | line buffer, tokenizer, dispatch |
| `Core/Src/trace.cc` | add `_getchar()` and a `traceHasData()` wrapper |
| `Core/Src/tsm.cc` | one `cliPoll()` call per main loop pass |

Acceptance: paste-test from `JLinkRTTClient`/Ozone — typed line round-trips
in <50 ms, frames TX'd via `j1850 tx …` show up on a separate analyzer.

---

## 3. Spec — J1850 ECM-handshake mode

This is the "preferred" path. Goal: emit enough TSM traffic that the ECM
considers the body-control side present and never raises the MIL/SIL DTCs.

### 3.1 What the ECM is waiting for

1. Turn-signal heartbeat from source `0x40` to destination `0xDA`, header
   bytes `0x48 0xDA 0x40 0x39`, payload byte 0 = bitfield
   `(left ? 0x01 : 0) | (right ? 0x02 : 0)`. Send every **100 ms** while
   the ignition is on; the original module sends it whether or not the
   signals are active.
2. Reply to functional addressing query `0x6C 0x40 0xF1 0x3F` ("are you
   alive?") with `0x6C 0xF1 0x40 0x7F` plus a build-string payload. Optional
   on most ECM revisions — confirm on a wireshark trace of a stock bike
   before relying on it.

Nothing else from the OEM TSM matters to the ECM — odometer/RPM/speed are
sourced from the ECM itself or the gauge cluster.

### 3.2 Transmit-side rework

The existing `sendFrame()` busy-waits via `DWT->CYCCNT` and is non-
preemptible. For a 100 ms heartbeat that's fine, but it must:

1. **Re-enable arbitration.** Read PA1 after every passive period; if the
   bus shows active when we are supposed to be passive, abort and retry
   after one `TX_IFS`.
2. **Lock the IC interrupt** during TX (or set a TX-in-flight flag the ISR
   honours) so we don't try to decode our own pulses as inbound.
3. **Run from a software timer** (extend `BLINKER_TIMER` or steal a TIM6
   basic timer) rather than the main-loop cadence — heartbeat jitter must
   be under ~10 ms.

### 3.3 RX-side glue

`parseFrame()` already decodes MIL/SIL. Add:

- DTC list (`0x6C 0xF1 0x00 0x59 …`) so we can log what trouble codes the
  ECM is actually raising and confirm the heartbeat is killing them.
- Hand-off into a tiny `TsmStateMachine` that owns:
  `bool ignitionOn`, `bool ecmAlive`, `uint32_t lastHeartbeatTxMs`,
  `uint8_t lastSignalState`.

### 3.4 Acceptance

Bench:

- With an OBD-II J1850 sniffer attached, our board on the bus alone, every
  100 ms a frame with header `48 DA 40 39` followed by the correct
  state-byte and a CRC that matches the textbook table.

On the bike:

- Cold start, ride 15 minutes, no DTCs in `0x6c00f119` reply.
- Force a left signal — ECM-side logging confirms it sees the right state
  bit transition.

---

## 4. Spec — fallback DTC-clear loop

Used if (a) the bike-specific TSM heartbeat is not figured out, or (b) the
ECM still latches MIL/SIL for a reason unrelated to the missing module.

### 4.1 Behaviour

Every 2 s, send `0x6C 0x00 0xF1 0x14` (clear DTCs). 2 s is long enough that
the bus has time to recover and short enough that the cluster lamp test
(typically 1 s on, blinks twice when latched) never flips the lamp visibly.
Skip the clear cycle for the first 5 s after ignition-on so the OEM lamp
self-test completes — otherwise the rider has no way to know a lamp
filament has blown.

### 4.2 Implementation notes

- Re-uses the same transmit path from §3.2.
- Switchable at runtime via `j1850 spoof on/off` from §2.
- Default: **off**. Reason: the clear cycle interacts badly with the
  diagnostic port when somebody is actually trying to read codes. Keep
  this an opt-in mode that the user enables once they have verified the
  heartbeat path is not going to work for their ECM revision.

### 4.3 Risk

- Some ECM revisions interpret a continuous `clearDTC` stream as an attack
  and latch a tamper code that needs a dealer tool to clear. Mitigation:
  watch for `0x6C 0xF1 0x00 0x54` (clear ack); if the ack stops arriving
  for >10 consecutive clear requests, stop the loop and emit a warning
  over RTT.

---

## 5. Phased rollout

| Phase | Deliverable | Issue / branch |
|---|---|---|
| 0 | This document committed | done (`ba26b9d` push) |
| 1 | RTT down-channel + minimal CLI (§2) | done (`ba26b9d`) |
| 2 | Flip `J1850_ENABLED` to 1; runtime trace toggle; CLI-driven `j1850 tx` and `j1850 clear`; auto-parse every RX frame | done (this commit) |
| 3 | Heartbeat + arbitration-safe TX (§3) | new issue, branch `feat/j1850-tsm-heartbeat` |
| 4 | DTC-list parsing + ECM `clearDTC` ack/tracking | same branch |
| 5 | Fallback DTC-clear loop behind `j1850 spoof` (§4) | new issue, branch `feat/j1850-dtc-loop` |
| 6 | Bike-side verification on the 2005 Sportster, document the actual Delphi ECM revision and any per-bike tuning | merge / docs |

Phases 1–2 are safe to do without bike access — bench sniffer is enough.
Phase 3 requires a known-good capture from a stock bike to lock down the
exact heartbeat payload. Phase 5 is the safety net.

---

## 6. Open questions

1. **Delphi ECM revision** on this specific 2005 Sportster (some XL carb
   bikes shipped with an earlier MT4-class part, later ones with MT4.4).
   Affects DTC code-space mapping in phase 4 — capture a stock bike's
   `0x6c00f119` reply to lock down.
2. **Bus terminator / pull-down state** at idle on this particular harness
   — the MC33390 expects the bus to float low. Verify with a scope before
   enabling phase-3 heartbeat TX so we are not fighting another bus master.
3. **Capture available?** A 30-second OEM bus log with the stock TSM in
   place would massively shorten phase 3.
