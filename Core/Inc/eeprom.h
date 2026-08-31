/**
 * @file eeprom.h
 * @brief Flash-emulated EEPROM - key/value store for rarely-written settings
 *        (security PIN, future configuration).
 *
 * The board has no external EEPROM (the I2C peripheral was removed), so two
 * 1 KB pages at the top of the STM32F103C8's 64 KB flash are reserved by the
 * linker script (FLASH LENGTH = 62K) and used as a ping-pong record store:
 *
 *   - Records are appended to the active page; the last valid record for a
 *     key wins. A write with an unchanged value is a no-op (write-on-change).
 *   - When the active page is full, the latest value of every key is compacted
 *     into the other page FIRST, then the old page is erased - there is never
 *     a moment without a valid copy, so a power loss mid-compaction cannot
 *     lose the store (the page header carries a sequence number; the higher
 *     one wins on boot).
 *   - Each record is CRC-protected; a torn record (power loss mid-write) fails
 *     the CRC and is skipped.
 *
 * Wear: one page erase per ~120 writes, against a 10 kcycle endurance spec -
 * effectively unlimited for settings written a handful of times a year.
 *
 * Timing: ee_write() programs flash synchronously. While the flash is busy
 * the CPU stalls on instruction fetch (a page erase is ~20-40 ms), so J1850
 * frames on the bus during a compaction are lost and re-synced afterwards.
 * Call ee_write() only from user-initiated, standstill contexts (settings /
 * PIN entry), never from the fast path.
 *
 * Register-level (CMSIS) like watchdog.c: no HAL_FLASH module enable, so a
 * CubeMX regeneration cannot break it. Host tests build the same code against
 * a RAM-backed flash simulation (the STM32F103xB guard is the seam).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    /* 0x00 and 0xFF are reserved (0xFF = erased flash). */
    EE_KEY_SECURITY_PIN = 0x01, /**< u32-packed security PIN; absent = PIN not set */
  } ee_key_t;

/** Distinct keys the RAM cache can hold. Raise when keys are added. */
#define EE_MAX_KEYS 8u

  /** Scan both pages, pick the active one and load the newest value of every
   *  key into the RAM cache. Call once at boot, before any read. Returns
   *  false only if neither page carries a valid header AND formatting the
   *  store failed (flash fault) - reads then report "absent" and writes fail. */
  bool ee_init(void);

  /** Read a key from the RAM cache. Returns false if the key has never been
   *  written (caller applies its default). */
  bool ee_read(ee_key_t key, uint32_t *out);

  /** Persist a key. Write-on-change; appends a record, compacting to the
   *  other page when full. Blocking (see timing note above). Returns false
   *  on a flash fault or cache overflow (more than EE_MAX_KEYS keys). */
  bool ee_write(ee_key_t key, uint32_t value);

#ifndef STM32F103xB
  /* Host-test hooks: the RAM-backed flash simulation. */
  extern uint8_t ee_sim_pages[2][1024];
  void ee_test_reset(void); /* erase both sim pages and forget cache state */
#endif

#ifdef __cplusplus
}
#endif
