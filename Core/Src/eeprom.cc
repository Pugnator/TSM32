/**
 * @file eeprom.cc
 * @brief Flash-emulated EEPROM - implementation. See eeprom.h for the design.
 *
 * Page layout (1 KB each, base addresses fixed for the 64 KB STM32F103C8):
 *   [0..1]  magic 0xEE5A          (0xFFFF while erased)
 *   [2..3]  sequence number       (higher = newer store; 0xFFFF invalid)
 *   [4.. ]  8-byte records:
 *             [0] key  [1] ~key  [2..5] value (LE)  [6..7] CRC16-CCITT
 *                                                    over bytes [0..5]
 * F103 flash programs by half-word and only clears bits; an erased slot reads
 * all-0xFF (key 0xFF = free, which is why 0xFF is a reserved key).
 */
#include "eeprom.h"
#include <string.h>

#ifdef STM32F103xB
#include "stm32f1xx.h"
#include "watchdog.h"
#include "trace.h"
#else
#include <stdio.h>
#define PrintF(...) ((void)0)
static void watchdog_refresh(void) {}
#endif

#define EE_PAGE_SIZE 1024u
#define EE_MAGIC 0xEE5Au
#define EE_HDR_SIZE 4u
#define EE_REC_SIZE 8u
#define EE_REC_COUNT ((EE_PAGE_SIZE - EE_HDR_SIZE) / EE_REC_SIZE)

#ifdef STM32F103xB
/* Top two pages of the 64 KB part; the linker reserves them
 * (FLASH LENGTH = 62K in STM32F103xx_FLASH.ld). */
static uint8_t *const eePage[2] = {
    (uint8_t *)0x0800F800u,
    (uint8_t *)0x0800FC00u,
};
#else
uint8_t ee_sim_pages[2][EE_PAGE_SIZE];
static uint8_t *const eePage[2] = {ee_sim_pages[0], ee_sim_pages[1]};
#endif

static struct
{
  uint8_t key;
  uint32_t value;
} eeCache[EE_MAX_KEYS];
static uint8_t eeCacheCount = 0;
static int8_t eeActive = -1;   /* index into eePage, -1 = store unusable */
static uint16_t eeNextSlot = 0; /* next free record index in the active page */

/* ---- low-level flash ops (the host/target seam) ------------------------- */

#ifdef STM32F103xB

static bool flashWait(void)
{
  while (FLASH->SR & FLASH_SR_BSY)
  {
  }
  if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
  {
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
    return false;
  }
  FLASH->SR = FLASH_SR_EOP;
  return true;
}

static void flashUnlock(void)
{
  if (FLASH->CR & FLASH_CR_LOCK)
  {
    FLASH->KEYR = 0x45670123u;
    FLASH->KEYR = 0xCDEF89ABu;
  }
}

static void flashLock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

static bool flashErasePage(uint8_t *page)
{
  /* A page erase stalls instruction fetch for ~20-40 ms; keep the IWDG fed
   * on both sides (its ~2 s timeout dwarfs the stall, this is belt-and-
   * braces around the caller's loop cadence). */
  watchdog_refresh();
  flashUnlock();
  FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = (uint32_t)page;
  FLASH->CR |= FLASH_CR_STRT;
  const bool ok = flashWait();
  FLASH->CR &= ~FLASH_CR_PER;
  flashLock();
  watchdog_refresh();
  return ok;
}

static bool flashProgram(uint8_t *dst, const uint8_t *src, uint32_t len)
{
  bool ok = true;
  flashUnlock();
  FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
  for (uint32_t i = 0; ok && i < len; i += 2)
  {
    uint16_t half = (uint16_t)(src[i] | ((uint16_t)src[i + 1] << 8));
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t *)(dst + i) = half;
    ok = flashWait();
    FLASH->CR &= ~FLASH_CR_PG;
    if (ok && *(volatile uint16_t *)(dst + i) != half)
    {
      ok = false; /* verify */
    }
  }
  flashLock();
  return ok;
}

#else /* host simulation: erased = 0xFF, programming can only clear bits */

static bool flashErasePage(uint8_t *page)
{
  memset(page, 0xFF, EE_PAGE_SIZE);
  return true;
}

static bool flashProgram(uint8_t *dst, const uint8_t *src, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i)
  {
    dst[i] &= src[i];
  }
  return memcmp(dst, src, len) == 0;
}

void ee_test_reset(void)
{
  memset(ee_sim_pages, 0xFF, sizeof(ee_sim_pages));
  eeCacheCount = 0;
  eeActive = -1;
  eeNextSlot = 0;
}

#endif

/* ---- record helpers ------------------------------------------------------ */

static uint16_t crc16(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFu;
  for (uint32_t i = 0; i < len; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b)
    {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static uint8_t *recAt(int8_t page, uint16_t slot)
{
  return eePage[page] + EE_HDR_SIZE + (uint32_t)slot * EE_REC_SIZE;
}

static void packRecord(uint8_t out[EE_REC_SIZE], uint8_t key, uint32_t value)
{
  out[0] = key;
  out[1] = (uint8_t)~key;
  out[2] = (uint8_t)(value);
  out[3] = (uint8_t)(value >> 8);
  out[4] = (uint8_t)(value >> 16);
  out[5] = (uint8_t)(value >> 24);
  const uint16_t crc = crc16(out, 6);
  out[6] = (uint8_t)(crc);
  out[7] = (uint8_t)(crc >> 8);
}

/* Valid record: key/~key pair intact, key not reserved, CRC matches. */
static bool recordValid(const uint8_t *r, uint8_t *key, uint32_t *value)
{
  if (r[0] == 0xFFu || r[0] == 0x00u || r[1] != (uint8_t)~r[0])
  {
    return false;
  }
  const uint16_t crc = (uint16_t)(r[6] | ((uint16_t)r[7] << 8));
  if (crc != crc16(r, 6))
  {
    return false;
  }
  *key = r[0];
  *value = (uint32_t)r[2] | ((uint32_t)r[3] << 8) | ((uint32_t)r[4] << 16) |
           ((uint32_t)r[5] << 24);
  return true;
}

static bool slotErased(const uint8_t *r)
{
  for (uint32_t i = 0; i < EE_REC_SIZE; ++i)
  {
    if (r[i] != 0xFFu)
    {
      return false;
    }
  }
  return true;
}

static uint16_t pageSeq(int8_t page)
{
  const uint8_t *p = eePage[page];
  const uint16_t magic = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
  if (magic != EE_MAGIC)
  {
    return 0xFFFFu; /* invalid */
  }
  return (uint16_t)(p[2] | ((uint16_t)p[3] << 8));
}

static bool writeHeader(int8_t page, uint16_t seq)
{
  uint8_t hdr[EE_HDR_SIZE] = {
      (uint8_t)(EE_MAGIC), (uint8_t)(EE_MAGIC >> 8),
      (uint8_t)(seq), (uint8_t)(seq >> 8)};
  return flashProgram(eePage[page], hdr, EE_HDR_SIZE);
}

static void cachePut(uint8_t key, uint32_t value)
{
  for (uint8_t i = 0; i < eeCacheCount; ++i)
  {
    if (eeCache[i].key == key)
    {
      eeCache[i].value = value;
      return;
    }
  }
  if (eeCacheCount < EE_MAX_KEYS)
  {
    eeCache[eeCacheCount].key = key;
    eeCache[eeCacheCount].value = value;
    eeCacheCount++;
  }
}

/* Replay a page into the cache; returns the first free slot index (or
 * EE_REC_COUNT if the page is full). */
static uint16_t replayPage(int8_t page)
{
  uint16_t freeSlot = EE_REC_COUNT;
  for (uint16_t s = 0; s < EE_REC_COUNT; ++s)
  {
    const uint8_t *r = recAt(page, s);
    uint8_t key;
    uint32_t value;
    if (recordValid(r, &key, &value))
    {
      cachePut(key, value);
    }
    else if (slotErased(r))
    {
      /* First erased slot: freeSlot is still EE_REC_COUNT here by construction
       * (it is only ever assigned immediately before the break below). */
      freeSlot = s;
      /* Keep scanning: a torn write can leave a valid record after a
       * skipped slot only if programming reordered, which it does not -
       * but records after a free slot are ignored by construction. */
      break;
    }
    /* Torn/corrupt record: skip, keep scanning for the free space. */
  }
  return freeSlot;
}

/* ---- public API ---------------------------------------------------------- */

bool ee_init(void)
{
  eeCacheCount = 0;
  eeActive = -1;
  eeNextSlot = 0;

  const uint16_t seq0 = pageSeq(0);
  const uint16_t seq1 = pageSeq(1);

  if (seq0 == 0xFFFFu && seq1 == 0xFFFFu)
  {
    /* Virgin (or corrupted) store: format page 0. */
    if (!flashErasePage(eePage[0]) || !writeHeader(0, 0))
    {
      PrintF("EEPROM: format failed - store disabled\r\n");
      return false;
    }
    eeActive = 0;
    eeNextSlot = 0;
    return true;
  }

  /* Higher sequence wins (both valid only after a power loss between the
   * compaction copy and the old page's erase - the copy is complete by
   * construction, so the newer page is safe to prefer). */
  int8_t active;
  if (seq0 == 0xFFFFu)
    active = 1;
  else if (seq1 == 0xFFFFu)
    active = 0;
  else
    active = (int16_t)(uint16_t)(seq0 - seq1) < 0 ? 1 : 0;

  eeActive = active;
  eeNextSlot = replayPage(active);

  /* Finish an interrupted compaction: the losing page must end up erased. */
  const int8_t other = (int8_t)(1 - active);
  if (pageSeq(other) != 0xFFFFu)
  {
    (void)flashErasePage(eePage[other]);
  }

  PrintF("EEPROM: %u key(s), page %d, %u/%u slots used\r\n",
         (unsigned)eeCacheCount, (int)eeActive, (unsigned)eeNextSlot,
         (unsigned)EE_REC_COUNT);
  return true;
}

bool ee_read(ee_key_t key, uint32_t *out)
{
  for (uint8_t i = 0; i < eeCacheCount; ++i)
  {
    if (eeCache[i].key == (uint8_t)key)
    {
      *out = eeCache[i].value;
      return true;
    }
  }
  return false;
}

/* Compact the cache into the other page, then erase the old one. */
static bool compact(void)
{
  const int8_t oldPage = eeActive;
  const int8_t newPage = (int8_t)(1 - eeActive);
  uint16_t newSeq = (uint16_t)(pageSeq(oldPage) + 1u);
  if (newSeq == 0xFFFFu)
  {
    newSeq = 0; /* skip the erased-flash sentinel */
  }

  if (!flashErasePage(eePage[newPage]))
  {
    return false;
  }
  uint16_t slot = 0;
  for (uint8_t i = 0; i < eeCacheCount; ++i, ++slot)
  {
    uint8_t rec[EE_REC_SIZE];
    packRecord(rec, eeCache[i].key, eeCache[i].value);
    if (!flashProgram(recAt(newPage, slot), rec, EE_REC_SIZE))
    {
      return false;
    }
  }
  /* Header last: the new page only becomes the active store once every
   * record is in place. A power loss before this point leaves the old page
   * authoritative. */
  if (!writeHeader(newPage, newSeq))
  {
    return false;
  }
  (void)flashErasePage(eePage[oldPage]);
  eeActive = newPage;
  eeNextSlot = slot;
  return true;
}

bool ee_write(ee_key_t key, uint32_t value)
{
  /* Compare on the raw value: casting 0x00/0xFF *into* ee_key_t would be an
   * out-of-range enum conversion (unspecified/UB in C++), whereas enum -> int
   * is always well defined (PVS V1016/V560). */
  const unsigned rawKey = (unsigned)key;
  if (eeActive < 0 || rawKey == 0x00u || rawKey == 0xFFu)
  {
    return false;
  }

  uint32_t current;
  if (ee_read(key, &current) && current == value)
  {
    return true; /* write-on-change */
  }

  /* Cache first (also feeds compaction). Reject overflow before touching
   * flash so the store never holds a key the cache cannot replay. */
  bool cached = false;
  for (uint8_t i = 0; i < eeCacheCount && !cached; ++i)
  {
    cached = eeCache[i].key == (uint8_t)key;
  }
  if (!cached && eeCacheCount >= EE_MAX_KEYS)
  {
    return false;
  }
  cachePut((uint8_t)key, value);

  if (eeNextSlot >= EE_REC_COUNT)
  {
    /* Compaction copies the cache - which already holds the new value -
     * into the fresh page, so the write is complete when it succeeds. */
    return compact();
  }

  uint8_t rec[EE_REC_SIZE];
  packRecord(rec, (uint8_t)key, value);
  if (!flashProgram(recAt(eeActive, eeNextSlot), rec, EE_REC_SIZE))
  {
    return false;
  }
  eeNextSlot++;
  return true;
}
