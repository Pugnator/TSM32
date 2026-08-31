// Host tests for the flash-emulated EEPROM (Core/Src/eeprom.cc).
// The firmware source compiles against a RAM flash simulation (the
// STM32F103xB guard) with real flash semantics: erase -> 0xFF, programming
// can only clear bits.
#include "eeprom.h"

#include <cstdio>
#include <cstring>

static int failures = 0;

#define CHECK(cond)                                                        \
  do                                                                       \
  {                                                                        \
    if (!(cond))                                                           \
    {                                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
      failures++;                                                          \
    }                                                                      \
  } while (0)

static constexpr uint32_t kPageSize = 1024;
static constexpr uint32_t kHdrSize = 4;
static constexpr uint32_t kRecSize = 8;
static constexpr uint32_t kRecCount = (kPageSize - kHdrSize) / kRecSize;

static void test_virgin_init_and_basic_rw()
{
  ee_test_reset();
  CHECK(ee_init());

  uint32_t v = 0;
  CHECK(!ee_read(EE_KEY_SECURITY_PIN, &v)); // never written -> absent

  CHECK(ee_write(EE_KEY_SECURITY_PIN, 1234));
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == 1234);

  // Survives a reboot (re-init from flash).
  CHECK(ee_init());
  v = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == 1234);
}

static void test_write_on_change_is_noop()
{
  ee_test_reset();
  CHECK(ee_init());
  CHECK(ee_write(EE_KEY_SECURITY_PIN, 42));
  uint8_t snapshot[kPageSize];
  std::memcpy(snapshot, ee_sim_pages[0], kPageSize);
  CHECK(ee_write(EE_KEY_SECURITY_PIN, 42)); // same value
  CHECK(std::memcmp(snapshot, ee_sim_pages[0], kPageSize) == 0);
}

static void test_last_record_wins()
{
  ee_test_reset();
  CHECK(ee_init());
  for (uint32_t i = 1; i <= 10; ++i)
  {
    CHECK(ee_write(EE_KEY_SECURITY_PIN, i));
  }
  CHECK(ee_init()); // replay from flash
  uint32_t v = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == 10);
}

static void test_compaction_pingpong()
{
  ee_test_reset();
  CHECK(ee_init());
  // Fill page 0 completely, then two more writes: the first triggers
  // compaction into page 1, the second appends there.
  for (uint32_t i = 0; i < kRecCount + 2; ++i)
  {
    CHECK(ee_write(EE_KEY_SECURITY_PIN, 1000 + i));
  }
  uint32_t v = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == 1000 + kRecCount + 1);

  // Old page must have been erased, new page carries the value across boot.
  bool page0Erased = true;
  for (uint32_t i = 0; i < kPageSize; ++i)
  {
    page0Erased = page0Erased && ee_sim_pages[0][i] == 0xFF;
  }
  CHECK(page0Erased);

  CHECK(ee_init());
  v = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == 1000 + kRecCount + 1);
}

static void test_torn_record_skipped()
{
  ee_test_reset();
  CHECK(ee_init());
  CHECK(ee_write(EE_KEY_SECURITY_PIN, 7777));
  CHECK(ee_write(EE_KEY_SECURITY_PIN, 8888));
  // Corrupt the LAST record's CRC (simulates a torn write of the newest
  // value): replay must fall back to the previous record.
  ee_sim_pages[0][kHdrSize + 1 * kRecSize + 6] ^= 0xFF;
  CHECK(ee_init());
  uint32_t v = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == 7777);
}

static void test_interrupted_compaction_recovers()
{
  ee_test_reset();
  CHECK(ee_init());
  for (uint32_t i = 0; i < kRecCount + 1; ++i) // force one compaction
  {
    CHECK(ee_write(EE_KEY_SECURITY_PIN, i));
  }
  // Simulate the power loss window where the new page (1) is complete but
  // the old page (0) was not erased yet: rebuild page 0 as a stale, full,
  // valid store with seq 0.
  std::memset(ee_sim_pages[0], 0xFF, kPageSize);
  ee_sim_pages[0][0] = 0x5A; // magic lo
  ee_sim_pages[0][1] = 0xEE; // magic hi
  ee_sim_pages[0][2] = 0x00; // seq 0 (older than page 1's seq 1)
  ee_sim_pages[0][3] = 0x00;
  CHECK(ee_init());
  uint32_t v = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &v));
  CHECK(v == kRecCount); // the newer page's value
  // Loser page erased on boot.
  bool page0Erased = true;
  for (uint32_t i = 0; i < kPageSize; ++i)
  {
    page0Erased = page0Erased && ee_sim_pages[0][i] == 0xFF;
  }
  CHECK(page0Erased);
}

static void test_multiple_keys()
{
  ee_test_reset();
  CHECK(ee_init());
  const ee_key_t k2 = (ee_key_t)0x02;
  CHECK(ee_write(EE_KEY_SECURITY_PIN, 1111));
  CHECK(ee_write(k2, 2222));
  CHECK(ee_write(EE_KEY_SECURITY_PIN, 3333));
  CHECK(ee_init());
  uint32_t a = 0, b = 0;
  CHECK(ee_read(EE_KEY_SECURITY_PIN, &a));
  CHECK(ee_read(k2, &b));
  CHECK(a == 3333);
  CHECK(b == 2222);
}

int main()
{
  test_virgin_init_and_basic_rw();
  test_write_on_change_is_noop();
  test_last_record_wins();
  test_compaction_pingpong();
  test_torn_record_skipped();
  test_interrupted_compaction_recovers();
  test_multiple_keys();

  if (failures == 0)
  {
    std::printf("test_eeprom: all tests passed\n");
    return 0;
  }
  std::printf("test_eeprom: %d FAILURE(S)\n", failures);
  return 1;
}
