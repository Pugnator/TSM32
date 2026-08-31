#include "test_env.h"
#include "../Core/Inc/j1850.h"

#include <cstdio>

bool mil = false;
bool milAux = false;
bool sil = false;
bool silAux = false;
uint8_t dtc = 0;
uint32_t trip = 0;
int8_t gear_num = 0;
bool in_neutral = false;
bool clutch_engaged = false;
uint8_t turn_signals = 0;
uint8_t engine_temp_f = 0;
uint32_t fuel_ticks = 0;
uint8_t fuel_gauge_level = 0;
bool passwordDtcSeen = false;
volatile bool securityPollPending = false;
bool ecmSeen = false;
bool bcmDtcSeen = false;
bool ipcDtcSeen = false;
uint8_t payloadJ1850[J1850_PAYLOAD_SIZE] = {};
volatile uint8_t j1850RXctr = 0;
volatile uint32_t j1850DroppedFrames = 0;

namespace
{
int failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

void loadFrame(const uint8_t *data, uint8_t lengthWithoutCrc)
{
    for (uint8_t i = 0; i < J1850_PAYLOAD_SIZE; ++i)
        payloadJ1850[i] = 0;
    for (uint8_t i = 0; i < lengthWithoutCrc; ++i)
        payloadJ1850[i] = data[i];
    payloadJ1850[lengthWithoutCrc] = crc1850(data, lengthWithoutCrc);
    j1850RXctr = lengthWithoutCrc + 1;
}

void resetRpmSignal()
{
    rpms = 111;
    rpmLastUpdateTick = 7;
    rpmSignalSeen = false;
}

void testRpmSchemaValidation()
{
    const uint8_t valid[] = {0x28, 0x1B, 0x10, 0x02, 0x0A, 0xF0}; // 700 RPM
    fakeTick = 1234;
    resetRpmSignal();
    loadFrame(valid, sizeof(valid));
    CHECK(J1850VPW::parseFrame());
    CHECK(rpms == 700);
    CHECK(rpmSignalSeen);
    CHECK(rpmLastUpdateTick == fakeTick);

    uint8_t wrongSource[] = {0x28, 0x1B, 0x61, 0x02, 0x0A, 0xF0};
    resetRpmSignal();
    loadFrame(wrongSource, sizeof(wrongSource));
    CHECK(J1850VPW::parseFrame());
    CHECK(rpms == 111);
    CHECK(!rpmSignalSeen);
    CHECK(rpmLastUpdateTick == 7);

    uint8_t wrongSubtype[] = {0x28, 0x1B, 0x10, 0x03, 0x0A, 0xF0};
    resetRpmSignal();
    loadFrame(wrongSubtype, sizeof(wrongSubtype));
    CHECK(J1850VPW::parseFrame());
    CHECK(rpms == 111);
    CHECK(!rpmSignalSeen);

    uint8_t wrongLength[] = {0x28, 0x1B, 0x10, 0x02, 0x0A, 0xF0, 0x00};
    resetRpmSignal();
    loadFrame(wrongLength, sizeof(wrongLength));
    CHECK(J1850VPW::parseFrame());
    CHECK(rpms == 111);
    CHECK(!rpmSignalSeen);
}

void testSpeedSchemaValidation()
{
    const uint8_t valid[] = {0x48, 0x29, 0x10, 0x02, 0x05, 0x00}; // 10 km/h
    fakeTick = 4321;
    kph = 99;
    speedLastUpdateTick = 8;
    speedSignalSeen = false;
    loadFrame(valid, sizeof(valid));
    CHECK(J1850VPW::parseFrame());
    CHECK(kph == 10);
    CHECK(speedSignalSeen);
    CHECK(speedLastUpdateTick == fakeTick);

    uint8_t wrongSubtype[] = {0x48, 0x29, 0x10, 0x12, 0x05, 0x00};
    kph = 99;
    speedLastUpdateTick = 8;
    speedSignalSeen = false;
    loadFrame(wrongSubtype, sizeof(wrongSubtype));
    CHECK(J1850VPW::parseFrame());
    CHECK(kph == 99);
    CHECK(!speedSignalSeen);
    CHECK(speedLastUpdateTick == 8);
}
} // namespace

extern "C" uint8_t crc1850(const uint8_t *msgBuf, uint8_t nbytes)
{
    if (nbytes == 0 || nbytes > 11)
        return 0;

    uint8_t crc = 0xFF;
    for (uint8_t byteIndex = 0; byteIndex < nbytes; ++byteIndex)
    {
        for (uint8_t bit = 0x80; bit != 0; bit >>= 1)
        {
            uint8_t polynomial;
            if ((msgBuf[byteIndex] & bit) != 0)
            {
                polynomial = (crc & 0x80) ? 0x01 : 0x1C;
                crc = static_cast<uint8_t>(((crc << 1) | 1) ^ polynomial);
            }
            else
            {
                polynomial = (crc & 0x80) ? 0x1D : 0x00;
                crc = static_cast<uint8_t>((crc << 1) ^ polynomial);
            }
        }
    }
    return static_cast<uint8_t>(~crc);
}

int main()
{
    testRpmSchemaValidation();
    testSpeedSchemaValidation();
    std::printf("J1850 parser tests: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
