/*
* Copyright (c) 2014, Richard Buerk
* All rights reserved.

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the <organization> nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL JENS NIELSEN BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "stm32f4xx.h"
#include "tsm.h"
#include "gpio.h"
#include "1wire.h"

#define STM32F4XX

void SetPinInput(ONE_WIRE_PIN *pin);
void SetPinOutput(ONE_WIRE_PIN *pin);
uint8_t PinRead(ONE_WIRE_PIN *pin);
void SetPinHigh(ONE_WIRE_PIN *pin);
void SetPinLow(ONE_WIRE_PIN *pin);

void OneWireWriteBit(ONE_WIRE_PIN *pin, uint8_t val);
uint8_t OWRead_bit(ONE_WIRE_PIN *pin);

uint8_t ow_adr[8];

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Low Level functions
//

void SetPinInput(ONE_WIRE_PIN *pin)
{
    *pin->pModeReg &= ~pin->modeRegMask;
    *pin->pModeReg |= pin->modeInputMask;
}

void SetPinOutput(ONE_WIRE_PIN *pin)
{
    *pin->pModeReg &= ~pin->modeRegMask;
    *pin->pModeReg |= pin->modeOutputMaske;
}

uint8_t PinRead(ONE_WIRE_PIN *pin)
{
    return ((uint8_t)((pin->pPort->IDR & pin->bPin) > 0 ? 1 : 0));
}

void SetPinHigh(ONE_WIRE_PIN *pin)
{
#ifdef STM32F4XX
    //pin->pPort->BSRRL = pin->bPin;
#else
    pin->pPort->BSRR = pin->bPin;
#endif
}

void SetPinLow(ONE_WIRE_PIN *pin)
{
#ifdef STM32F4XX
    //pin->pPort->BSRRH = pin->bPin;
#else
    pin->pPort->BRR = pin->bPin;
#endif
}

// =====================================================================================
// Medium layer functions
void OneWireWriteBit(ONE_WIRE_PIN *pin, uint8_t bitVal)
{
    if (bitVal)
    {
        __disable_irq();
        SetPinLow(pin);
        SetPinOutput(pin); // drive output low
        DWT_Delay(10);
        SetPinHigh(pin); // drive output high
        __enable_irq();
        DWT_Delay(50);
    }
    else
    {
        __disable_irq();
        SetPinLow(pin);
        SetPinOutput(pin); // drive output low
        DWT_Delay(65);
        SetPinHigh(pin); // drive output high
        __enable_irq();
        DWT_Delay(5);
    }
}

uint8_t OWRead_bit(ONE_WIRE_PIN *pin)
{
    uint8_t r;

    __disable_irq();
    SetPinLow(pin);
    SetPinOutput(pin);
    DWT_Delay(3);
    SetPinInput(pin); // let pin float, pull up will raise
    DWT_Delay(10);
    r = PinRead(pin);
    __enable_irq();
    DWT_Delay(55);
    return (r);
}

// =====================================================================================

void OneWireInit(ONE_WIRE *owire, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    OneWireResetSearch(owire);
    OneWireSearch(owire, ow_adr);
}

uint8_t OneWireReset(ONE_WIRE *owire)
{
    uint8_t r;
    uint8_t retries = 125;

    SetPinInput(&owire->pin);

    // wait until the wire is high... just in case
    do
    {
        if (--retries == 0)
        {
            return (0);
        }

        DWT_Delay(2);
    } while (!PinRead(&owire->pin));

    SetPinLow(&owire->pin);    // Set output
    SetPinOutput(&owire->pin); // Set pin low

    DWT_Delay(500);

    __disable_irq();
    SetPinInput(&owire->pin);
    DWT_Delay(80);
    r = !PinRead(&owire->pin);
    __enable_irq();
    DWT_Delay(440);
    return (r);
}

void OneWireWrite(ONE_WIRE *owire, uint8_t val)
{
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        OneWireWriteBit(&owire->pin, bitMask & val);
    }

    SetPinInput(&owire->pin);
}

void OneWireWritePowerOn(ONE_WIRE *owire, uint8_t val)
{
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        OneWireWriteBit(&owire->pin, bitMask & val);
    }
}

uint8_t OneWireRead(ONE_WIRE *owire)
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        if (OWRead_bit(&owire->pin))
        {
            r |= bitMask;
        }
    }

    return (r);
}

void OneWireWriteBytes(ONE_WIRE *owire, const uint8_t *buf, uint16_t count)
{
    uint16_t i;

    for (i = 0; i < count; i++)
    {
        OneWireWrite(owire, buf[i]);
    }

    SetPinInput(&owire->pin);
    SetPinLow(&owire->pin);
}

void OneWireReadBytes(ONE_WIRE *owire, uint8_t *buf, uint16_t count)
{
    uint16_t i;

    for (i = 0; i < count; i++)
    {
        buf[i] = OneWireRead(owire);
    }
}

void OneWireSelect(ONE_WIRE *owire, uint8_t const *rom)
{
    int i;

    OneWireWrite(owire, ONEWIRE_CMD_SELECT); // Choose ROM

    for (i = 0; i < 8; i++)
    {
        OneWireWrite(owire, rom[i]);
    }
}

void OneWireSkipRom(ONE_WIRE *owire)
{
    OneWireWrite(owire, ONEWIRE_CMD_SKIP); // Skip ROM
}

void OneWireResetSearch(ONE_WIRE *owire)
{
    int i;
    // reset the search state
    owire->LastDiscrepancy = 0;
    owire->LastDeviceFlag = FALSE;
    owire->LastFamilyDiscrepancy = 0;

    for (i = 0; i < 7; i++)
    {
        owire->ROM_NO[i] = 0;
    }
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.
//
// --- Search Code from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
// crc8 code removed
uint8_t OneWireSearch(ONE_WIRE *owire, uint8_t *newAddr)
{
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number, search_result;
    uint8_t id_bit, cmp_id_bit;
    int i;
    unsigned char rom_byte_mask, search_direction;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    // if the last call was not the last one
    if (!owire->LastDeviceFlag)
    {
        // 1-Wire reset
        if (!OneWireReset(owire))
        {
            // reset the search
            owire->LastDiscrepancy = 0;
            owire->LastDeviceFlag = FALSE;
            owire->LastFamilyDiscrepancy = 0;
            return (FALSE);
        }

        // issue the search command
        OneWireWrite(owire, ONEWIRE_CMD_SEARCH);

        // loop to do the search
        do
        {
            // read a bit and its complement
            id_bit = OWRead_bit(&owire->pin);
            cmp_id_bit = OWRead_bit(&owire->pin);

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1))
            {
                break;
            }
            else
            {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                {
                    search_direction = id_bit; // bit write value for search
                }
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < owire->LastDiscrepancy)
                    {
                        search_direction = ((owire->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    }
                    else
                    // if equal to last pick 1, if not then pick 0
                    {
                        search_direction = (id_bit_number == owire->LastDiscrepancy);
                    }

                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                        {
                            owire->LastFamilyDiscrepancy = last_zero;
                        }
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                {
                    owire->ROM_NO[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    owire->ROM_NO[rom_byte_number] &= ~rom_byte_mask;
                }

                // serial number search direction write bit
                OneWireWriteBit(&owire->pin, search_direction);
                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65))
        {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            owire->LastDiscrepancy = last_zero;

            // check for last device
            if (owire->LastDiscrepancy == 0)
            {
                owire->LastDeviceFlag = TRUE;
            }

            search_result = TRUE;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !owire->ROM_NO[0])
    {
        owire->LastDiscrepancy = 0;
        owire->LastDeviceFlag = FALSE;
        owire->LastFamilyDiscrepancy = 0;
        search_result = FALSE;
    }

    for (i = 0; i < 8; i++)
    {
        newAddr[i] = owire->ROM_NO[i];
    }

    return (search_result);
}

#ifdef ONEWIRE_CRC
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

// table from Dallas sample code where it is freely reusable,
// Copyright (C) 2000 Dallas Semiconductor Corporation
const uint8_t dscrc_table[] =
    {
        0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
        35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
        190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
        70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
        219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
        101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
        248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
        140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
        17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
        175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
        50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
        202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
        87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
        233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
        116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

// Compute 8 bit CRC. These show up in the ROM
uint8_t CalcCRC8(uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc = dscrc_table[(crc ^ *addr++)];
    }

    return (crc);
}

#endif
