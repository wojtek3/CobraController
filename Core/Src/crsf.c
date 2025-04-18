#include "crsf.h"
#include <string.h>
#include "usart.h"

uint8_t dmaRxBuf[CRSF_MAX_FRAME_LEN];
uint8_t frameBuf[64];
uint16_t frameLength;
uint16_t rcChannels[16];


uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }
    return crc;
}

HAL_StatusTypeDef CRSF_SendBatteryData(BatteryData_t *batteryData) {
    // Total frame size: 1 (start) + 1 (length) + 1 (type) + 8 (payload) + 1 (CRC) = 12 bytes
    uint8_t frame[12];

    // Header
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // Start byte (0xC8)
    frame[1] = CRSF_FRAME_LENGTH;                // Length = 10 (bytes following)
    frame[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;      // Frame type (0x08)

    // Payload
    // Voltage (2 bytes, big-endian)
    frame[3] = (batteryData->voltage >> 8) & 0xFF;
    frame[4] = batteryData->voltage & 0xFF;

    // Current (2 bytes, big-endian)
    frame[5] = (batteryData->current >> 8) & 0xFF;
    frame[6] = batteryData->current & 0xFF;

    // Capacity (3 bytes, big-endian, using lower 24 bits)
    frame[7] = (batteryData->capacity >> 16) & 0xFF;
    frame[8] = (batteryData->capacity >> 8) & 0xFF;
    frame[9] = batteryData->capacity & 0xFF;

    // Remaining battery percentage (1 byte)
    frame[10] = batteryData->remaining;

    // CRC calculated over bytes from index 2 to index 10 (9 bytes)
    frame[11] = crsf_crc8(&frame[2], 9);

    // Transmit the frame via UART (12 bytes total)
    return HAL_UART_Transmit(&huart6, frame, sizeof(frame), 100);
}

static inline uint16_t scale_channel(uint16_t raw) {
    // Rescale from [191, 1792] to [900, 2100]
    return (((raw - 191) * 1200) / 1601) + 900;
}

void processCRSFframe(uint8_t *data, uint16_t len) {
    if (len < 5) return;  // minimal frame size (addr + len + type + 1-byte CRC)
    uint8_t addr   = data[0];
    uint8_t length = data[1];  // length of Type+Payload+CRC
    uint8_t type   = data[2];
    // Validate frame length
    if (length != (len - 2)) {
        // Length mismatch – discard frame (or handle error)
        return;
    }
    // Optional: verify CRC (last byte) using CRSF CRC8 polynomial 0xD5
    // -- omitted for brevity --
    if (type == 0x16) {  // RC Channels Packed
        if (length < (1 + 22 + 1)) return; // (Type + 22-byte payload + CRC) expected
        // Point to the 22-byte channel data payload (after Type byte)
        uint8_t *payload = &data[3];
        // Define a packed struct matching 16x 11-bit channel fields
        #pragma pack(push,1)
        struct Channels {
            unsigned ch0  : 11;
            unsigned ch1  : 11;
            unsigned ch2  : 11;
            unsigned ch3  : 11;
            unsigned ch4  : 11;
            unsigned ch5  : 11;
            unsigned ch6  : 11;
            unsigned ch7  : 11;
            unsigned ch8  : 11;
            unsigned ch9  : 11;
            unsigned ch10 : 11;
            unsigned ch11 : 11;
            unsigned ch12 : 11;
            unsigned ch13 : 11;
            unsigned ch14 : 11;
            unsigned ch15 : 11;
        };
        #pragma pack(pop)
        const struct Channels *ch = (const struct Channels *)payload;

        rcChannels[0]  = ch->ch0;
        rcChannels[1]  = ch->ch1;
        rcChannels[2]  = ch->ch2;
        rcChannels[3]  = ch->ch3;
        rcChannels[4]  = ch->ch4;
        rcChannels[5]  = ch->ch5;
        rcChannels[6]  = ch->ch6;
        rcChannels[7]  = ch->ch7;
        rcChannels[8]  = ch->ch8;
        rcChannels[9]  = ch->ch9;
        rcChannels[10]  = ch->ch10;
        rcChannels[11]  = ch->ch11;
        rcChannels[12]  = ch->ch12;
        rcChannels[13]  = ch->ch13;
        rcChannels[14]  = ch->ch14;
        rcChannels[15] = ch->ch15;

        for (uint8_t i = 0; i < 16; i++) {
                    rcChannels[i] = scale_channel(rcChannels[i]);
                }
    }
    else {
        // Handle other frame types (telemetry, link stats, etc.)
        // e.g., 0x1C = Link statistics, 0x02 = Battery telemetry, etc.
    }
}
