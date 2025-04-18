#ifndef CRSF_H
#define CRSF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define CRSF_MAX_FRAME_LEN 64
#define CRSF_ADDR_FLIGHT_CONTROLLER  0xC8  // Device address for FC (sync byte)&#8203;:contentReference[oaicite:13]{index=13}
#define CRSF_PAYLEN_BATTERY_SENSOR   8

#define CRSF_FRAMETYPE_BATTERY_SENSOR           0x08
#define CRSF_FRAME_START                        0xC8
#define CRSF_FRAME_SIZE_BATTERY                 8
#define CRSF_ADDRESS_FLIGHT_CONTROLLER          0xC8
#define CRSF_ADDRESS_TRANSMITTER                0xEA
#define CRSF_ADDRESS_BROADCAST                  0x00
#define CRSF_FRAME_LENGTH  0x0A


extern uint8_t dmaRxBuf[CRSF_MAX_FRAME_LEN];
extern uint8_t frameBuf[64];
extern uint16_t frameLength;
extern uint16_t rcChannels[16];



typedef struct {
    uint16_t voltage;     // Napięcie * 10 (np. 123 = 12.3V)
    uint16_t current;     // Prąd * 10 (np. 142 = 14.2A)
    uint32_t capacity;    // Pojemność zużyta w mAh (24 bity)
    uint8_t remaining;    // Pozostała pojemność w % (0-100)
} BatteryData_t;


void processCRSFframe(uint8_t *data, uint16_t len);
uint8_t CRSF_CalcCRC(const uint8_t *data, uint8_t length);
HAL_StatusTypeDef CRSF_SendBatteryData(BatteryData_t *batteryData);
void CRSF_SendBatteryTelemetry_IT(uint16_t voltage_mv, uint16_t current_ma, uint32_t capacity_used, uint8_t battery_remaining);

#ifdef __cplusplus
}
#endif

#endif // CRSF_H
