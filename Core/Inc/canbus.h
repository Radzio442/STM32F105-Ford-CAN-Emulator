#ifndef CANBUS_H
#define CANBUS_H

#include "main.h"          // zawiera HAL, definicje hcan1/hcan2

extern uint32_t last_4D0_time;
extern uint32_t last_can_activity;
extern uint32_t last_apim_time;

/* Okresowa wysyłka ramek emulatora na CAN2 (0x048, 0x3B2, 0x3B3 itd.). */
void CAN2_TICK(CAN_HandleTypeDef *can2);

/* Obsługa magistrali z APIM/emulatorem:
 * - odbiór ramek APIM (0x048, 0x3B2, 0x3B3 → TEST=0)
 * - wysyłka VIN 0x40A
 * - obsługa przycisków (VOL+/-, RESET, EJECT)
 */
void CAN2_PROCESS(CAN_HandleTypeDef *can2);

/* Odbiór z CAN1 – obecnie monitorowanie aktywności i ramki 0x4D0. */
void CAN1_PROCESS(CAN_HandleTypeDef *can1);

#endif /* CANBUS_H */

