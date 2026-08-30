#include "canbus.h"
#include <string.h>

/* =================== KONFIG / KONSTANTY =================== */

#define TICK_INTERVAL_MS        10U
#define APIM_TEST_TIMEOUT_MS    500U
#define CAN_HEALTH_TIMEOUT_MS   2000U

#define KEY_HOLD_THRESHOLD_MS   300U
#define KEY_REPEAT_INTERVAL_MS  120U
#define EJECT_HOLD_THRESHOLD_MS 600U

#define RESET_HOLD_MS           2000U   // RESET SYNC TUNE+ >2s

/* „PRO” wysyłanie – krótka próba, bez wiecznego blokowania */
#define CAN_TX_TIMEOUT_MS       2U      // max czekania na wolny mailbox
#define CAN_TX_MAX_ATTEMPTS     2U      // max prób wysyłki

/* =================== EXTERNAL HANDLES =================== */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* =================== ZMIENNE GLOBALNE =================== */

static CAN_RxHeaderTypeDef RxCAN;
static uint8_t  RxDATA[8];

static uint32_t TICK = 0;

static uint8_t KEY15_LAST = 1; // zakomentuj do trzech przycisków
static uint8_t KEY10_LAST = 1;
static uint8_t KEY11_LAST = 1;
static uint8_t KEY12_LAST = 1;

static uint8_t  SEND = 0;
static uint8_t  LAMP = 0;
static uint8_t  TEST = 1;      // 1 = tryb emulatora (brak APIM)
static uint8_t  FLIP = 1;      // VIN on/off
static uint8_t  WORD = 0;

uint32_t last_4D0_time = 0;
uint32_t last_can_activity = 0;
uint32_t last_apim_time = 0;


static const char VIN[] = "WF0JXXWPCJFM57486";

_Static_assert(sizeof(VIN) == 18U, "VIN must contain exactly 17 characters");

static uint8_t  vin_step     = 0;
static uint32_t last_vin_time = 0;
static uint8_t  vin_frame0[8];
static uint8_t  vin_frame1[8];
static uint8_t  vin_frame2[8];

typedef struct {
    uint8_t  d[8];   // dane
    uint16_t id;     // StdId
} EMU_FRAME;

static const EMU_FRAME emu_frames[] = {

    {{0x00,0x00,0x00,0x00,0x04,0x00,0xE0,0x00}, 0x048},
    {{0x44,0x88,0xC2,0x0C,0x10,0x00,0x00,0x02}, 0x3B2},
    {{0x44,0x88,0x00,0x0C,0x82,0x00,0x01,0x02}, 0x3B3},
    {{0x34,0xFF,0xFF,0xFF,0x00,0x00,0x1E,0x00}, 0x2A0},
    {{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 0x081},
    {{0x61,0x06,0x41,0x50,0x01,0x94,0x04,0xA0}, 0x360},
    {{0x05,0x4D,0x31,0x38,0x31,0x38,0x00,0x00}, 0x361},
    {{0x02,0x00,0x00,0x18,0x00,0x00,0x00,0x00}, 0x04A},
    {{0x00,0x00,0x80,0x03,0x80,0x05,0x00,0x00}, 0x200},
    {{0x20,0x55,0x51,0x40,0x00,0x00,0x00,0x00}, 0x04C},
    {{0x95,0x00,0x00,0x00,0x03,0x00,0x00,0x00}, 0x156},
    {{0x14,0x60,0x00,0x00,0x03,0x00,0x00,0x00}, 0x171},
    {{0x00,0x00,0x00,0x00,0x76,0x00,0x03,0xD1}, 0x179},
    {{0xFF,0xFF,0x00,0x21,0x80,0x01,0xFF,0x41}, 0x213},
    {{0x01,0x00,0x51,0x0E,0x00,0x00,0x00,0x00}, 0x312},
    {{0x20,0x00,0x15,0xFF,0x02,0x9A,0x82,0xEE}, 0x365},
    {{0x40,0x11,0x11,0xFF,0x23,0x20,0x00,0x00}, 0x3B4},
	{{0x01,0x02,0x00,0xFF,0x00,0xFF,0x00,0xFF}, 0x3B5},
    {{0x5C,0xC0,0x00,0x00,0x00,0x00,0x00,0x00}, 0x3C3},
    {{0x00,0x01,0x08,0x09,0x80,0xC5,0x31,0x30}, 0x3D8},
    {{0x51,0x00,0xFD,0x04,0x01,0x00,0x00,0x00}, 0x416},
    {{0x03,0x00,0x28,0x00,0x08,0x98,0x00,0xFF}, 0x421},
    {{0x76,0x10,0x00,0x1C,0x35,0x05,0x0C,0x00}, 0x166},
    {{0x04,0xF2,0x50,0x00,0x60,0x00,0x00,0x00}, 0x202},
    {{0xDC,0x00,0x7D,0xC1,0x91,0xF5,0x00,0x00}, 0x204},
    {{0x16,0x52,0x65,0x32,0xFC,0xD5,0x12,0x20}, 0x3CD},
    {{0x72,0x80,0x00,0x18,0x00,0x1A,0x01,0x00}, 0x167},
    {{0x00,0x17,0xA6,0x00,0x00,0x00,0x00,0x00}, 0x42C},
    {{0xCD,0x04,0x00,0x01,0x90,0x00,0x00,0x00}, 0x42F},
    {{0x78,0x01,0x00,0x00,0x00,0x01,0x41,0x01}, 0x077},
    {{0x3A,0x20,0x00,0x00,0x00,0x00,0x00,0x00}, 0x2FD},
    {{0x00,0x20,0x18,0x01,0x11,0x00,0x00,0x00}, 0x18A},
    {{0x00,0x00,0x04,0x06,0x00,0x04,0x00,0x00}, 0x082},
    {{0x0E,0x40,0x0A,0x04,0x5F,0x00,0x00,0x00}, 0x3A7},
    {{0x0E,0x40,0x0A,0x04,0x5F,0x00,0x00,0x00}, 0x3A6},
    {{0x00,0x02,0x04,0x06,0x80,0x04,0x64,0x00}, 0x3E3},
    {{0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00}, 0x2A1}
};

static uint32_t key15_time = 0, key10_time = 0, key11_time = 0, key12_time = 0;
static uint8_t  key15_hold = 0, key10_hold = 0, key11_hold = 0, key12_hold = 0;

/* =================== PROTOTYPY LOKALNE =================== */

static void DATA_TO_NULL(CAN_HandleTypeDef *can);
static void DATA_TO_SEND(CAN_HandleTypeDef *can, const uint16_t data[9]);
static void DATA_TO_KEYS(CAN_HandleTypeDef *can2);
static void PREPARE_VIN_FRAMES(void);
static void CHECK_CAN_HEALTH(void);

/* ============================================================
   CAN2_TICK — okresowe ramki emulatora na CAN2
   - wywoływać z while(1): CAN2_TICK(&hcan2);
============================================================ */
void CAN2_TICK(CAN_HandleTypeDef *can2)
{
    if (HAL_GetTick() - TICK <= TICK_INTERVAL_MS)
        return;

    TICK = HAL_GetTick();
    SEND++;

    if (SEND == 90U)
        SEND = 10U;

    /* 37 wpisów odpowiada parzystym krokom 12, 14, ..., 84. */
    if (TEST == 1U && SEND >= 12U && SEND <= 84U && (SEND % 2U) == 0U)
    {
        const uint8_t frame_index = (uint8_t)((SEND - 12U) / 2U);
        const EMU_FRAME *f = &emu_frames[frame_index];

        uint16_t Tx[9] = {
            f->d[0], f->d[1], f->d[2], f->d[3],
            f->d[4], f->d[5], f->d[6], f->d[7],
            f->id
        };

        DATA_TO_SEND(can2, Tx);
    }
    /* ------------------------------------------
       BRAK 4D0 → WYŚLIJ 1E6 CO 100 ms
    -------------------------------------------*/
    if (HAL_GetTick() - last_4D0_time > 5000U)   // 5 sekund
    {
        if (SEND % 10 == 0)   // co 100 ms
        {
            uint16_t E6A[9] = {0x80,0x0C,0xE4,0x07,0x00,0x00,0x00,0x00,0x1E6};
            uint16_t E6B[9] = {0x80,0x0C,0xE8,0x07,0x00,0x00,0x00,0x00,0x1E6};
            uint16_t E6C[9] = {0x51,0x01,0x32,0x09,0xC2,0x38,0x48,0x00,0x2D5};

            DATA_TO_SEND(can2, E6A);
            DATA_TO_SEND(can2, E6B);
            DATA_TO_SEND(can2, E6C);
        }
    }

    /* LED status */
    LAMP++;
    {
        uint16_t blink_limit = (TEST == 1U ? 10U : 70U);
        if (LAMP > blink_limit)
        {
            LAMP = 0;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
        }
    }

    /* jeśli długo nie ma ramek od APIM → powrót do TEST=1 */
    if (HAL_GetTick() - last_apim_time > APIM_TEST_TIMEOUT_MS)
    {
        TEST = 1U;
    }

    CHECK_CAN_HEALTH();
}

/* ============================================================
   CAN1_PROCESS — monitorowanie aktywności CAN1 i ramki 0x4D0
============================================================ */
void CAN1_PROCESS(CAN_HandleTypeDef *can1)
{
    if (HAL_CAN_GetRxFifoFillLevel(can1, CAN_RX_FIFO1) == 0U)
        return;

    if (HAL_CAN_GetRxMessage(can1, CAN_RX_FIFO1, &RxCAN, RxDATA) == HAL_OK)
    {
        last_can_activity = HAL_GetTick();

        if (RxCAN.StdId == 0x4D0)
        {
            last_4D0_time = HAL_GetTick();   // ACM żyje
        }
    }
}

/* ============================================================
   CAN2_PROCESS — obsługa CAN2 z APIM + VIN + przyciski
============================================================ */
void CAN2_PROCESS(CAN_HandleTypeDef *can2)
{
    /* Odbiór ramek od APIM (048, 3B2, 3B3) → wyłączamy emulator (TEST=0) */
    while (HAL_CAN_GetRxFifoFillLevel(can2, CAN_RX_FIFO0) != 0U)
    {
        if (HAL_CAN_GetRxMessage(can2, CAN_RX_FIFO0, &RxCAN, RxDATA) == HAL_OK)
        {
            last_can_activity = HAL_GetTick();

            if (RxCAN.StdId == 0x048 ||
                RxCAN.StdId == 0x3B2 ||
                RxCAN.StdId == 0x3B3)
            {
                TEST = 0U;
                last_apim_time = HAL_GetTick();
            }
            if (RxCAN.StdId == 0x3DA)
            {
                uint8_t nib = RxDATA[0] & 0xF0;

                if (nib != WORD)
                {
                    WORD = nib;
                    SEND = 0;

                    switch (nib)
                    {
                        case 0x30:
                            DATA_TO_SEND(can2, (uint16_t[9]){0x00,0x08,0x00,0xF8,0x00,0x00,0x00,0x00,0x1DC});
                            break;

                        case 0x40:
                            DATA_TO_SEND(can2, (uint16_t[9]){0x00,0x20,0x00,0xF8,0x00,0x00,0x00,0x00,0x1DC});
                            break;

                        case 0x50:
                            DATA_TO_SEND(can2, (uint16_t[9]){0x00,0x10,0x00,0xF8,0x00,0x00,0x00,0x00,0x1DC});
                            break;

                        case 0x60:
                            DATA_TO_SEND(can2, (uint16_t[9]){0x00,0x18,0x00,0xF8,0x00,0x00,0x00,0x00,0x1DC});
                            break;

                        case 0x70:
                            DATA_TO_SEND(can2, (uint16_t[9]){0x00,0x28,0x00,0xF8,0x00,0x00,0x00,0x00,0x1DC});
                            break;

                    }
                }
            }

        }
        else
        {
            break;
        }
    }

    /* VIN 0x40A co ~20 ms */
    if (FLIP && (HAL_GetTick() - last_vin_time >= 20U))
    {
        last_vin_time = HAL_GetTick();
        PREPARE_VIN_FRAMES();

        uint16_t frame[9];

        switch (vin_step)
        {
            case 0:
                memcpy(frame, (uint16_t[9]){0xC0,0x00,0x00,0x82,0x56,0x34,0xD3,0x00,0x40A}, sizeof(frame));
                break;
            case 1:
                memcpy(frame, (uint16_t[9]){vin_frame0[0],vin_frame0[1],vin_frame0[2],vin_frame0[3],
                                            vin_frame0[4],vin_frame0[5],vin_frame0[6],vin_frame0[7],0x40A}, sizeof(frame));
                break;
            case 2:
                memcpy(frame, (uint16_t[9]){vin_frame1[0],vin_frame1[1],vin_frame1[2],vin_frame1[3],
                                            vin_frame1[4],vin_frame1[5],vin_frame1[6],vin_frame1[7],0x40A}, sizeof(frame));
                break;
            case 3:
                memcpy(frame, (uint16_t[9]){vin_frame2[0],vin_frame2[1],vin_frame2[2],vin_frame2[3],
                                            vin_frame2[4],vin_frame2[5],vin_frame2[6],vin_frame2[7],0x40A}, sizeof(frame));
                break;
            case 4:
                memcpy(frame, (uint16_t[9]){0xC1,0x04,0x01,0xB4,0x08,0x38,0x44,0x41,0x40A}, sizeof(frame));
                break;
            case 5:
                memcpy(frame, (uint16_t[9]){0xC1,0x10,0x41,0x0C,0x60,0x25,0x00,0x00,0x40A}, sizeof(frame));
                break;
            default:
                vin_step = 0;
                memcpy(frame, (uint16_t[9]){0xC0,0x00,0x00,0x82,0x56,0x34,0xD3,0x00,0x40A}, sizeof(frame));
                break;
        }

        DATA_TO_SEND(can2, frame);

        vin_step++;
        if (vin_step > 5U)
            vin_step = 0U;
    }

    /* Przyciski (VOL+/-, RESET, EJECT) */
    DATA_TO_KEYS(can2);
}

/* Obsługa czterech przycisków: PA15 oraz PC10/PC11/PC12. */
static void DATA_TO_KEYS(CAN_HandleTypeDef *can2)
{
    uint32_t now = HAL_GetTick();

    uint8_t k15 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  // VOL+
    uint8_t k10 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);  // VOL-
    uint8_t k11 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);  // RESET SYNC TUNE+
    uint8_t k12 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);  // EJECT

    // ---------- PRZYCISK 1 — VOL+ CAMERA ON (A15) ---------- //
    if (KEY15_LAST == 1U && k15 == 0U)
    {
        key15_time = now;
        key15_hold = 0U;

        uint16_t Tx[9] = {0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x1F,0x00,0x2A0};
        DATA_TO_SEND(can2, Tx);
    }
    else if (KEY15_LAST == 0U && k15 == 0U)
    {
        if (!key15_hold && (now - key15_time > KEY_HOLD_THRESHOLD_MS))
            key15_hold = 1U;

        if (key15_hold && (now - key15_time > KEY_REPEAT_INTERVAL_MS))
        {
            key15_time = now;
            uint16_t Tx1[9] = {0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x081};
            uint16_t Tx2[9] = {0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x082};
            uint16_t Tx3[9] = {0x00,0x00,0x10,0x00,0x00,0x00,0x01,0x00,0x109};
            DATA_TO_SEND(can2, Tx1);
            DATA_TO_SEND(can2, Tx2);
            DATA_TO_SEND(can2, Tx3);
        }
    }

    // ---------- PRZYCISK 2 — VOL- CAMERA OFF (C10) ---------- //
    if (KEY10_LAST == 1U && k10 == 0U)
    {
        key10_time = now;
        key10_hold = 0U;

        uint16_t Tx[9] = {0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x1D,0x00,0x2A0};
        DATA_TO_SEND(can2, Tx);
    }
    else if (KEY10_LAST == 0U && k10 == 0U)
    {
        if (!key10_hold && (now - key10_time > KEY_HOLD_THRESHOLD_MS))
            key10_hold = 1U;

        if (key10_hold && (now - key10_time > KEY_REPEAT_INTERVAL_MS))
        {
            key10_time = now;
            uint16_t Tx[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x109};
            DATA_TO_SEND(can2, Tx);
        }
    }

    // ---------- PRZYCISK 3 — RESET SYNC TUNE+ (C11) ---------- //
    if (KEY11_LAST == 1U && k11 == 0U)   // wciśnięcie //
    {
        key11_time = now;
        key11_hold = 0U;
    }
    else if (KEY11_LAST == 0U && k11 == 0U) // przytrzymanie //
    {
        if (!key11_hold && (now - key11_time > RESET_HOLD_MS))
        {
            key11_hold = 1U;

            uint16_t Tx[9] = {0x02,0x11,0x01,0x00,0x00,0x00,0x00,0x00,0x7D0};
            DATA_TO_SEND(can2, Tx);
        }
    }
    else if (KEY11_LAST == 0U && k11 == 1U) // puszczenie //
    {
        if (!key11_hold)
        {
            uint16_t Tx1[9] = {0x33,0xFF,0xFF,0xFF,0x10,0x00,0x1E,0x00,0x2A0};
            uint16_t Tx2[9] = {0x33,0xFF,0xFF,0xFF,0x00,0x00,0x1E,0x00,0x2A0};
            DATA_TO_SEND(can2, Tx1);
            DATA_TO_SEND(can2, Tx2);
        }
    }

    // ---------- PRZYCISK 4 — EJECT SYNC TUNE - (C12) ---------- //
    if (KEY12_LAST == 1U && k12 == 0U)   // wciśnięcie //
    {
        key12_time = now;
        key12_hold = 0U;
    }
    else if (KEY12_LAST == 0U && k12 == 0U) // przytrzymanie //
    {
        if (!key12_hold && (now - key12_time > EJECT_HOLD_THRESHOLD_MS))
        {
            key12_hold = 1U;

            uint16_t Tx[9] = {0x34,0xFF,0xFF,0xFF,0x10,0x00,0x1E,0x00,0x2A0};
            DATA_TO_SEND(can2, Tx);
        }
    }
    else if (KEY12_LAST == 0U && k12 == 1U) // puszczenie //
    {
        if (!key12_hold)
        {
            uint16_t Tx1[9] = {0x32,0xFF,0xFF,0xFF,0x10,0x00,0x1E,0x00,0x2A0};
            uint16_t Tx2[9] = {0x32,0xFF,0xFF,0xFF,0x00,0x00,0x1E,0x00,0x2A0};
            DATA_TO_SEND(can2, Tx1);
            DATA_TO_SEND(can2, Tx2);
        }
    }

    KEY15_LAST = k15;
    KEY10_LAST = k10;
    KEY11_LAST = k11;
    KEY12_LAST = k12;
}

/* ============================================================
   DATA_TO_SEND — wersja PRO (krótka próba, bez wieszania)
   DATA[0..7] = bajty, DATA[8] = StdId
============================================================ */
static void DATA_TO_SEND(CAN_HandleTypeDef *can, const uint16_t data[9])
{
    uint8_t payload[8];
    for (uint8_t i = 0; i < 8U; i++)
        payload[i] = (uint8_t)data[i];

    CAN_TxHeaderTypeDef tx;
    tx.DLC   = 8U;
    tx.ExtId = 0U;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.StdId = data[8];

    uint32_t attempts = 0U;

    while (attempts < CAN_TX_MAX_ATTEMPTS)
    {
        uint32_t start = HAL_GetTick();

        /* czekamy KRÓTKO na wolny mailbox */
        while (HAL_CAN_GetTxMailboxesFreeLevel(can) == 0U)
        {
            if (HAL_GetTick() - start > CAN_TX_TIMEOUT_MS)
            {
                break;  // po czasie dajemy sobie spokój
            }
        }

        if (HAL_CAN_GetTxMailboxesFreeLevel(can) == 0U)
        {
            attempts++;
            continue;   // spróbuj jeszcze raz (max CAN_TX_MAX_ATTEMPTS)
        }

        uint32_t mailbox;
        if (HAL_CAN_AddTxMessage(can, &tx, payload, &mailbox) == HAL_OK)
        {
            return;     // sukces
        }

        attempts++;
    }

    /* Jeśli tu dojdziemy → ramka odpuszczona.
     * Można tu kiedyś dodać licznik dropów / debug.
     */
}

/* ============================================================
   DATA_TO_NULL — abort wszystkich skrzynek TX
============================================================ */
static void DATA_TO_NULL(CAN_HandleTypeDef *can)
{
    HAL_CAN_AbortTxRequest(can, CAN_TX_MAILBOX0);
    HAL_CAN_AbortTxRequest(can, CAN_TX_MAILBOX1);
    HAL_CAN_AbortTxRequest(can, CAN_TX_MAILBOX2);
}

/* ============================================================
   PREPARE_VIN_FRAMES — 3 ramki VIN 0x40A
============================================================ */
static void PREPARE_VIN_FRAMES(void)
{
    /* FRAME 0 */
    vin_frame0[0] = 0xC1;
    vin_frame0[1] = 0x00;
    vin_frame0[2] = VIN[0];
    vin_frame0[3] = VIN[1];
    vin_frame0[4] = VIN[2];
    vin_frame0[5] = VIN[3];
    vin_frame0[6] = VIN[4];
    vin_frame0[7] = VIN[5];

    /* FRAME 1 */
    vin_frame1[0] = 0xC1;
    vin_frame1[1] = 0x01;
    vin_frame1[2] = VIN[6];
    vin_frame1[3] = VIN[7];
    vin_frame1[4] = VIN[8];
    vin_frame1[5] = VIN[9];
    vin_frame1[6] = VIN[10];
    vin_frame1[7] = VIN[11];

    /* FRAME 2 */
    vin_frame2[0] = 0xC1;
    vin_frame2[1] = 0x02;
    vin_frame2[2] = VIN[12];
    vin_frame2[3] = VIN[13];
    vin_frame2[4] = VIN[14];
    vin_frame2[5] = VIN[15];
    vin_frame2[6] = VIN[16];
    vin_frame2[7] = 0x00;  // padding
}

/* ============================================================
   CHECK_CAN_HEALTH — resetuje CAN przy BusOff / braku ruchu
============================================================ */
static void CHECK_CAN_HEALTH(void)
{
    uint32_t now = HAL_GetTick();

    if (last_can_activity == 0U)
    {
        last_can_activity = now;
        return;
    }

    uint32_t err1 = HAL_CAN_GetError(&hcan1);
    uint32_t err2 = HAL_CAN_GetError(&hcan2);

    if ( (now - last_can_activity > CAN_HEALTH_TIMEOUT_MS) ||
         (err1 & HAL_CAN_ERROR_BOF) ||
         (err2 & HAL_CAN_ERROR_BOF) )
    {
        DATA_TO_NULL(&hcan1);
        DATA_TO_NULL(&hcan2);

        HAL_CAN_Stop(&hcan1);
        HAL_CAN_Stop(&hcan2);

        HAL_CAN_Start(&hcan1);
        HAL_CAN_Start(&hcan2);

        last_can_activity = now;
    }
}
