/*--------------------------------------------------------------------
 * Minimal mixer implementation – fixed‑point & crash‑safe
 *------------------------------------------------------------------*/
#include "mixer.h"
#include <stddef.h>

/*===========================  PUBLIC BUFFERS  ===========================*/
uint16_t inputChannels[TOTAL_CHANNELS]          = {0};
uint16_t pwmMixedOutputValues[NUM_PWM_OUTPUTS]  = {0};

/*=============================  CONSTANTS  ==============================*/
enum { MIXER_MODE_MANUAL = 0, MIXER_MODE_STABILISED, MIXER_MODE_COUNT };

/* Helper macro: 100 % → +10000, ‑100 % → ‑10000 */
#define W(x)  ((int16_t)((x) * 10000))

/* Default configuration: direct 1‑to‑1 passthrough                     */
/* You can edit the tables below or generate them from a GUI later.     */
static const MixerOutput_t mixerTable[MIXER_MODE_COUNT][NUM_PWM_OUTPUTS] =
{
/* =========================== MODE 0 : MANUAL ========================== */
{
    /* PWM1‑12: simply forward channels 0‑11 (expand as needed) */
    { .inputs = { {0, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {1, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {2, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {3, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {4, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {5, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {6, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {7, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {8, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {9, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {10,W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {11,W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
},
/* ========================= MODE 1 : STABILISED ======================== */
/* Currently identical – replace weights with PID outputs later          */
{
    { .inputs = { {0, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {1, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {2, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {3, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {4, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {5, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {6, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {7, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {8, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {9, W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {10,W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
    { .inputs = { {11,W(1.0)} }, .inputCount = 1, .offset = 0, .minOutput = 1000, .maxOutput = 2000 },
}
};

/*==========================  PRIVATE STATE  ============================*/
static const MixerOutput_t *activeMixer = NULL;

/*=============================  FUNCTIONS  =============================*/
void mixerInit(void)
{
    selectMixerMode(MIXER_MODE_MANUAL);   /* safe default */
}

void selectMixerMode(uint8_t mode)
{
    if (mode >= MIXER_MODE_COUNT) mode = MIXER_MODE_MANUAL;
    activeMixer = mixerTable[mode];
}

/*----------------------------------------------------------------------
 * Fast fixed‑point update – call from your 1 kHz control loop
 *--------------------------------------------------------------------*/
void MixerUpdate(void)
{
    /* Make sure we have a valid table (paranoia once per call) */
    if (!activeMixer) {
        selectMixerMode(MIXER_MODE_MANUAL);
    }

    for (uint8_t outIdx = 0; outIdx < NUM_PWM_OUTPUTS; ++outIdx)
    {
        const MixerOutput_t *m = &activeMixer[outIdx];

        /* Neutral if no inputs defined */
        if (m->inputCount == 0) {
            pwmMixedOutputValues[outIdx] = 1500;
            continue;
        }

        /* Fixed‑point accumulation: sum(weight * channel) / 10000         */
        int32_t acc = 0;
        for (uint8_t i = 0; i < m->inputCount; ++i) {
            const MixerInput_t *in = &m->inputs[i];
            if (in->channelIndex >= TOTAL_CHANNELS) continue;   /* guard */
            acc += (int32_t)inputChannels[in->channelIndex] * in->weight;
        }
        acc = acc / 10000 + m->offset;

        /* Saturate */
        if (acc < m->minOutput)       acc = m->minOutput;
        else if (acc > m->maxOutput)  acc = m->maxOutput;

        pwmMixedOutputValues[outIdx] = (uint16_t)acc;
    }
}
