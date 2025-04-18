#ifndef MIXER_H
#define MIXER_H
/*----------------------------------------------------------------------
 * Very‑small generic mixer for STM32 projects
 *  ‑ maps RC / helper channels → PWM outputs
 *  ‑ no dynamic allocation, no floats in the update loop
 *--------------------------------------------------------------------*/

#include <stdint.h>
#include "config.h"

/*--------------------------------------------------------------------
 * User‑tunable project‑wide constants
 *  (NUM_RC_CHANNELS / NUM_HELPER_CHANNELS are expected from elsewhere)
 *------------------------------------------------------------------*/
#define MAX_MIXER_INPUTS         4U

/* Combined RC + helper channel count */


/*--------------------------------------------------------------------
 * Public data
 *------------------------------------------------------------------*/
extern uint16_t inputChannels[TOTAL_CHANNELS];           /* updated by RC / attitude code */
extern uint16_t pwmMixedOutputValues[NUM_PWM_OUTPUTS];   /* written by MixerUpdate()      */

/*--------------------------------------------------------------------
 * Mixer structures
 *------------------------------------------------------------------*/
typedef struct {
    uint8_t channelIndex;   /* 0 … TOTAL_CHANNELS‑1                */
    int16_t weight;         /* fixed‑point: +10000 = +100 %        */
} MixerInput_t;

typedef struct {
    MixerInput_t inputs[MAX_MIXER_INPUTS];
    uint8_t      inputCount;    /* how many entries in inputs[] are valid   */
    int16_t      offset;        /* µs added after mixing                    */
    int16_t      minOutput;     /* clamp                                   */
    int16_t      maxOutput;
} MixerOutput_t;

/*--------------------------------------------------------------------
 * API
 *------------------------------------------------------------------*/
void  mixerInit(void);              /* must be called once at start‑up     */
void  selectMixerMode(uint8_t mode);/* 0 = manual, 1 = stabilised (future) */
void  MixerUpdate(void);            /* call in control loop                */

#endif /* MIXER_H */
