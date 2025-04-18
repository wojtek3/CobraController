/*============================================================================
 * control.h – high‑level flight‑control front‑end
 *  ▸ Maps CRSF sticks → physical set‑points
 *  ▸ Runs the main control loop (PID + mixer + PWM)
 *===========================================================================*/
#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*------------ Flight‑mode enum ------------------------------------------------*/
typedef enum {
    FLIGHT_MODE_MANUAL     = 0,   /* full passthrough                */
    FLIGHT_MODE_STABILIZE  = 1,   /* angle mode (self‑level)         */
    FLIGHT_MODE_ACRO       = 2    /* rate mode (pure “acro”)         */
} FlightMode_t;

/*------------ Physical set‑points produced by the mapper ---------------------*/
typedef struct
{
    float roll;        /*  deg  in STABILIZE,  dps in ACRO */
    float pitch;       /*  idem                           */
    float yawRate;     /*  dps  in every attitude mode    */
    float throttle;    /*  0.0 … 1.0  (no scaling)        */
} Setpoints_t;

/*---------------------------------------------------------------------------*/
void  controlInit(void);                 /* one‑time initialisation         */
void  mapRcToSetpoints(const uint16_t *rc,
                       Setpoints_t     *sp,
                       FlightMode_t     mode);

void  controlLoop(float dt);             /* call at a fixed rate (e.g. 1 ms)*/

#ifdef __cplusplus
}
#endif
#endif /* INC_CONTROL_H_ */
