/*============================================================================
 * control.c – implementation
 *===========================================================================*/
#include "control.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "crsf.h"          /* rcChannels[]                                  */
#include "pid.h"           /* rollPID / pitchPID / yawPID                   */
#include "ahrs.h"          /* ahrs.roll / pitch / yaw                       */
#include "mixer.h"         /* inputChannels[], MixerUpdate(), …             */
#include "pwm.h"           /* setAllPWMPulses()                             */

/*==============  USER‑TUNABLE CONSTANTS  ==================================*/
#define PWM_MIN        1000
#define PWM_MAX        2000
#define PWM_MID        1500
#define DEADBAND_US       20          /* stick dead‑zone                     */

#define MAX_ANGLE_DEG    45.0f        /* STABILIZE   roll/pitch limit        */
#define MAX_RATE_DPS    400.0f        /* ACRO rates (+ yaw in STAB)          */

#define EXPO             0.35f        /* 0 = linear | 0.3…0.4 = typical      */

/*==============  RADIO CHANNEL ASSIGNMENTS  ===============================*/
/* change if your CRSF order differs                                        */
enum {
    CH_ROLL    = 0,
    CH_PITCH   = 1,
    CH_THROTTLE= 2,
    CH_YAW     = 3,
    CH_MODE    = 4      /* three‑pos switch chooses flight mode            */
};

/* helper “virtual” channels reserved for PID outputs → mixer --------------*/
#define CH_HELPER_ROLL_PID    (NUM_RC_CHANNELS+0)
#define CH_HELPER_PITCH_PID   (NUM_RC_CHANNELS+1)
#define CH_HELPER_YAW_PID     (NUM_RC_CHANNELS+2)

/*===========================================================================*/
/*---------------------------- LOCAL HELPERS --------------------------------*/
static inline float constrain_u16(float v, uint16_t lo, uint16_t hi)
{ return (v < lo) ? lo : (v > hi ? hi : v); }

static inline uint16_t clipToPulse(float v)          /* 1000…2000 µs */
{
    if (v < 1000.0f) return 1000;
    if (v > 2000.0f) return 2000;
    return (uint16_t)lroundf(v);
}

static float expoCurve(float x)
{
    /* x in −1…+1  |  returns −1…+1 with exponential response                */
    return x * (1.0f + EXPO * (x*x - 1.0f));
}

static float pwmToSignedNorm(int16_t pwm)   /* 1000…2000 → −1…+1  (dead‑band)*/
{
    int16_t v = pwm - PWM_MID;
    if (abs(v) < DEADBAND_US) v = 0;
    return (float)v / (float)(PWM_MAX - PWM_MID);
}

/*===========================================================================*/
/*---------------- RC‑to‑SET‑POINT MAPPER  ----------------------------------*/
void mapRcToSetpoints(const uint16_t *rc, Setpoints_t *sp, FlightMode_t mode)
{
    /*--- throttle (always linear 0…1) --------------------------------------*/
    sp->throttle = (float)(rc[CH_THROTTLE] - PWM_MIN) / (PWM_MAX - PWM_MIN);
    if (sp->throttle < 0.0f) sp->throttle = 0.0f;
    if (sp->throttle > 1.0f) sp->throttle = 1.0f;

    /*--- sticks to −1…+1  with expo ----------------------------------------*/
    float rollIn   = expoCurve(pwmToSignedNorm(rc[CH_ROLL]));
    float pitchIn  = expoCurve(pwmToSignedNorm(rc[CH_PITCH]));
    float yawIn    = expoCurve(pwmToSignedNorm(rc[CH_YAW]));

    switch (mode)
    {
    case FLIGHT_MODE_MANUAL:
        /* mapper not used (handled in controlLoop) */
        break;

    case FLIGHT_MODE_STABILIZE:       /* angle for R/P, rate for Y ----------*/
        sp->roll   =  rollIn  * MAX_ANGLE_DEG;
        sp->pitch  =  pitchIn * MAX_ANGLE_DEG;
        sp->yawRate=  yawIn   * MAX_RATE_DPS;
        break;

    case FLIGHT_MODE_ACRO:            /* pure body‑rate mode ----------------*/
    default:
        sp->roll   =  rollIn  * MAX_RATE_DPS;
        sp->pitch  =  pitchIn * MAX_RATE_DPS;
        sp->yawRate=  yawIn   * MAX_RATE_DPS;
        break;
    }
}

/*===========================================================================*/
/*---------------- CONTROL INITIALISATION -----------------------------------*/
void controlInit(void)
{
    pidInit();                 /* default gains                              */
    /* choose a sane mixer by default                                         */
    selectMixerMode(FLIGHT_MODE_MANUAL);
}

/*===========================================================================*/
/*---------------- THE MAIN CONTROL LOOP  -----------------------------------*/
/* Call at a constant rate (e.g. 1 kHz from StartKalmanTask or a timer).     */
/*---------------------------------------------------------------------------*/
void controlLoop(float dt)
{
    /*------------------------------------------------------------------+
     | 1. Determine requested flight mode from MODE switch              |
     +------------------------------------------------------------------*/
    FlightMode_t mode;
    uint16_t modePwm = rcChannels[CH_MODE];

    if      (modePwm < 1300) mode = FLIGHT_MODE_MANUAL;
    else if (modePwm < 1700) mode = FLIGHT_MODE_STABILIZE;
    else                     mode = FLIGHT_MODE_ACRO;

    /*------------------------------------------------------------------+
     | 2. Manual passthrough (simplest/fastest path)                    |
     +------------------------------------------------------------------*/
    if (mode == FLIGHT_MODE_MANUAL)
    {
        /* feed sticks straight to outputs; no PID, no mixer math       */
        setAllPWMPulses(rcChannels);
        selectMixerMode(FLIGHT_MODE_MANUAL);
        return;
    }

    /*------------------------------------------------------------------+
     | 3. Convert sticks → physical set‑points                          |
     +------------------------------------------------------------------*/
    Setpoints_t sp;
    mapRcToSetpoints(rcChannels, &sp, mode);

    /*------------------------------------------------------------------+
     | 4. Run the angle (STAB) or rate (ACRO) controllers               |
     +------------------------------------------------------------------*/
    extern float gyro_x_dps, gyro_y_dps, gyro_z_dps;  /* from IMU task     */
    extern ahrs_t ahrs;

    uint16_t rollOut, pitchOut, yawOut;

    if (mode == FLIGHT_MODE_STABILIZE)
    {
        /* --- outer angle loop produces desired BODY rates ------------*/
        float rollRateCmd  = pidComputeRate(&rollPID,  sp.roll,  ahrs.roll,  dt);
        float pitchRateCmd = pidComputeRate(&pitchPID, sp.pitch, ahrs.pitch, dt);

        /* --- inner rate loop : here reuse PID objects ----------------*/
        rollOut  = clipToPulse(1500.0f + pidComputeRate(&rollPID, rollRateCmd, gyro_x_dps, dt));
        pitchOut = clipToPulse(1500.0f + pidComputeRate(&pitchPID, pitchRateCmd, gyro_y_dps, dt));
        yawOut   = clipToPulse(1500.0f + pidComputeRate(&yawPID, sp.yawRate, gyro_z_dps, dt));
    }
    else  /* ACRO – single rate loop only --------------------------------*/
    {
    	rollOut  = clipToPulse(1500.0f + pidComputeRate(&rollPID, sp.roll, gyro_x_dps, dt));
    	pitchOut = clipToPulse(1500.0f + pidComputeRate(&pitchPID, sp.pitch, gyro_y_dps, dt));
    	yawOut   = clipToPulse(1500.0f + pidComputeRate(&yawPID, sp.yawRate, gyro_z_dps, dt));
    }

    /* store in helper channels for the mixer -------------------------------*/
    inputChannels[CH_HELPER_ROLL_PID]  = rollOut;
    inputChannels[CH_HELPER_PITCH_PID] = pitchOut;
    inputChannels[CH_HELPER_YAW_PID]   = yawOut;

    /* send throttle straight (or feed it through another filter first)      */
    inputChannels[CH_THROTTLE]         = (uint16_t)(PWM_MIN +
                                        sp.throttle * (PWM_MAX-PWM_MIN));

    /*------------------------------------------------------------------+
     | 5. Mix & push to PWM outputs                                     |
     +------------------------------------------------------------------*/
    selectMixerMode(mode == FLIGHT_MODE_STABILIZE ? 1 : 2);
    MixerUpdate();
    setAllPWMPulses(pwmMixedOutputValues);
}

/*===========================================================================*/
