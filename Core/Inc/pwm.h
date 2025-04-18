#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "config.h"


/**
  * @brief  Starts PWM on the specified channels.
  * @param  pwmNumbers: Pointer to an array of PWM numbers (1 to 12) to start.
  * @param  count: Number of PWM numbers in the array.
  * @retval None
  */
void startPWM(uint8_t *pwmNumbers, uint8_t count);

/**
  * @brief  Sets the pulse (compare value) for a given PWM channel.
  * @param  pwmNumber: PWM number (1 to 12).
  * @param  pulse: The new pulse value.
  * @retval None
  */
void setPWMPulse(uint8_t pwmNumber, uint16_t pulse);

/**
  * @brief  Sets the pulse (compare value) for all PWM channels.
  * @param  pulses: Pointer to an array of pulse values for each PWM channel.
  *                The array must have NUM_PWM_OUTPUTS elements.
  * @retval None
  */
void setAllPWMPulses(uint16_t *pulses);


#ifdef __cplusplus
}
#endif

#endif // CRSF_H
