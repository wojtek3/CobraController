#include "pwm.h"

// External timer handles – ensure these are defined elsewhere (e.g., in your main or HAL init files)
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;

// Structure to map a PWM output to its timer and channel
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} PWM_Channel_t;

static PWM_Channel_t pwm_channels[NUM_PWM_OUTPUTS] = {
    { &htim3, TIM_CHANNEL_3 },   // PWM1
    { &htim3, TIM_CHANNEL_4 },   // PWM2
    { &htim2, TIM_CHANNEL_1 },   // PWM3
    { &htim2, TIM_CHANNEL_2 },   // PWM4
    { &htim2, TIM_CHANNEL_3 },   // PWM5
    { &htim2, TIM_CHANNEL_4 },   // PWM6
    { &htim4, TIM_CHANNEL_1 },   // PWM7
    { &htim4, TIM_CHANNEL_2 },   // PWM8
    { &htim4, TIM_CHANNEL_3 },   // PWM9
    { &htim4, TIM_CHANNEL_4 },   // PWM10
    { &htim15, TIM_CHANNEL_1 },  // PWM11
    { &htim15, TIM_CHANNEL_2 }   // PWM12
};

void startPWM(uint8_t *pwmNumbers, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t pwmNum = pwmNumbers[i];
        if (pwmNum < 1 || pwmNum > NUM_PWM_OUTPUTS)
        {
            // Invalid PWM number; skip or handle error as needed
            continue;
        }
        PWM_Channel_t *pwm = &pwm_channels[pwmNum - 1];
        // Start PWM signal on the specified timer channel
        HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    }
}

void setPWMPulse(uint8_t pwmNumber, uint16_t pulse)
{
    if (pwmNumber < 1 || pwmNumber > NUM_PWM_OUTPUTS)
    {
        // Invalid PWM number; do nothing or handle error as needed
        return;
    }
    PWM_Channel_t *pwm = &pwm_channels[pwmNumber - 1];
    // Update the pulse width (compare register value) for the PWM channel
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, pulse);
}

void setAllPWMPulses(uint16_t *pulses)
{
    for (uint8_t i = 0; i < NUM_PWM_OUTPUTS; i++)
    {
        // Update pulse for each PWM channel from the provided array
        __HAL_TIM_SET_COMPARE(pwm_channels[i].htim, pwm_channels[i].channel, pulses[i]);
    }
}
