#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f4xx_hal.h"

typedef enum
{
    BUZZER_SOUND_NONE = 0,
    BUZZER_SOUND_BOOT,           /* three short high tones at MCU power-on */
    BUZZER_SOUND_POWER_ON,       /* DJI-style startup melody */
    BUZZER_SOUND_STATE_CHANGE,   /* one short high tone */
    BUZZER_SOUND_CLASSIFY,       /* two short high tones */
    BUZZER_SOUND_PICK,           /* one long high tone */
    BUZZER_SOUND_PLACE,          /* one low plus two high tones */
    BUZZER_SOUND_RETURN,         /* one long low tone */
    BUZZER_SOUND_COMPLETE,       /* three short high tones */
    BUZZER_SOUND_ERROR,          /* three short low tones */
    BUZZER_SOUND_TIMEOUT,        /* four short low tones, repeated until reset */
    BUZZER_SOUND_COUNT,
} buzzer_sound_t;

/* RoboMaster A board onboard passive buzzer: PH6 / TIM12_CH1. */
HAL_StatusTypeDef Buzzer_Init(void);
void Buzzer_Play(buzzer_sound_t sound);
void Buzzer_Task(void);

#endif
