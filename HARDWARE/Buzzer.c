#include "Buzzer.h"

/* State-notification patterns are adapted from the LionHeart RoboMaster
 * C-board buzzer program (2020):
 * https://github.com/GTUnderdog/CTGU-LionHeart-Buzzer_Prgm
 * The power-on melody follows the DJI-style ESC M1 score demonstrated at:
 * https://www.bilibili.com/video/BV12M4m1r71L/
 * The hardware layer targets the RoboMaster A-board onboard passive buzzer
 * (PH6 / TIM12_CH1, rated at 2700 Hz). Playback is a non-blocking state
 * machine for this project's 2 ms bare-metal loop. */

#define BUZZER_GPIO_PORT          GPIOH
#define BUZZER_GPIO_PIN           GPIO_PIN_6
#define BUZZER_TIM_CHANNEL        TIM_CHANNEL_1

/* APB1 timer clock is 84 MHz with the current 168 MHz system clock setup. */
#define BUZZER_TIMER_CLOCK_HZ      84000000U
#define BUZZER_TIMER_TICK_HZ       1000000U
#define BUZZER_TIMER_PRESCALER     ((BUZZER_TIMER_CLOCK_HZ / BUZZER_TIMER_TICK_HZ) - 1U)
#define BUZZER_TONE_LOW_HZ         1960U
#define BUZZER_TONE_HIGH_HZ        2400U
#define BUZZER_START_C6_HZ         1047U
#define BUZZER_START_D6_HZ         1175U
#define BUZZER_START_G6_HZ         1568U
#define BUZZER_START_QUARTER_MS    203U
#define BUZZER_START_SIXTEENTH_MS  51U
#define BUZZER_REST_HZ             0U
#define BUZZER_QUEUE_CAPACITY     8U

#define BUZZER_ARRAY_SIZE(a)      ((uint8_t)(sizeof(a) / sizeof((a)[0])))

typedef struct
{
    uint16_t frequency_hz;
    uint16_t duration_ms;
} buzzer_note_t;

typedef struct
{
    const buzzer_note_t *notes;
    uint8_t count;
} buzzer_pattern_t;

static const buzzer_note_t buzzer_power_on[] =
{
    /* DJI ESC M1: C6 D6 G6 / D6 G6 / D6 G6, Gen.Length 13. */
    { BUZZER_START_C6_HZ, BUZZER_START_QUARTER_MS },
    { BUZZER_START_D6_HZ, BUZZER_START_QUARTER_MS },
    { BUZZER_START_G6_HZ, BUZZER_START_QUARTER_MS },
    { BUZZER_REST_HZ, BUZZER_START_SIXTEENTH_MS },
    { BUZZER_START_D6_HZ, BUZZER_START_QUARTER_MS },
    { BUZZER_START_G6_HZ, BUZZER_START_QUARTER_MS },
    { BUZZER_REST_HZ, BUZZER_START_SIXTEENTH_MS },
    { BUZZER_START_D6_HZ, BUZZER_START_QUARTER_MS },
    { BUZZER_START_G6_HZ, BUZZER_START_QUARTER_MS },
};
static const buzzer_note_t buzzer_state_change[] =
{
    { BUZZER_TONE_HIGH_HZ, 70U },
};
static const buzzer_note_t buzzer_classify[] =
{
    { BUZZER_TONE_HIGH_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_HIGH_HZ, 70U },
};
static const buzzer_note_t buzzer_pick[] =
{
    { BUZZER_TONE_HIGH_HZ, 500U },
};
static const buzzer_note_t buzzer_place[] =
{
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_HIGH_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_HIGH_HZ, 70U },
};
static const buzzer_note_t buzzer_return[] =
{
    { BUZZER_TONE_LOW_HZ, 500U },
};
static const buzzer_note_t buzzer_complete[] =
{
    { BUZZER_TONE_HIGH_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_HIGH_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_HIGH_HZ, 70U },
};
static const buzzer_note_t buzzer_error[] =
{
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 70U },
    { BUZZER_TONE_LOW_HZ, 70U },
};
static const buzzer_note_t buzzer_timeout[] =
{
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 930U },
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 930U },
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 930U },
    { BUZZER_TONE_LOW_HZ, 70U }, { BUZZER_REST_HZ, 1000U },
};

static const buzzer_pattern_t buzzer_patterns[BUZZER_SOUND_COUNT] =
{
    { 0, 0U },
    { buzzer_complete,     BUZZER_ARRAY_SIZE(buzzer_complete) },
    { buzzer_power_on,     BUZZER_ARRAY_SIZE(buzzer_power_on) },
    { buzzer_state_change, BUZZER_ARRAY_SIZE(buzzer_state_change) },
    { buzzer_classify,     BUZZER_ARRAY_SIZE(buzzer_classify) },
    { buzzer_pick,         BUZZER_ARRAY_SIZE(buzzer_pick) },
    { buzzer_place,        BUZZER_ARRAY_SIZE(buzzer_place) },
    { buzzer_return,       BUZZER_ARRAY_SIZE(buzzer_return) },
    { buzzer_complete,     BUZZER_ARRAY_SIZE(buzzer_complete) },
    { buzzer_error,        BUZZER_ARRAY_SIZE(buzzer_error) },
    { buzzer_timeout,      BUZZER_ARRAY_SIZE(buzzer_timeout) },
};

static TIM_HandleTypeDef buzzer_tim;
static buzzer_sound_t buzzer_queue[BUZZER_QUEUE_CAPACITY];
static buzzer_sound_t buzzer_active_sound = BUZZER_SOUND_NONE;
static const buzzer_pattern_t *buzzer_active_pattern = 0;
static uint32_t buzzer_note_deadline = 0U;
static uint8_t buzzer_queue_head = 0U;
static uint8_t buzzer_queue_tail = 0U;
static uint8_t buzzer_queue_count = 0U;
static uint8_t buzzer_note_index = 0U;
static uint8_t buzzer_initialized = 0U;
static uint8_t buzzer_busy = 0U;
static uint8_t buzzer_timeout_latched = 0U;

static void Buzzer_OutputOff(void)
{
    __HAL_TIM_SET_COMPARE(&buzzer_tim, BUZZER_TIM_CHANNEL, 0U);
}

static void Buzzer_OutputTone(uint16_t frequency_hz)
{
    uint32_t period_ticks = BUZZER_TIMER_TICK_HZ / frequency_hz;

    if (period_ticks < 2U)
    {
        period_ticks = 2U;
    }

    __HAL_TIM_SET_AUTORELOAD(&buzzer_tim, period_ticks - 1U);
    __HAL_TIM_SET_COMPARE(&buzzer_tim, BUZZER_TIM_CHANNEL, period_ticks / 2U);
    __HAL_TIM_SET_COUNTER(&buzzer_tim, 0U);
    buzzer_tim.Instance->EGR = TIM_EGR_UG;
}

static void Buzzer_ApplyNote(const buzzer_note_t *note)
{
    if (note->frequency_hz == BUZZER_REST_HZ)
    {
        Buzzer_OutputOff();
    }
    else
    {
        Buzzer_OutputTone(note->frequency_hz);
    }
}

static void Buzzer_StartNext(uint32_t now)
{
    buzzer_sound_t sound;
    const buzzer_note_t *note;

    if (buzzer_queue_count == 0U)
    {
        return;
    }

    sound = buzzer_queue[buzzer_queue_head];
    buzzer_queue_head = (uint8_t)((buzzer_queue_head + 1U) % BUZZER_QUEUE_CAPACITY);
    buzzer_queue_count--;

    buzzer_active_sound = sound;
    buzzer_active_pattern = &buzzer_patterns[sound];
    buzzer_note_index = 0U;
    buzzer_busy = 1U;
    note = &buzzer_active_pattern->notes[0];
    Buzzer_ApplyNote(note);
    buzzer_note_deadline = now + note->duration_ms;
}

HAL_StatusTypeDef Buzzer_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef channel = {0};
    HAL_StatusTypeDef status;

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    gpio.Pin = BUZZER_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &gpio);

    buzzer_tim.Instance = TIM12;
    buzzer_tim.Init.Prescaler = BUZZER_TIMER_PRESCALER;
    buzzer_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    buzzer_tim.Init.Period = (BUZZER_TIMER_TICK_HZ / BUZZER_TONE_HIGH_HZ) - 1U;
    buzzer_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    buzzer_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    status = HAL_TIM_PWM_Init(&buzzer_tim);
    if (status != HAL_OK)
    {
        return status;
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    status = HAL_TIM_PWM_ConfigChannel(&buzzer_tim, &channel, BUZZER_TIM_CHANNEL);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_TIM_PWM_Start(&buzzer_tim, BUZZER_TIM_CHANNEL);
    if (status != HAL_OK)
    {
        return status;
    }

    Buzzer_OutputOff();
    buzzer_queue_head = 0U;
    buzzer_queue_tail = 0U;
    buzzer_queue_count = 0U;
    buzzer_active_pattern = 0;
    buzzer_active_sound = BUZZER_SOUND_NONE;
    buzzer_note_index = 0U;
    buzzer_busy = 0U;
    buzzer_timeout_latched = 0U;
    buzzer_initialized = 1U;
    return HAL_OK;
}

void Buzzer_Play(buzzer_sound_t sound)
{
    if (!buzzer_initialized || sound <= BUZZER_SOUND_NONE || sound >= BUZZER_SOUND_COUNT)
    {
        return;
    }

    /* A timeout is latched until MCU reset and cannot be masked by later sounds. */
    if (buzzer_timeout_latched)
    {
        return;
    }

    if (sound == BUZZER_SOUND_TIMEOUT)
    {
        buzzer_timeout_latched = 1U;
    }

    /* Fault sound has priority over queued normal state notifications. */
    if (sound == BUZZER_SOUND_ERROR || sound == BUZZER_SOUND_TIMEOUT)
    {
        buzzer_queue_head = 0U;
        buzzer_queue_tail = 0U;
        buzzer_queue_count = 0U;
        buzzer_active_pattern = 0;
        buzzer_active_sound = BUZZER_SOUND_NONE;
        buzzer_busy = 0U;
        Buzzer_OutputOff();
    }

    if (buzzer_queue_count >= BUZZER_QUEUE_CAPACITY)
    {
        return;
    }

    buzzer_queue[buzzer_queue_tail] = sound;
    buzzer_queue_tail = (uint8_t)((buzzer_queue_tail + 1U) % BUZZER_QUEUE_CAPACITY);
    buzzer_queue_count++;
}

void Buzzer_Task(void)
{
    uint32_t now;
    const buzzer_note_t *note;

    if (!buzzer_initialized)
    {
        return;
    }

    now = HAL_GetTick();
    if (!buzzer_busy)
    {
        Buzzer_StartNext(now);
        return;
    }

    if ((int32_t)(now - buzzer_note_deadline) < 0)
    {
        return;
    }

    buzzer_note_index++;
    if (buzzer_note_index >= buzzer_active_pattern->count)
    {
        if (buzzer_active_sound == BUZZER_SOUND_TIMEOUT)
        {
            buzzer_note_index = 0U;
            note = &buzzer_active_pattern->notes[0];
            Buzzer_ApplyNote(note);
            buzzer_note_deadline = now + note->duration_ms;
            return;
        }

        Buzzer_OutputOff();
        buzzer_active_pattern = 0;
        buzzer_active_sound = BUZZER_SOUND_NONE;
        buzzer_busy = 0U;
        Buzzer_StartNext(now);
        return;
    }

    note = &buzzer_active_pattern->notes[buzzer_note_index];
    Buzzer_ApplyNote(note);
    buzzer_note_deadline = now + note->duration_ms;
}
