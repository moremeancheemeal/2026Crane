#include "ControlTask.h"

#include "main.h"
#include "MotorControlTask.h"

#define ABOARD_POS_KEY_PORT     GPIOB
#define ABOARD_POS_KEY_PIN      GPIO_PIN_2

uint8_t Remote_Control_Mode = 1;

static GPIO_PinState pos_key_last = GPIO_PIN_SET;

static uint8_t KeyPressed(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState *last_state)
{
    GPIO_PinState now = HAL_GPIO_ReadPin(port, pin);
    uint8_t pressed = (*last_state == GPIO_PIN_SET && now == GPIO_PIN_RESET);

    *last_state = now;
    return pressed;
}

void ABoardKey_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = ABOARD_POS_KEY_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    pos_key_last = HAL_GPIO_ReadPin(ABOARD_POS_KEY_PORT, ABOARD_POS_KEY_PIN);
}

void AllTask(void)
{
    static uint8_t key_scan_cnt = 0;

    MotorSysTask();

    key_scan_cnt++;
    if (key_scan_cnt < 10)
    {
        return;
    }
    key_scan_cnt = 0;

    /* Z 升降高度测试:按键每按一次推进一步,交替"使能去下一高度 / 失能"。
     * 高度序列:三层 -> 二层 -> 一层 -> 放货 -> 回零,走完循环。 */
    if (KeyPressed(ABOARD_POS_KEY_PORT, ABOARD_POS_KEY_PIN, &pos_key_last))
    {
        ZHeightStep();
    }
}
