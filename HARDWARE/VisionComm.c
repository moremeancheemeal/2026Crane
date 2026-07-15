#include "VisionComm.h"

#include "MotorControlTask.h"
#include <string.h>

#define VISION_LINE_MAX        64U
#define VISION_TX_TIMEOUT_MS   20U

static UART_HandleTypeDef *vision_uart = 0;
static uint8_t vision_rx_byte = 0U;

static volatile uint8_t vision_rx_index = 0U;
static volatile uint8_t vision_line_ready = 0U;
static volatile uint8_t vision_line_overflow = 0U;

static char vision_rx_buf[VISION_LINE_MAX];
static char vision_line_buf[VISION_LINE_MAX];

static void VisionComm_ResetRxLine(void)
{
    vision_rx_index = 0U;
    vision_rx_buf[0] = '\0';
}

void VisionComm_Init(UART_HandleTypeDef *huart)
{
    vision_uart = huart;
    vision_line_ready = 0U;
    vision_line_overflow = 0U;
    VisionComm_ResetRxLine();

    if (vision_uart != 0)
    {
        (void)HAL_UART_Receive_IT(vision_uart, &vision_rx_byte, 1U);
    }
}

void VisionComm_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (vision_uart == 0 || huart != vision_uart)
    {
        return;
    }

    if (vision_rx_byte == '\n')
    {
        if (!vision_line_ready)
        {
            vision_rx_buf[vision_rx_index] = '\0';
            for (i = 0U; i <= vision_rx_index && i < VISION_LINE_MAX; i++)
            {
                vision_line_buf[i] = vision_rx_buf[i];
            }
            vision_line_ready = 1U;
        }
        VisionComm_ResetRxLine();
    }
    else if (vision_rx_byte != '\r')
    {
        if (vision_rx_index < (VISION_LINE_MAX - 1U))
        {
            vision_rx_buf[vision_rx_index++] = (char)vision_rx_byte;
        }
        else
        {
            vision_line_overflow = 1U;
            VisionComm_ResetRxLine();
        }
    }

    (void)HAL_UART_Receive_IT(vision_uart, &vision_rx_byte, 1U);
}

HAL_StatusTypeDef VisionComm_SendLine(const char *line)
{
    HAL_StatusTypeDef ret;
    size_t len;

    if (vision_uart == 0 || line == 0)
    {
        return HAL_ERROR;
    }

    len = strlen(line);
    ret = HAL_UART_Transmit(vision_uart, (uint8_t *)line, (uint16_t)len, VISION_TX_TIMEOUT_MS);
    if (ret != HAL_OK)
    {
        return ret;
    }

    return HAL_UART_Transmit(vision_uart, (uint8_t *)"\n", 1U, VISION_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef VisionComm_SendClassifyTrigger(void)
{
    return VisionComm_SendLine("s");
}

static uint8_t VisionComm_ParseUint1To5(const char **p, uint8_t *value)
{
    char c;

    if (p == 0 || *p == 0 || value == 0)
    {
        return 0U;
    }

    c = **p;
    if (c < '1' || c > '5')
    {
        return 0U;
    }

    *value = (uint8_t)(c - '0');
    (*p)++;
    return 1U;
}

static uint8_t VisionComm_ParseMatch(const char *line, uint8_t match[3])
{
    const char *p = line;

    if (strncmp(p, "MATCH:", 6U) != 0)
    {
        return 0U;
    }
    p += 6U;

    if (!VisionComm_ParseUint1To5(&p, &match[0]) || *p++ != ',')
    {
        return 0U;
    }
    if (!VisionComm_ParseUint1To5(&p, &match[1]) || *p++ != ',')
    {
        return 0U;
    }
    if (!VisionComm_ParseUint1To5(&p, &match[2]) || *p != '\0')
    {
        return 0U;
    }

    return 1U;
}

void VisionComm_Poll(void)
{
    char line[VISION_LINE_MAX];
    uint8_t match[3];
    uint8_t has_line;

    __disable_irq();
    has_line = vision_line_ready;
    if (has_line)
    {
        memcpy(line, vision_line_buf, VISION_LINE_MAX);
        vision_line_ready = 0U;
    }
    if (vision_line_overflow)
    {
        vision_line_overflow = 0U;
    }
    __enable_irq();

    if (!has_line)
    {
        return;
    }

    if (strcmp(line, "start") == 0)
    {
        CraneFlow_OnVisionStart();
    }
    else if (strcmp(line, "SDONE") == 0)
    {
        CraneFlow_OnVisionClassifyDone();
    }
    else if (strcmp(line, "ERR:NO_OCR") == 0)
    {
        CraneFlow_OnVisionError();
    }
    else if (VisionComm_ParseMatch(line, match))
    {
        CraneFlow_OnVisionMatch(match[0], match[1], match[2]);
    }
}
