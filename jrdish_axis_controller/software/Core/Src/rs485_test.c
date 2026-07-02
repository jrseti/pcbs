/* -----------------------------------------------------------------------
 * rs485_test.c
 * Simple RS485 ping test - receives "PING <n>", replies "ACK <n>"
 *
 * Handles both RS485 ports simultaneously. Plug the USB-RS485 adapter
 * into either port and it will respond correctly.
 * ----------------------------------------------------------------------- */

#include "rs485_test.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ----------------------------------------------------------------------- */
/* Config                                                                   */
/* ----------------------------------------------------------------------- */
#define RX_BUF_SIZE     64
#define TX_BUF_SIZE     64

/* ----------------------------------------------------------------------- */
/* Port descriptor                                                          */
/* ----------------------------------------------------------------------- */
typedef struct
{
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *dePort;
    uint16_t            dePin;
    uint8_t             rxByte;
    char                rxBuf[RX_BUF_SIZE];
    uint8_t             rxIdx;
    uint8_t             lineReady;
} RS485_Port_t;

/* ----------------------------------------------------------------------- */
/* Private variables                                                        */
/* ----------------------------------------------------------------------- */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

static RS485_Port_t ports[2] =
{
    {
        .huart   = &huart1,
        .dePort  = RS485_1_DE_GPIO_Port,
        .dePin   = RS485_1_DE_Pin,
    },
    {
        .huart   = &huart3,
        .dePort  = RS485_2_DE_GPIO_Port,
        .dePin   = RS485_2_DE_Pin,
    },
};

/* ----------------------------------------------------------------------- */
/* Private helpers                                                          */
/* ----------------------------------------------------------------------- */

static void Port_StartReceive(RS485_Port_t *p)
{
    HAL_UART_Receive_IT(p->huart, &p->rxByte, 1);
}

static void Port_Transmit(RS485_Port_t *p, const char *str)
{
    uint16_t len = (uint16_t)strlen(str);

    HAL_GPIO_WritePin(p->dePort, p->dePin, GPIO_PIN_SET);
    HAL_UART_Transmit(p->huart, (uint8_t *)str, len, 100);
    while (__HAL_UART_GET_FLAG(p->huart, UART_FLAG_TC) == RESET);
    HAL_GPIO_WritePin(p->dePort, p->dePin, GPIO_PIN_RESET);
}

static void Port_ProcessLine(RS485_Port_t *p)
{
    char cmd[16] = {0};
    int  counter = -1;

    if (sscanf(p->rxBuf, "%15s %d", cmd, &counter) == 2 &&
        strcmp(cmd, "PING") == 0 &&
        counter >= 0)
    {
        char txBuf[TX_BUF_SIZE];
        snprintf(txBuf, sizeof(txBuf), "ACK %d\n", counter);
        Port_Transmit(p, txBuf);
    }
}

/* ----------------------------------------------------------------------- */
/* UART RX complete callback                                                */
/* If you have an existing HAL_UART_RxCpltCallback, merge this body in.    */
/* ----------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < 2; i++)
    {
        RS485_Port_t *p = &ports[i];

        if (huart->Instance != p->huart->Instance)
            continue;

        char c = (char)p->rxByte;

        if (c == '\n')
        {
            p->rxBuf[p->rxIdx] = '\0';
            p->lineReady = 1;
            p->rxIdx = 0;
        }
        else if (c != '\r')
        {
            if (p->rxIdx < RX_BUF_SIZE - 1)
                p->rxBuf[p->rxIdx++] = c;
            else
                p->rxIdx = 0;   /* overflow - reset */
        }

        Port_StartReceive(p);
        break;
    }
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

void RS485_Test_Init(void)
{
    for (int i = 0; i < 2; i++)
    {
        RS485_Port_t *p = &ports[i];
        HAL_GPIO_WritePin(p->dePort, p->dePin, GPIO_PIN_RESET);
        Port_StartReceive(p);
    }
}

void RS485_Test_Run(void)
{
    for (int i = 0; i < 2; i++)
    {
        RS485_Port_t *p = &ports[i];

        if (p->lineReady)
        {
            p->lineReady = 0;
            Port_ProcessLine(p);
        }
    }
}
