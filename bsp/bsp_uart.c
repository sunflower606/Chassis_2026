#include "bsp_uart.h"

#include <stddef.h>

#include "board_config.h"

/* 接收环形缓冲：永远留一个空位区分「空」与「满」 */
static uint8_t s_rx_buf[BOARD_LINK_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* 发送环形缓冲：SendString 只把字节搬进内存，真正的移位交给 TXE 中断。
 *
 * 改之前是「塞一个字节 -> 死等 TXE -> 再塞下一个」：115200 下 1 字节 86.8 us，
 * 110 字节的调试行要 9.5 ms，这段时间主循环完全停摆在 while 里转圈。
 * 改成中断后主循环只花几十微秒，剩下 110 次小中断每次 1~2 us，总 CPU 从
 * 9500 us 降到约 150 us，而且都切在两个字节之间，不再打断控制周期。
 *
 * head 只有主循环写，tail 只有中断写，天然不需要关中断保护。 */
static uint8_t s_tx_buf[BOARD_LINK_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0U;
static volatile uint16_t s_tx_tail = 0U;

void BSP_UART_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

    RCC_APB2PeriphClockCmd(BOARD_LINK_UART_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BOARD_LINK_GPIO_CLK, ENABLE);

    /* TX：复用推挽 */
    gpio_init.GPIO_Pin = BOARD_LINK_TX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BOARD_LINK_GPIO, &gpio_init);

    /* RX：浮空输入 */
    gpio_init.GPIO_Pin = BOARD_LINK_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BOARD_LINK_GPIO, &gpio_init);

    usart_init.USART_BaudRate = BOARD_LINK_BAUDRATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(BOARD_LINK_UART, &usart_init);

    USART_ITConfig(BOARD_LINK_UART, USART_IT_RXNE, ENABLE);
    USART_Cmd(BOARD_LINK_UART, ENABLE);

    nvic_init.NVIC_IRQChannel = BOARD_LINK_UART_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic_init.NVIC_IRQChannelSubPriority = 1U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

void BSP_UART_SendByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_tx_head + 1U) % BOARD_LINK_TX_BUF_SIZE);

    /* 缓冲满：丢弃新字节，保护还没发出去的内容 */
    if (next == s_tx_tail) {
        return;
    }

    s_tx_buf[s_tx_head] = byte;
    s_tx_head = next;

    /* DR 空了就进中断；重复使能无副作用 */
    USART_ITConfig(BOARD_LINK_UART, USART_IT_TXE, ENABLE);
}

void BSP_UART_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        BSP_UART_SendByte((uint8_t)(*text));
        text++;
    }
}

uint16_t BSP_UART_GetTxPending(void)
{
    uint16_t head = s_tx_head;
    uint16_t tail = s_tx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BOARD_LINK_TX_BUF_SIZE - tail + head);
}

uint16_t BSP_UART_GetRxCount(void)
{
    uint16_t head = s_rx_head;
    uint16_t tail = s_rx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BOARD_LINK_RX_BUF_SIZE - tail + head);
}

/* 距队尾 offset 个字节的物理下标 */
static uint16_t BSP_UART_IndexAt(uint16_t offset)
{
    uint16_t index = (uint16_t)(s_rx_tail + offset);

    if (index >= BOARD_LINK_RX_BUF_SIZE) {
        index = (uint16_t)(index - BOARD_LINK_RX_BUF_SIZE);
    }

    return index;
}

uint8_t BSP_UART_PeekAt(uint16_t offset, uint8_t *byte)
{
    if (offset >= BSP_UART_GetRxCount()) {
        return 0U;
    }

    if (byte != NULL) {
        *byte = s_rx_buf[BSP_UART_IndexAt(offset)];
    }

    return 1U;
}

void BSP_UART_Skip(uint16_t count)
{
    uint16_t available = BSP_UART_GetRxCount();

    if (count > available) {
        count = available;
    }

    s_rx_tail = BSP_UART_IndexAt(count);
}

uint8_t BSP_UART_ReadByte(void)
{
    uint8_t byte;

    if (s_rx_head == s_rx_tail) {
        return 0xFFU;
    }

    byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BOARD_LINK_RX_BUF_SIZE);

    return byte;
}

void BOARD_LINK_UART_IRQHandler(void)
{
    if (USART_GetITStatus(BOARD_LINK_UART, USART_IT_RXNE) != RESET) {
        uint8_t byte = (uint8_t)USART_ReceiveData(BOARD_LINK_UART);
        uint16_t next = (uint16_t)((s_rx_head + 1U) % BOARD_LINK_RX_BUF_SIZE);

        /* 缓冲满时丢弃新字节，保护未解析数据 */
        if (next != s_rx_tail) {
            s_rx_buf[s_rx_head] = byte;
            s_rx_head = next;
        }

        USART_ClearITPendingBit(BOARD_LINK_UART, USART_IT_RXNE);
    }

    /* 发送：DR 空了就喂下一个字节，队列空就关掉自己（TXE 由写 DR 自动清除） */
    if (USART_GetITStatus(BOARD_LINK_UART, USART_IT_TXE) != RESET) {
        if (s_tx_tail != s_tx_head) {
            USART_SendData(BOARD_LINK_UART, (uint16_t)s_tx_buf[s_tx_tail]);
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) % BOARD_LINK_TX_BUF_SIZE);
        } else {
            USART_ITConfig(BOARD_LINK_UART, USART_IT_TXE, DISABLE);
        }
    }
}
