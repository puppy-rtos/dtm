#include "stm32_uart.h"
#include "../chips/stm32f407/dtm_bindings.h"

/* 获取UART寄存器指针 */
static DTM_STM32_UART_Type *get_uart_reg(uint32_t uart_base) {
    return (DTM_STM32_UART_Type *)uart_base;
}

/* 初始化UART */
void dtm_uart_init(uint32_t uart_base, uint32_t baudrate) {
    DTM_STM32_UART_Type *uart = get_uart_reg(uart_base);
    
    /* 计算波特率 */
    uint32_t clock = DTM_SYSCLK_FREQ;
    uint32_t div = (clock + (baudrate / 2)) / baudrate;
    
    /* 配置波特率 */
    uart->BRR = div;
    
    /* 启用UART */
    uart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* 发送一个字符 */
void dtm_uart_putc(uint32_t uart_base, char c) {
    DTM_STM32_UART_Type *uart = get_uart_reg(uart_base);
    while (!(uart->SR & USART_SR_TXE));
    uart->DR = (c & 0xFF);
}

/* 发送字符串 */
void dtm_uart_puts(uint32_t uart_base, const char *str) {
    while (*str) {
        dtm_uart_putc(uart_base, *str++);
    }
}