#pragma once

#include <stdint.h>

/* UART寄存器结构体 */
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} DTM_STM32_UART_Type;

/* 寄存器位定义 */
#define USART_SR_TXE    (1 << 7)
#define USART_SR_RXNE   (1 << 5)
#define USART_CR1_UE    (1 << 13)
#define USART_CR1_TE    (1 << 3)
#define USART_CR1_RE    (1 << 2)

/* 函数声明 */
void dtm_uart_init(uint32_t uart_base, uint32_t baudrate);
void dtm_uart_putc(uint32_t uart_base, char c);
void dtm_uart_puts(uint32_t uart_base, const char *str);