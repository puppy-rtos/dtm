#include "chips/stm32f407/dtm_bindings.h"
#include "ips/stm32_uart.h"

/* 简单延时函数 */
static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

int main(void) {
    /* 初始化调试UART */
    dtm_uart_init(DTM_DEBUG_UART, 115200);
    
    /* 打印欢迎信息 */
    dtm_uart_puts(DTM_DEBUG_UART, "\r\nHello DTM!\r\n");
    dtm_uart_puts(DTM_DEBUG_UART, "STM32F407 running at ");
    dtm_uart_puts(DTM_DEBUG_UART, "168MHz\r\n");
    
    while (1) {
        dtm_uart_puts(DTM_DEBUG_UART, ".");
        delay(1000000);
    }
    
    return 0;
}

void _exit(int status) __attribute__((weak, alias("default_exit_handler")));

void default_exit_handler(int status) {
    (void)status;
    while(1) {}
}