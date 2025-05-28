#include "chips/stm32f407/dtm_bindings.h"
#include "stdint.h"
#include "common/dtm_gpio.h"
#include "common/dtm_pmctrl.h"
#include "common/dtm_flash.h"
#include "common/dtm_clk.h"
#include "common/dtm_uart.h"

/* 简单延时函数 */
static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

#ifdef __GNUC__

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
int dtm_uart_puts(const char *str, int len)
{
    size_t i;

    for (i = 0; i < len; i++) {
        if (str[i] == '\n')
        {
            dtm_uart_putc('\r');
        }
        dtm_uart_putc(str[i]);
    }
    return 0;
}

void vprint(const char* fmt, va_list argp)
{
    char string[200];
    if(vsprintf(string, fmt, argp) > 0) // build string
    {
        dtm_uart_puts(string, strlen(string));
    }
}

void dtm_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

#endif

int main(void) {
    dtm_pmctrl_init();  /* Initialize power management controller */
    dtm_flash_init();   /* Initialize flash controller */
    dtm_clk_init();     /* Initialize system clock */
    dtm_uart_init(84, 115200); /* Initialize UART with 84MHz clock and 115200 baud rate */

    dtm_clk_dumptree(); /* Dump clock tree for debugging */

    // GPIOF11 led pin configuration
    dtm_gpio_set(GPIOF, DTM_GPIO_PIN11, DTM_GPIO_MODE_OUT, DTM_GPIO_OTYPE_PP, DTM_GPIO_SPEED_HIGH, DTM_GPIO_PUPD_NONE);

    // led blink loop
    while (1) {
        dtm_printf("Hi\n"); // Send "Hi" to UART
        dtm_gpio_pin_set(GPIOF, DTM_GPIO_PIN11, 0); // Set GPIOF11 low
        delay(1000000); // Delay for a while
        dtm_gpio_pin_set(GPIOF, DTM_GPIO_PIN11, 1); // Set GPIOF11 high
        delay(1000000); // Delay for a while
    }

    return 0;
}

void _exit(int status) __attribute__((weak, alias("default_exit_handler")));
void _close_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _lseek_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _write_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _read_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _sbrk_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _fstat_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _isatty_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _kill_r(int status) __attribute__((weak, alias("default_exit_handler")));
void _getpid_r(int status) __attribute__((weak, alias("default_exit_handler")));

void default_exit_handler(int status) {
    (void)status;
    while(1) {}
}
