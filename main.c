#include "chips/stm32f407/dtm_bindings.h"
#include "stdint.h"
#include "common/dtm_gpio.h"

/* 简单延时函数 */
static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

int main(void) {
    dtm_pmctrl_init();  /* Initialize power management controller */
    dtm_flash_init();   /* Initialize flash controller */
    dtm_clk_init();     /* Initialize system clock */

    // GPIOF11 led pin configuration
    dtm_gpio_set(GPIOF, DTM_GPIO_PIN11, DTM_GPIO_MODE_OUT, DTM_GPIO_OTYPE_PP, DTM_GPIO_SPEED_HIGH, DTM_GPIO_PUPD_NONE);

    // led blink loop
    while (1) {
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

void default_exit_handler(int status) {
    (void)status;
    while(1) {}
}