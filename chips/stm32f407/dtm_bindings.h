#pragma once

/*
 * 1. IP Description 
 */
/* CLK IP Description */
#define DTM_BINFDING_CLK_STM32_F4
#define DTM_CLK_BASE           0x40023800UL

/* GPIO IP Description */
#define DTM_BINFDING_GPIO_STM32_F4
#define DTM_GPIO_BASE          0x40020000UL
#define GPIOA                  (DTM_GPIO_BASE + 0x0000) // GPIOA base address
#define GPIOB                  (DTM_GPIO_BASE + 0x0400) // GPIOB base address
#define GPIOC                  (DTM_GPIO_BASE + 0x0800) // GPIOC base address
#define GPIOD                  (DTM_GPIO_BASE + 0x0C00) // GPIOD base address
#define GPIOE                  (DTM_GPIO_BASE + 0x1000) // GPIOE base address
#define GPIOF                  (DTM_GPIO_BASE + 0x1400) // GPIOF base address
#define GPIOG                  (DTM_GPIO_BASE + 0x1800) // GPIOG base address
#define GPIOH                  (DTM_GPIO_BASE + 0x1C00) // GPIOH base address
#define GPIOI                  (DTM_GPIO_BASE + 0x2000) // GPIOI base address
#define GPIOJ                  (DTM_GPIO_BASE + 0x2400) // GPIOJ base address
#define GPIOK                  (DTM_GPIO_BASE + 0x2800) // GPIOK base address
// GPIO Clk Definitions
#define DTM_GPIOA_CLK_EN      (1 << 0)  // GPIOA clock enable
#define DTM_GPIOF_CLK_EN      (1 << 5)  // GPIOF clock enable

/*
 * 2. IP Description
 */
 /* pmctrl */
#define DTM_BINFDING_PMCTRL_STM32_F4
#define DTM_PMCTRL_BASE        0x40007000UL

/* flash */
#define DTM_BINFDING_FLASH_STM32_F4
#define DTM_FLASH_BASE         0x40023C00UL

/* pinctrl */
#define DTM_BINFDING_PINCTRL_STM32_F4
#define DTM_PINCTRL_BASE       0x40020000UL

/*
 * 3. IP Configuration
 */
 /* Pmctrl IP configuration */
#define DTM_PMCTRL_HIGHPERF_MODE (3 << 14) // 高性能模式，时钟可达168MHz

 /* 内存配置 */
#define DTM_RAM_SIZE          (192 * 1024)  // 192KB RAM
#define DTM_STACK_SIZE        (16 * 1024)   // 16KB栈

/* CLK IP Configuration */
#define DTM_HSE_FREQ           8000000      // 8MHz外部晶振
#define DTM_SYSCLK_FREQ        168000000    // 目标系统时钟168MHz

/* UART配置 */
#define DTM_DEBUG_UART         DTM_USART1_BASE
#define DTM_UART1_TX_PIN       9            // PA9
#define DTM_UART1_RX_PIN       10           // PA10
#define DTM_UART1_AF           7            // GPIO AF7 for USART1