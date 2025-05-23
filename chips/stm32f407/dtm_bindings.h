#pragma once

#include "dtm_irq.h"  // 包含芯片特定中断定义
// #include "stm32f407xx.h"  // 包含芯片特定定义

/* 内存配置 */
#define DTM_FLASH_BASE         0x08000000UL
#define DTM_RAM_BASE           0x20000000UL
#define DTM_RAM_SIZE          (192 * 1024)  // 192KB RAM
#define DTM_STACK_SIZE        (16 * 1024)   // 16KB栈

/* 外设基地址 */
#define DTM_RCC_BASE           0x40023800UL
#define DTM_USART1_BASE        0x40011000UL
#define DTM_GPIOA_BASE         0x40020000UL

/* 时钟配置 */
#define DTM_HSE_FREQ           8000000      // 8MHz外部晶振
#define DTM_SYSCLK_FREQ        168000000    // 目标系统时钟168MHz

/* UART配置 */
#define DTM_DEBUG_UART         DTM_USART1_BASE
#define DTM_UART1_TX_PIN       9            // PA9
#define DTM_UART1_RX_PIN       10           // PA10
#define DTM_UART1_AF           7            // GPIO AF7 for USART1