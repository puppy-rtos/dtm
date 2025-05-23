#pragma once

#include <stdint.h>

/* 时钟源类型定义 */
typedef enum {
    CLOCK_SOURCE_HSI,   // 内部高速时钟
    CLOCK_SOURCE_HSE,   // 外部高速时钟
    CLOCK_SOURCE_PLL,   // PLL时钟
    CLOCK_SOURCE_PLL_HSE, // PLL使用HSE作为输入
    CLOCK_SOURCE_PLL_HSI  // PLL使用HSI作为输入
} clock_source_t;

/* 时钟配置结构体 */
typedef struct {
    clock_source_t source;    // 时钟源
    uint32_t hse_freq;        // HSE频率(Hz)
    uint32_t pll_m;           // PLL M分频因子
    uint32_t pll_n;           // PLL N倍频因子
    uint32_t pll_p;           // PLL P分频因子
    uint32_t pll_q;           // PLL Q分频因子(某些系列需要)
    uint32_t ahb_div;         // AHB分频因子
    uint32_t apb1_div;        // APB1分频因子
    uint32_t apb2_div;        // APB2分频因子
    uint32_t target_freq;     // 目标系统频率(Hz)
} dtm_clock_config;


/* 函数声明 */
void dtm_clock_init(const dtm_clock_config *config);
