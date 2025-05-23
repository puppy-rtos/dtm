#include "dtm_bindings.h"
#include "../ips/stm32_clk.h"

/* 时钟配置 */
static const dtm_clock_config clock_cfg = {
    .source = CLOCK_SOURCE_PLL,
    .hse_freq = DTM_HSE_FREQ,
    .pll_m = 8,
    .pll_n = 336,
    .pll_p = 2,
    .ahb_div = 1,
    .apb1_div = 4,
    .apb2_div = 2,
    .target_freq = DTM_SYSCLK_FREQ
};

void SystemInit(void) {
    /* 初始化时钟 */
    dtm_clock_init(&clock_cfg);
    
    /* 初始化FPU */
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    #endif
    
    /* 启用指令和数据缓存 */
    // SCB->CCR |= SCB_CCR_IC_Msk | SCB_CCR_DC_Msk;
    // __DSB();
    // __ISB();
}