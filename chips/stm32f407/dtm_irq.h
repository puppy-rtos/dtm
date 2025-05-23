// dtm/chips/stm32f407/dtm_irq.h
#pragma once

#include "../../arch/arm/cortex-m4/dtm_irq.h"

/* STM32F407 特定中断编号 */
typedef enum {
    DTM_WWDG_IRQn = 0,
    DTM_PVD_IRQn,
    DTM_TAMP_STAMP_IRQn,
    // ... 完整的中断编号列表
    DTM_USART1_IRQn = 37,
    DTM_USART2_IRQn,
    // ... 继续添加
    DTM_DMA2_Stream5_IRQn = 87
} DTM_STM32_IRQn_Type;