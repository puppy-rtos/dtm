
#include "dtm_bindings.h"

#ifdef DTM_BINFDING_CLK_STM32_F4

#include <stdint.h>

/* RCC寄存器结构定义 */
typedef struct
{
  volatile uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  volatile uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  volatile uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  volatile uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  volatile uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  volatile uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  volatile uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  volatile uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  volatile uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  volatile uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  volatile uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  volatile uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} DTM_RCC_Type;


#ifndef DTM_CLK_BASE
#define DTM_CLK_BASE 0x40023800UL
#endif

#define RCC ((DTM_RCC_Type *)DTM_CLK_BASE)

/* 寄存器位定义 */
#define RCC_CR_HSION      (1 << 0)
#define RCC_CR_HSEON      (1 << 16)
#define RCC_CR_HSERDY     (1 << 17)
#define RCC_CR_PLLON      (1 << 24)
#define RCC_CR_PLLRDY     (1 << 25)

#define RCC_CFGR_SW       (3 << 0)  // 系统时钟选择
#define RCC_CFGR_SW_HSI   (0 << 0)
#define RCC_CFGR_SW_HSE   (1 << 0)
#define RCC_CFGR_SW_PLL   (2 << 0)

#define RCC_CFGR_SWS      (3 << 2)  // 系统时钟状态
#define RCC_CFGR_SWS_HSI  (0 << 2)
#define RCC_CFGR_SWS_HSE  (1 << 2)
#define RCC_CFGR_SWS_PLL  (2 << 2)

uint32_t sysclk = 0, pll_vco = 0, pll_p_ck = 0, pll_q_ck = 0, hclk = 0, pclk1 = 0, pclk2 = 0;

/**
 * @brief       Clock setting function
 * @param       plln: Main PLL multiplication factor (PLL multiplication), range: 64~432.
 * @param       pllm: Main PLL and audio PLL predivision factor (predivision before PLL), range: 2~63.
 * @param       pllp: Main PLL p division factor (division after PLL), divided as system clock, range: 2, 4, 6, 8. (only these 4 values)
 * @param       pllq: Main PLL q division factor (division after PLL), range: 2~15.
 * @note
 *
 *              Fvco: VCO frequency
 *              Fsys: System clock frequency, also the output clock frequency after main PLL p division
 *              Fq:   Main PLL q division output clock frequency
 *              Fs:   Main PLL input clock frequency, can be HSI, HSE, etc.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              When the external crystal is 8M, recommended values: plln = 336, pllm = 8, pllp = 2, pllq = 7.
 *              Get: Fvco = 8 * (336 / 8) = 336Mhz
 *                   Fsys = pll_p_ck = 336 / 2 = 168Mhz
 *                   Fq   = pll_q_ck = 336 / 7 = 48Mhz
 *
 *              The default frequencies that need to be configured for F407 are as follows:
 *              CPU frequency (HCLK) = pll_p_ck = 168Mhz
 *              AHB1/2/3(rcc_hclk1/2/3) = 168Mhz
 *              APB1(rcc_pclk1) = pll_p_ck / 4 = 42Mhz
 *              APB2(rcc_pclk2) = pll_p_ck / 2 = 84Mhz
 *
 * @retval      Error code: 0, success; 1, HSE error; 2, PLL1 error; 3, PLL2 error; 4, clock switch error;
 */
uint8_t sys_clock_set(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    uint32_t retry = 0;
    uint8_t retval = 0;
    uint8_t swsval = 0;

    RCC->CR |= 1 << 16; /* HSEON = 1, enable HSE */

    while (((RCC->CR & (1 << 17)) == 0) && (retry < 0X7FFF))
    {
        retry++;        /* Wait for HSE RDY */
    }

    if (retry == 0X7FFF)
    {
        retval = 1;     /* HSE not ready */
    }
    else
    {
        RCC->APB1ENR |= 1 << 28;                /* Enable power interface clock */

        RCC->PLLCFGR |= 0X3F & pllm;            /* Set main PLL predivision factor,  PLLM[5:0]: 2~63 */
        RCC->PLLCFGR |= plln << 6;              /* Set main PLL multiplication factor,    PLLN[8:0]: 192~432 */
        RCC->PLLCFGR |= ((pllp >> 1) - 1) << 16;/* Set main PLL p division factor, PLLP[1:0]: 0~3, representing 2~8 division */
        RCC->PLLCFGR |= pllq << 24;             /* Set main PLL q division factor, PLLQ[3:0]: 2~15 */
        RCC->PLLCFGR |= 1 << 22;                /* Set main PLL clock source from HSE */

        RCC->CFGR |= 0 << 4;                    /* HPRE[3:0]  = 0, AHB  no division, rcc_hclk1/2/3 = pll_p_ck */
        RCC->CFGR |= 5 << 10;                   /* PPRE1[2:0] = 5, APB1 4 division   rcc_pclk1 = pll_p_ck / 4 */
        RCC->CFGR |= 4 << 13;                   /* PPRE2[2:0] = 4, APB2 2 division   rcc_pclk2 = pll_p_ck / 2 */

        RCC->CR |= 1 << 24;                     /* Enable main PLL */

        retry = 0;
        while ((RCC->CR & (1 << 25)) == 0)      /* Wait for PLL ready */
        {
            retry++;

            if (retry > 0X1FFFFF)
            {
                retval = 2;                     /* Main PLL not ready */
                break;
            }
        }

        
        RCC->CFGR |= 2 << 0;                    /* Select main PLL as system clock */
        
        retry = 0;
        while (swsval != 3)                     /* Wait for successful switch of system clock source to pll_p_ck */
        {
            swsval = (RCC->CFGR & 0X0C) >> 2;   /* Get SWS[1:0] status to check if switch is successful */
            retry++;

            if (retry > 0X1FFFFF)
            {
                retval = 4; /* Unable to switch clock */
                break;
            }
        }
    }

    return retval;
}

/**
 * @brief       System clock initialization function
 * @param       plln: PLL1 multiplication factor (PLL multiplication), range: 4~512.
 * @param       pllm: PLL1 predivision factor (predivision before PLL), range: 2~63.
 * @param       pllp: PLL1 p division factor (division after PLL), divided as system clock, range: 2~128. (must be a multiple of 2)
 * @param       pllq: PLL1 q division factor (division after PLL), range: 1~128.
 * @retval      None
 */
void sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    RCC->CR = 0x00000001;           /* Set HISON, enable internal high-speed RC oscillator, clear other bits */
    RCC->CFGR = 0x00000000;         /* Clear CFGR */
    RCC->PLLCFGR = 0x00000000;      /* Clear PLLCFGR */
    RCC->CIR = 0x00000000;          /* Clear CIR */
    
    sys_clock_set(plln, pllm, pllp, pllq);  /* Set clock */
}

/**
 * @brief       dump the clock tree by printing the current clock configuration
 * @note        This function prints the current clock configuration in a tree format.
 * @retval      None
 */
void dtm_clk_dumptree(void)
{
    uint32_t hse = DTM_HSE_FREQ;
    uint32_t pllm = (RCC->PLLCFGR & 0x3F);                // PLLM[5:0]
    uint32_t plln = (RCC->PLLCFGR >> 6) & 0x1FF;          // PLLN[8:0]
    uint32_t pllp = ((((RCC->PLLCFGR >> 16) & 0x3) + 1) * 2); // PLLP[1:0]
    uint32_t pllq = (RCC->PLLCFGR >> 24) & 0xF;           // PLLQ[3:0]
    uint32_t sysclk_src = (RCC->CFGR >> 2) & 0x3;         // SWS[1:0]
    uint32_t hpre = (RCC->CFGR >> 4) & 0xF;               // HPRE[3:0]
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7;             // PPRE1[2:0]
    uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7;             // PPRE2[2:0]
    // Calculate PLL VCO frequency
    pll_vco = hse * plln / pllm;
    pll_p_ck = pll_vco / pllp;
    pll_q_ck = pll_vco / pllq;

    // Calculate SYSCLK
    if (sysclk_src == 0)
        sysclk = 16000000; // HSI
    else if (sysclk_src == 1)
        sysclk = hse;      // HSE
    else if (sysclk_src == 2)
        sysclk = pll_p_ck; // PLL
    else
        sysclk = 0;

    // AHB prescaler table
    const uint16_t ahb_presc_tbl[16] = {1,1,1,1,1,1,1,1,2,4,8,16,64,128,256,512};
    // APB prescaler table
    const uint8_t apb_presc_tbl[8] = {1,1,1,1,2,4,8,16};

    hclk = sysclk / ahb_presc_tbl[hpre];
    pclk1 = hclk / apb_presc_tbl[ppre1];
    pclk2 = hclk / apb_presc_tbl[ppre2];

    dtm_printf("clock-tree\n");
    dtm_printf("├── HSE (%lu Hz)\n", hse);
    dtm_printf("│   └── PLL\n");
    dtm_printf("│       ├── VCO (%lu Hz)\n", pll_vco);
    dtm_printf("│       ├── PLL_P (SYSCLK, %lu Hz)\n", pll_p_ck);
    dtm_printf("│       │   └── AHB (HCLK, %lu Hz)\n", hclk);
    dtm_printf("│       │       ├── APB1 (PCLK1, %lu Hz)\n", pclk1);
    dtm_printf("│       │       └── APB2 (PCLK2, %lu Hz)\n", pclk2);
    dtm_printf("│       └── PLL_Q (USB, %lu Hz)\n", pll_q_ck);
    dtm_printf("├── HSI (16 MHz)\n");
    dtm_printf("└── LSE/LSE not shown\n");
}

/**
 * @brief       Initialize the system clock to 168MHz
 * @note        This function configures the system clock to 168MHz using the PLL.
 *              It also enables the clocks for GPIOA, GPIOF, USART1, and USART2.
 * @retval      None
 */
void dtm_clk_init(void)
{
    /* Initialize the system clock to 168MHz */
    sys_stm32_clock_init(336, 8, 2, 7); /* Set PLL1 parameters for 168MHz system clock */

    /* Enable GPIOA and GPIOF clocks */
#ifdef DTM_GPIOA_CLK_EN
    RCC->AHB1ENR |= DTM_GPIOA_CLK_EN;  /* Enable GPIOA clock */
#endif
#ifdef DTM_GPIOF_CLK_EN
    RCC->AHB1ENR |= DTM_GPIOF_CLK_EN;  /* Enable GPIOF clock */
#endif

    /* Enable USART1 and USART2 clocks */
#ifdef DTM_UART1_CLK_EN
    RCC->APB2ENR |= DTM_UART1_CLK_EN;  /* Enable USART1 clock */
#endif
#ifdef DTM_UART2_CLK_EN
    RCC->APB1ENR |= DTM_UART2_CLK_EN;  /* Enable USART2 clock */
#endif

}

#endif /* DTM_BINFDING_CLK_STM32_F4 */
