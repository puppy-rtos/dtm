
#include "dtm_bindings.h"

#ifdef DTM_BINFDING_CLK_ARM_M7_ST_GD

#include <stdint.h>

/* RCC寄存器结构定义 */
typedef struct
{
    volatile uint32_t CR;             /*!< RCC clock control register,                                              Address offset: 0x00  */
    volatile uint32_t HSICFGR;        /*!< HSI Clock Calibration Register,                                          Address offset: 0x04  */
    volatile uint32_t CRRCR;          /*!< Clock Recovery RC  Register,                                             Address offset: 0x08  */
    volatile uint32_t CSICFGR;        /*!< CSI Clock Calibration Register,                                          Address offset: 0x0C  */
    volatile uint32_t CFGR;           /*!< RCC clock configuration register,                                        Address offset: 0x10  */
    uint32_t     RESERVED1;       /*!< Reserved,                                                                Address offset: 0x14  */
    volatile uint32_t D1CFGR;         /*!< RCC Domain 1 configuration register,                                     Address offset: 0x18  */
    volatile uint32_t D2CFGR;         /*!< RCC Domain 2 configuration register,                                     Address offset: 0x1C  */
    volatile uint32_t D3CFGR;         /*!< RCC Domain 3 configuration register,                                     Address offset: 0x20  */
    uint32_t     RESERVED2;       /*!< Reserved,                                                                Address offset: 0x24  */
    volatile uint32_t PLLCKSELR;      /*!< RCC PLLs Clock Source Selection Register,                                Address offset: 0x28  */
    volatile uint32_t PLLCFGR;        /*!< RCC PLLs  Configuration Register,                                        Address offset: 0x2C  */
    volatile uint32_t PLL1DIVR;       /*!< RCC PLL1 Dividers Configuration Register,                                Address offset: 0x30  */
    volatile uint32_t PLL1FRACR;      /*!< RCC PLL1 Fractional Divider Configuration Register,                      Address offset: 0x34  */
    volatile uint32_t PLL2DIVR;       /*!< RCC PLL2 Dividers Configuration Register,                                Address offset: 0x38  */
    volatile uint32_t PLL2FRACR;      /*!< RCC PLL2 Fractional Divider Configuration Register,                      Address offset: 0x3C  */
    volatile uint32_t PLL3DIVR;       /*!< RCC PLL3 Dividers Configuration Register,                                Address offset: 0x40  */
    volatile uint32_t PLL3FRACR;      /*!< RCC PLL3 Fractional Divider Configuration Register,                      Address offset: 0x44  */
    uint32_t      RESERVED3;      /*!< Reserved,                                                                Address offset: 0x48  */
    volatile uint32_t  D1CCIPR;       /*!< RCC Domain 1 Kernel Clock Configuration Register                         Address offset: 0x4C  */
    volatile uint32_t  D2CCIP1R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x50  */
    volatile uint32_t  D2CCIP2R;      /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x54  */
    volatile uint32_t  D3CCIPR;       /*!< RCC Domain 3 Kernel Clock Configuration Register                         Address offset: 0x58  */
    uint32_t      RESERVED4;      /*!< Reserved,                                                                Address offset: 0x5C  */
    volatile uint32_t  CIER;          /*!< RCC Clock Source Interrupt Enable Register                               Address offset: 0x60  */
    volatile uint32_t  CIFR;          /*!< RCC Clock Source Interrupt Flag Register                                 Address offset: 0x64  */
    volatile uint32_t  CICR;          /*!< RCC Clock Source Interrupt Clear Register                                Address offset: 0x68  */
    uint32_t     RESERVED5;       /*!< Reserved,                                                                Address offset: 0x6C  */
    volatile uint32_t  BDCR;          /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x70  */
    volatile uint32_t  CSR;           /*!< RCC clock control & status register,                                     Address offset: 0x74  */
    uint32_t     RESERVED6;       /*!< Reserved,                                                                Address offset: 0x78  */
    volatile uint32_t AHB3RSTR;       /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x7C  */
    volatile uint32_t AHB1RSTR;       /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x80  */
    volatile uint32_t AHB2RSTR;       /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x84  */
    volatile uint32_t AHB4RSTR;       /*!< RCC AHB4 peripheral reset register,                                      Address offset: 0x88  */
    volatile uint32_t APB3RSTR;       /*!< RCC APB3 peripheral reset register,                                      Address offset: 0x8C  */
    volatile uint32_t APB1LRSTR;      /*!< RCC APB1 peripheral reset Low Word register,                             Address offset: 0x90  */
    volatile uint32_t APB1HRSTR;      /*!< RCC APB1 peripheral reset High Word register,                            Address offset: 0x94  */
    volatile uint32_t APB2RSTR;       /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x98  */
    volatile uint32_t APB4RSTR;       /*!< RCC APB4 peripheral reset register,                                      Address offset: 0x9C  */
    volatile uint32_t GCR;            /*!< RCC RCC Global Control  Register,                                        Address offset: 0xA0  */
    uint32_t     RESERVED8;       /*!< Reserved,                                                                Address offset: 0xA4  */
    volatile uint32_t D3AMR;          /*!< RCC Domain 3 Autonomous Mode Register,                                   Address offset: 0xA8  */
    uint32_t     RESERVED11[9];    /*!< Reserved, 0xAC-0xCC                                                      Address offset: 0xAC  */
    volatile uint32_t RSR;            /*!< RCC Reset status register,                                               Address offset: 0xD0  */
    volatile uint32_t AHB3ENR;        /*!< RCC AHB3 peripheral clock  register,                                     Address offset: 0xD4  */
    volatile uint32_t AHB1ENR;        /*!< RCC AHB1 peripheral clock  register,                                     Address offset: 0xD8  */
    volatile uint32_t AHB2ENR;        /*!< RCC AHB2 peripheral clock  register,                                     Address offset: 0xDC  */
    volatile uint32_t AHB4ENR;        /*!< RCC AHB4 peripheral clock  register,                                     Address offset: 0xE0  */
    volatile uint32_t APB3ENR;        /*!< RCC APB3 peripheral clock  register,                                     Address offset: 0xE4  */
    volatile uint32_t APB1LENR;       /*!< RCC APB1 peripheral clock  Low Word register,                            Address offset: 0xE8  */
    volatile uint32_t APB1HENR;       /*!< RCC APB1 peripheral clock  High Word register,                           Address offset: 0xEC  */
    volatile uint32_t APB2ENR;        /*!< RCC APB2 peripheral clock  register,                                     Address offset: 0xF0  */
    volatile uint32_t APB4ENR;        /*!< RCC APB4 peripheral clock  register,                                     Address offset: 0xF4  */
    uint32_t      RESERVED12;      /*!< Reserved,                                                                Address offset: 0xF8  */
    volatile uint32_t AHB3LPENR;      /*!< RCC AHB3 peripheral sleep clock  register,                               Address offset: 0xFC  */
    volatile uint32_t AHB1LPENR;      /*!< RCC AHB1 peripheral sleep clock  register,                               Address offset: 0x100 */
    volatile uint32_t AHB2LPENR;      /*!< RCC AHB2 peripheral sleep clock  register,                               Address offset: 0x104 */
    volatile uint32_t AHB4LPENR;      /*!< RCC AHB4 peripheral sleep clock  register,                               Address offset: 0x108 */
    volatile uint32_t APB3LPENR;      /*!< RCC APB3 peripheral sleep clock  register,                               Address offset: 0x10C */
    volatile uint32_t APB1LLPENR;     /*!< RCC APB1 peripheral sleep clock  Low Word register,                      Address offset: 0x110 */
    volatile uint32_t APB1HLPENR;     /*!< RCC APB1 peripheral sleep clock  High Word register,                     Address offset: 0x114 */
    volatile uint32_t APB2LPENR;      /*!< RCC APB2 peripheral sleep clock  register,                               Address offset: 0x118 */
    volatile uint32_t APB4LPENR;      /*!< RCC APB4 peripheral sleep clock  register,                               Address offset: 0x11C */
    uint32_t     RESERVED13[4];   /*!< Reserved, 0x120-0x12C                                                    Address offset: 0x120 */
} DTM_RCC_Type;


#ifndef DTM_CLK_BASE
#define DTM_CLK_BASE 0x58024400UL
#endif

#define RCC ((DTM_RCC_Type *)DTM_CLK_BASE)


// const RCC_CFGR_SW_HSI = 0;
// const RCC_CFGR_SW_CSI = 1;
// const RCC_CFGR_SW_HSE = 2;
// const RCC_CFGR_SW_PLL1 = 3;

// const RCC_PLLSOURCE_HSI = 0b00;
// const RCC_PLLSOURCE_CSI = 0b01;
// const RCC_PLLSOURCE_HSE = 0b10;
// const RCC_PLLSOURCE_NONE = 0b11;

// pub fn clock_init() void {

//     // Enable power interface clock vos3 and latency4 max freq = 180Mhz
//     regs.Flash.ACR.modify(.{ .LATENCY = 4 });

//     // HSI used as PLL clock source; freq:200Mhz
//     regs.RCC.PLLCKSELR.modify(.{ .PLLSRC = RCC_PLLSOURCE_HSI, .DIVM1 = 4 });
//     regs.RCC.PLLCFGR.modify(.{ .PLL1VCOSEL = 0, .PLL1RGE = 0b11, .PLL1FRACEN = 0, .DIVP1EN = 1, .DIVQ1EN = 1, .DIVR1EN = 1 });
//     regs.RCC.PLL1DIVR.modify(.{ .DIVN1 = 24, .DIVP1 = 1 });

//     // clear clock interrupt
//     regs.RCC.CIER.raw = 0;

//     // Enable PLL
//     regs.RCC.CR.modify(.{ .PLL1ON = 1 });
//     // Wait for PLL ready
//     while (regs.RCC.CR.read().PLL1RDY != 1) {
//         cpu.nop();
//     }
//     // Use PLL as system clock
//     regs.RCC.CFGR.modify(.{ .SW = RCC_CFGR_SW_PLL1 });
//     while (regs.RCC.CFGR.read().SWS != RCC_CFGR_SW_PLL1) {
//         cpu.nop();
//     }

//     // init systick
//     cpu.peripherals.SysTick.LOAD.raw = 0xFFFFFF;
//     cpu.peripherals.SysTick.CTRL.modify(.{ .ENABLE = 1, .CLKSOURCE = 1 });
// }

uint32_t sysclk = 0, pll_vco = 0, pll_p_ck = 0, pll_q_ck = 0, hclk = 0, pclk1 = 0, pclk2 = 0;

/**
 * @brief       Clock setting function
 * @param       plln: Main PLL multiplication factor (PLL multiplication), range: 4~512.
 * @param       pllm: Main PLL and audio PLL predivision factor (predivision before PLL), range: 1~63.
 * @param       pllp: Main PLL p division factor (division after PLL), divided as system clock, range: 2, 4, 6, 8...128. (must be a multiple of 2)
 * @param       pllq: Main PLL q division factor (division after PLL), range: DIVM1:2; DIVM2:1~128; DIVM3:1~128.
 * @param       pllr: Main PLL r division factor (division after PLL), range: 2.
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
 *              D1CPRE Prescaler = 1, CPU clock frequency = Fsys
 *              HPRE Prescaler = 2, AHB clock frequency = Fsys / 2
 *              
 *
 *              When the external crystal is 8M, 480M recommended values: plln = 120, pllm = 1, pllp = 2, pllq = 2, pllr = 2.
 *              Get: Fvco = 8 * (120 / 1) = 960Mhz
 *                   Fsys = pll_p_ck = 960 / 2 = 480Mhz
 *                   Fq   = pll_q_ck = 960 / 2 = 480Mhz
 *
 *              The default frequencies that need to be configured for H743 are as follows:
 *              CPU frequency (HCLK) = pll_p_ck = 480Mhz
 *              AHB1/2(rcc_hclk1/2) = 480Mhz
 *              APB1(rcc_pclk1) = pll_p_ck / 4 = 42Mhz
 *              APB2(rcc_pclk2) = pll_p_ck / 2 = 84Mhz
 *
 * @retval      Error code: 0, success; 1, HSE error; 2, PLL1 error; 3, PLL2 error; 4, clock switch error;
 */
uint8_t sys_clock_set(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllr)
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
        // RCC->APB1ENR |= 1 << 28;                /* Enable power interface clock */

        /* Set main PLL predivision factor,   DIVM1 PLLCKSELR[10:4]: 1~63 */
        RCC->PLLCKSELR &= ~(0x3F << 4);          /* Clear PLLM bits */
        RCC->PLLCKSELR |= pllm << 4;
        /* Set main PLL clock source,         PLLCKSELR[1:0]: 0b00: HSI, 0b01: CSI, 0b10: HSE, 0b11: reserved */
        RCC->PLLCKSELR &= ~(0b11 << 0);          /* Clear PLL source bits */
        RCC->PLLCKSELR |= 0b10 << 0;            /* Set main PLL clock source to HSE */
        
        /* Set main PLL VCO PLLCFGR[1] wide.  0: Wide VCO range: 192 to 836 MHz (default after reset) 1: Medium VCO range: 150 to 420 MHz */
        RCC->PLLCFGR &= ~(1 << 1);          /* Clear PLL VCO range bit */
        /* Set main PLL input ref range PLLCFGR[3:2] 11: The PLL1 input (ref1_ck) clock range frequency is between 8 and 16 MHz */
        RCC->PLLCFGR |= 0b11 << 2;          /* Set PLL input ref range to 8~16MHz */
        /* Clear PLL1FRACEN PLLCFGR[4] */
        RCC->PLLCFGR &= ~(1 << 4);          /* Clear PLL1 fractional enable bit */
        /* Enable main PLL P Q R PLLCFGR[18：16] */
        RCC->PLLCFGR |= (1 << 16) | (1 << 17) | (1 << 18); /* Enable main PLL P Q R division */

        /* Set main PLL multiplication factor, PLL1DIVR[8:0] 0x003: DIVN1 = 4 0x004: DIVN1 = 5 */
        RCC->PLL1DIVR &= ~(0x1FF << 0);         /* Clear PLLN bits */
        RCC->PLL1DIVR |= (plln - 1) << 0;            /* Set main PLL multiplication factor, PLLN[8:0]: 4~512 */
        /* Set main PLL p division factor, PLL1DIVR[15:9] 0000001: pll1_p_ck = vco1_ck / 2 0000011: pll1_p_ck = vco1_ck / 4 */
        RCC->PLL1DIVR &= ~(0x7F << 9);         /* Clear PLLP bits */
        RCC->PLL1DIVR |= (pllp - 1) << 9; /* Set main PLL p division factor, PLLP[6:0]: 2~128, must be a multiple of 2 */
        /* Set main PLL q division factor, PLL1DIVR[22:16] 0000001: pll1_q_ck = vco1_ck / 2 */
        RCC->PLL1DIVR &= ~(0x7F << 16);        /* Clear PLLQ bits */
        RCC->PLL1DIVR |= (pllq - 1) << 16; /* Set main PLL q division factor, PLLQ[6:0]: 2~128 */
        /* Set main PLL r division factor, PLL1DIVR[30:24] 0000001: pll1_r_ck = vco1_ck / 2 */
        RCC->PLL1DIVR &= ~(0x7F << 24);        /* Clear PLLR bits */
        RCC->PLL1DIVR |= (pllr - 1) << 24; /* Set main PLL r division factor, PLLR[6:0]: 2~128 */

        /* RCC Domain 1 Clock Configuration Register */
        /* Core prescaler not divided D1CFGR[11:8] 0xxx: sys_ck not divided (default after reset) 1000: sys_ck divided by 2 */
        RCC->D1CFGR &= ~(0xF << 8);            /* Clear D1CPRE bits */
        /* AHB prescaler 2 divided D1CFGR[3:0] 0xxx: rcc_hclk3 = sys_d1cpre_ck  1000: rcc_hclk3 = sys_d1cpre_ck / 2]  */
        RCC->D1CFGR &= ~(0xF << 0);            /* Clear HPRE bits */
        RCC->D1CFGR |= 0b1000 << 0;           /* Set HPRE[3:0] = 8, AHB prescaler 2 divided, rcc_hclk1/2/3 = sys_d1cpre_ck / 2 */

        /* APB1/2/3/4 2 division D2PPRE1 D2CFGR[6:4] /D2PPRE2 D2CFGR[10:8]/D1PPRE D1CFGR[6:4] 100: rcc_pclk3 = rcc_hclk3 / 2 101: rcc_pclk3 = rcc_hclk3 / 4 */
        RCC->D2CFGR &= ~(0x7 << 4);            /* Clear PPRE1 bits */
        RCC->D2CFGR |= 0b100 << 4;            /* Set PPRE1[2:0] = 4, APB1 2 division, rcc_pclk1 = rcc_hclk3 / 2 */
        RCC->D2CFGR &= ~(0x7 << 8);            /* Clear PPRE2 bits */
        RCC->D2CFGR |= 0b100 << 8;            /* Set PPRE2[2:0] = 4, APB2 2 division, rcc_pclk2 = rcc_hclk3 / 2 */
        RCC->D1CFGR &= ~(0x7 << 4);            /* Clear PPRE3 bits */
        RCC->D1CFGR |= 0b100 << 4;            /* Set PPRE3[2:0] = 4, APB3 2 division, rcc_pclk3 = rcc_hclk3 / 2 */

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

        
        RCC->CFGR |= 3 << 0;                    /* Select main PLL as system clock */
        
        retry = 0;
        while (swsval != 3)                     /* Wait for successful switch of system clock source to pll_p_ck */
        {
            swsval = (RCC->CFGR & 0X38) >> 3;   /* Get SWS[2:0] status to check if switch is successful */
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
 * @param       pllr: PLL1 r division factor (division after PLL), range: 1~128.
 * @retval      None
 */
void sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllr)
{
    RCC->CR = 0x00000001;           /* Set HISON, enable internal high-speed RC oscillator, clear other bits */
    RCC->CFGR = 0x00000000;         /* Clear CFGR */
    RCC->PLLCFGR = 0x00000000;      /* Clear PLLCFGR */
    RCC->CIER = 0x00000000;          /* Clear CIR */
    
    sys_clock_set(plln, pllm, pllp, pllq, pllr);  /* Set clock */
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
    sys_stm32_clock_init(120, 1, 2, 2, 2); /* Set PLL1 parameters for 168MHz system clock */

    /* Enable GPIOA and GPIOF clocks */
#ifdef DTM_GPIOA_CLK_EN
    RCC->AHB4ENR |= DTM_GPIOA_CLK_EN;  /* Enable GPIOA clock */
#endif
#ifdef DTM_GPIOB_CLK_EN
    RCC->AHB4ENR |= DTM_GPIOB_CLK_EN;  /* Enable GPIOB clock */
#endif
#ifdef DTM_GPIOF_CLK_EN
    RCC->AHB4ENR |= DTM_GPIOF_CLK_EN;  /* Enable GPIOF clock */
#endif

    /* Enable USART1 and USART2 clocks */
#ifdef DTM_UART1_CLK_EN
    RCC->APB2ENR |= DTM_UART1_CLK_EN;  /* Enable USART1 clock */
#endif
#ifdef DTM_UART2_CLK_EN
    RCC->APB1LENR |= DTM_UART2_CLK_EN;  /* Enable USART2 clock */
#endif

}

#endif /* DTM_BINFDING_CLK_ARM_M4_ST_GD */
