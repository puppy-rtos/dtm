
#include "dtm_bindings.h"

#ifdef DTM_BINFDING_UART_ARM_M4_ST_GD
#include "dtm_gpio.h"
#include "stdint.h"

typedef struct
{
  volatile uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  volatile uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  volatile uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  volatile uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  volatile uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  volatile uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  volatile uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

#define USART_X ((USART_TypeDef *)DTM_DEBUG_UART) /* Define the base address of USART1, change to DTM_UART2_BASE for USART2 */

int dtm_uart_putc(char ch)
{
    while ((USART_X->SR & 0X40) == 0);     /* Wait for the previous character to be sent */

    USART_X->DR = (uint8_t)ch;             /* Write the character ch to the DR register to be sent */
    return ch;
}

/**
 * @brief       USARTX initialization function
 * @param       sclk: Clock source frequency of USARTX (unit: MHz)
 *              USART1 and USART6 use: rcc_pclk2 = 84Mhz
 *              USART2 - 5 / 7 / 8 use: rcc_pclk1 = 42Mhz
 * @note        Note: sclk must be set correctly, otherwise the baud rate of the serial port will be set abnormally.
 * @param       baudrate: Baud rate, set the baud rate value according to your needs
 * @retval      None
 */
void dtm_uart_init(uint32_t sclk, uint32_t baudrate)
{
    uint32_t temp;
    /* IO and clock configuration */

    dtm_gpio_set(DTM_DEBUG_UART_TX_PORT, DTM_DEBUG_UART_TX_PIN,
                 DTM_GPIO_MODE_AF, DTM_GPIO_OTYPE_PP, DTM_GPIO_SPEED_MID, DTM_GPIO_PUPD_PU);    /* Set the mode of the TX pin */

    dtm_gpio_set(DTM_DEBUG_UART_RX_PORT, DTM_DEBUG_UART_RX_PIN,
                 DTM_GPIO_MODE_AF, DTM_GPIO_OTYPE_PP, DTM_GPIO_SPEED_MID, DTM_GPIO_PUPD_PU);    /* Set the mode of the RX pin */

    dtm_gpio_af_set(DTM_DEBUG_UART_TX_PORT, DTM_DEBUG_UART_TX_PIN, DTM_DEBUG_UART_TX_AF);    /* Select the TX pin multiplexing function, must be set correctly */
    dtm_gpio_af_set(DTM_DEBUG_UART_RX_PORT, DTM_DEBUG_UART_RX_PIN, DTM_DEBUG_UART_RX_AF);    /* Select the RX pin multiplexing function, must be set correctly */

    temp = (sclk * 1000000 + baudrate / 2) / baudrate;              /* Get USARTDIV@OVER8 = 0, calculate with rounding */
    /* Baud rate setting */
    USART_X->BRR = temp;       /* Baud rate setting @OVER8 = 0 */
    USART_X->CR1 = 0;          /* Clear the CR1 register */
    USART_X->CR1 |= 0 << 12;   /* Set M = 0, select 8-bit word length */
    USART_X->CR1 |= 0 << 15;   /* Set OVER8 = 0, 16 times oversampling */
    USART_X->CR1 |= 1 << 3;    /* Enable USART transmission */
#if USART_EN_RX  /* If reception is enabled */
    /* Enable receive interrupt */
    USART_X->CR1 |= 1 << 2;    /* Enable USART reception */
    USART_X->CR1 |= 1 << 5;    /* Enable receive buffer not empty interrupt */
    dtm_nvic_init(3, 3, USART_X_IRQn, 2); /* Group 2, lowest priority */
#endif
    USART_X->CR1 |= 1 << 13;   /* Enable USART */
}

#endif /* DTM_BINFDING_UART_ARM_M4_ST_GD */
