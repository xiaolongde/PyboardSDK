/**
  ******************************************************************************
  * @file           : uart_echo.h
  * @brief          : UART Echo Driver with DMA support
  * @description    : USART1 configuration on PB6(TX) and PB7(RX)
  *                   Baudrate: 921600, Echo-Reply mode
  ******************************************************************************
  */

#ifndef __UART_ECHO_H
#define __UART_ECHO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported defines ----------------------------------------------------------*/
#define UART_ECHO_INSTANCE          USART1
#define UART_ECHO_BAUDRATE          921600

/* TX: PB6, RX: PB7 */
#define UART_ECHO_TX_GPIO_PORT      GPIOB
#define UART_ECHO_TX_PIN            GPIO_PIN_6
#define UART_ECHO_RX_GPIO_PORT      GPIOB
#define UART_ECHO_RX_PIN            GPIO_PIN_7
#define UART_ECHO_AF                GPIO_AF7_USART1

/* DMA Configuration */
/* USART1_TX -> DMA2 Stream7 Channel4 */
/* USART1_RX -> DMA2 Stream2 Channel4 or Stream5 Channel4 */
#define UART_ECHO_DMA_TX_STREAM     DMA2_Stream7
#define UART_ECHO_DMA_TX_CHANNEL    DMA_CHANNEL_4
#define UART_ECHO_DMA_RX_STREAM     DMA2_Stream5
#define UART_ECHO_DMA_RX_CHANNEL    DMA_CHANNEL_4

/* Buffer sizes */
#define UART_ECHO_RX_BUFFER_SIZE    256
#define UART_ECHO_TX_BUFFER_SIZE    256

/* Exported types ------------------------------------------------------------*/
typedef struct {
    UART_HandleTypeDef huart;
    DMA_HandleTypeDef hdma_tx;
    DMA_HandleTypeDef hdma_rx;
    
    uint8_t rx_buffer[UART_ECHO_RX_BUFFER_SIZE];
    uint8_t tx_buffer[UART_ECHO_TX_BUFFER_SIZE];
    
    volatile uint16_t rx_write_index;
    volatile uint16_t rx_read_index;
    volatile uint8_t tx_busy;
    
    /* IDLE line detection for variable length reception */
    volatile uint16_t last_dma_counter;
} UART_Echo_HandleTypeDef;

/* Exported variables --------------------------------------------------------*/
extern UART_Echo_HandleTypeDef g_uart_echo;
extern UART_Echo_HandleTypeDef * const g_uart_echo_ptr;

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Get UART Echo handle (for debugging)
  * @retval Pointer to g_uart_echo
  */
UART_Echo_HandleTypeDef* UART_Echo_GetHandle(void);

/**
  * @brief  Initialize UART Echo module
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Echo_Init(void);

/**
  * @brief  De-initialize UART Echo module
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Echo_DeInit(void);

/**
  * @brief  Process echo (call this in main loop)
  * @note   This function checks for received data and echoes it back
  */
void UART_Echo_Process(void);

/**
  * @brief  Transmit data via DMA
  * @param  data: pointer to data buffer
  * @param  len: length of data
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Echo_Transmit(uint8_t *data, uint16_t len);

/**
  * @brief  Get number of bytes available in RX buffer
  * @retval Number of bytes available
  */
uint16_t UART_Echo_GetRxCount(void);

/**
  * @brief  Read data from RX buffer
  * @param  data: pointer to destination buffer
  * @param  max_len: maximum bytes to read
  * @retval Number of bytes actually read
  */
uint16_t UART_Echo_Read(uint8_t *data, uint16_t max_len);

/**
  * @brief  UART IRQ Handler (call from USART1_IRQHandler)
  */
void UART_Echo_IRQHandler(void);

/**
  * @brief  DMA TX Complete callback (call from DMA2_Stream7_IRQHandler)
  */
void UART_Echo_DMA_TX_IRQHandler(void);

/**
  * @brief  DMA RX callback (call from DMA2_Stream5_IRQHandler)
  */
void UART_Echo_DMA_RX_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_ECHO_H */
