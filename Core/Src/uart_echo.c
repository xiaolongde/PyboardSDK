/**
  ******************************************************************************
  * @file           : uart_echo.c
  * @brief          : UART Echo Driver Implementation with DMA support
  * @description    : USART1 on PB6(TX)/PB7(RX), 921600 baud, Echo-Reply mode
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_echo.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
UART_Echo_HandleTypeDef g_uart_echo = {0};

/* Private function prototypes -----------------------------------------------*/
static void UART_Echo_GPIO_Init(void);
static void UART_Echo_DMA_Init(void);
static void UART_Echo_NVIC_Init(void);
static void UART_Echo_StartReceive(void);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize UART Echo module
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Echo_Init(void)
{
    HAL_StatusTypeDef status;
    
    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    /* Initialize GPIO */
    UART_Echo_GPIO_Init();
    
    /* Initialize DMA */
    UART_Echo_DMA_Init();
    
    /* Configure UART */
    g_uart_echo.huart.Instance = UART_ECHO_INSTANCE;
    g_uart_echo.huart.Init.BaudRate = UART_ECHO_BAUDRATE;
    g_uart_echo.huart.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart_echo.huart.Init.StopBits = UART_STOPBITS_1;
    g_uart_echo.huart.Init.Parity = UART_PARITY_NONE;
    g_uart_echo.huart.Init.Mode = UART_MODE_TX_RX;
    g_uart_echo.huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart_echo.huart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    status = HAL_UART_Init(&g_uart_echo.huart);
    if (status != HAL_OK)
    {
        return status;
    }
    
    /* Link DMA handles to UART */
    __HAL_LINKDMA(&g_uart_echo.huart, hdmatx, g_uart_echo.hdma_tx);
    __HAL_LINKDMA(&g_uart_echo.huart, hdmarx, g_uart_echo.hdma_rx);
    
    /* Initialize NVIC */
    UART_Echo_NVIC_Init();
    
    /* Enable IDLE line interrupt for receiving variable length data */
    __HAL_UART_ENABLE_IT(&g_uart_echo.huart, UART_IT_IDLE);
    
    /* Reset buffer indices */
    g_uart_echo.rx_write_index = 0;
    g_uart_echo.rx_read_index = 0;
    g_uart_echo.tx_busy = 0;
    
    /* Start DMA reception */
    UART_Echo_StartReceive();
    
    return HAL_OK;
}

/**
  * @brief  De-initialize UART Echo module
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Echo_DeInit(void)
{
    /* Stop DMA */
    HAL_UART_DMAStop(&g_uart_echo.huart);
    
    /* Disable interrupts */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    
    /* De-init UART */
    HAL_UART_DeInit(&g_uart_echo.huart);
    
    /* De-init DMA */
    HAL_DMA_DeInit(&g_uart_echo.hdma_tx);
    HAL_DMA_DeInit(&g_uart_echo.hdma_rx);
    
    /* De-init GPIO */
    HAL_GPIO_DeInit(UART_ECHO_TX_GPIO_PORT, UART_ECHO_TX_PIN);
    HAL_GPIO_DeInit(UART_ECHO_RX_GPIO_PORT, UART_ECHO_RX_PIN);
    
    /* Disable clocks */
    __HAL_RCC_USART1_CLK_DISABLE();
    
    return HAL_OK;
}

/**
  * @brief  Process echo (call this in main loop)
  * @note   This checks for received data and echoes it back
  */
void UART_Echo_Process(void)
{
    uint8_t temp_buffer[UART_ECHO_RX_BUFFER_SIZE];
    uint16_t count;
    
    /* Check if there's data to echo and TX is not busy */
    count = UART_Echo_GetRxCount();
    if (count > 0 && !g_uart_echo.tx_busy)
    {
        /* Read available data */
        count = UART_Echo_Read(temp_buffer, count);
        
        if (count > 0)
        {
            /* Echo it back */
            UART_Echo_Transmit(temp_buffer, count);
        }
    }
}

/**
  * @brief  Transmit data via DMA
  * @param  data: pointer to data buffer
  * @param  len: length of data
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Echo_Transmit(uint8_t *data, uint16_t len)
{
    if (g_uart_echo.tx_busy)
    {
        return HAL_BUSY;
    }
    
    if (len > UART_ECHO_TX_BUFFER_SIZE)
    {
        len = UART_ECHO_TX_BUFFER_SIZE;
    }
    
    /* Copy to TX buffer */
    memcpy(g_uart_echo.tx_buffer, data, len);
    
    /* Mark as busy */
    g_uart_echo.tx_busy = 1;
    
    /* Start DMA transmission */
    return HAL_UART_Transmit_DMA(&g_uart_echo.huart, g_uart_echo.tx_buffer, len);
}

/**
  * @brief  Get number of bytes available in RX buffer
  * @retval Number of bytes available
  */
uint16_t UART_Echo_GetRxCount(void)
{
    uint16_t write_idx = g_uart_echo.rx_write_index;
    uint16_t read_idx = g_uart_echo.rx_read_index;
    
    if (write_idx >= read_idx)
    {
        return write_idx - read_idx;
    }
    else
    {
        return UART_ECHO_RX_BUFFER_SIZE - read_idx + write_idx;
    }
}

/**
  * @brief  Read data from RX buffer
  * @param  data: pointer to destination buffer
  * @param  max_len: maximum bytes to read
  * @retval Number of bytes actually read
  */
uint16_t UART_Echo_Read(uint8_t *data, uint16_t max_len)
{
    uint16_t count = 0;
    uint16_t available = UART_Echo_GetRxCount();
    
    if (max_len > available)
    {
        max_len = available;
    }
    
    while (count < max_len)
    {
        data[count++] = g_uart_echo.rx_buffer[g_uart_echo.rx_read_index];
        g_uart_echo.rx_read_index = (g_uart_echo.rx_read_index + 1) % UART_ECHO_RX_BUFFER_SIZE;
    }
    
    return count;
}

/**
  * @brief  UART IRQ Handler (call from USART1_IRQHandler)
  */
void UART_Echo_IRQHandler(void)
{
    /* Check for IDLE line interrupt */
    if (__HAL_UART_GET_FLAG(&g_uart_echo.huart, UART_FLAG_IDLE))
    {
        /* Clear IDLE flag */
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart_echo.huart);
        
        /* Calculate received bytes from DMA counter */
        uint16_t current_counter = __HAL_DMA_GET_COUNTER(&g_uart_echo.hdma_rx);
        uint16_t received;
        
        if (g_uart_echo.last_dma_counter >= current_counter)
        {
            received = g_uart_echo.last_dma_counter - current_counter;
        }
        else
        {
            /* DMA has wrapped around */
            received = g_uart_echo.last_dma_counter + (UART_ECHO_RX_BUFFER_SIZE - current_counter);
        }
        
        /* Update write index */
        g_uart_echo.rx_write_index = (g_uart_echo.rx_write_index + received) % UART_ECHO_RX_BUFFER_SIZE;
        
        /* Update last counter */
        g_uart_echo.last_dma_counter = current_counter;
    }
    
    /* Handle other UART interrupts */
    HAL_UART_IRQHandler(&g_uart_echo.huart);
}

/**
  * @brief  DMA TX Complete callback (call from DMA2_Stream7_IRQHandler)
  */
void UART_Echo_DMA_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_uart_echo.hdma_tx);
}

/**
  * @brief  DMA RX callback (call from DMA2_Stream5_IRQHandler)
  */
void UART_Echo_DMA_RX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_uart_echo.hdma_rx);
}

/* HAL Callbacks -------------------------------------------------------------*/

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: pointer to UART handle
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_ECHO_INSTANCE)
    {
        g_uart_echo.tx_busy = 0;
    }
}

/**
  * @brief  Rx Half Transfer completed callback (circular mode)
  * @param  huart: pointer to UART handle
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_ECHO_INSTANCE)
    {
        /* Update write index for first half of buffer */
        uint16_t current_counter = __HAL_DMA_GET_COUNTER(&g_uart_echo.hdma_rx);
        uint16_t received;
        
        if (g_uart_echo.last_dma_counter >= current_counter)
        {
            received = g_uart_echo.last_dma_counter - current_counter;
        }
        else
        {
            received = g_uart_echo.last_dma_counter + (UART_ECHO_RX_BUFFER_SIZE - current_counter);
        }
        
        g_uart_echo.rx_write_index = (g_uart_echo.rx_write_index + received) % UART_ECHO_RX_BUFFER_SIZE;
        g_uart_echo.last_dma_counter = current_counter;
    }
}

/**
  * @brief  Rx Transfer completed callback (circular mode - full buffer)
  * @param  huart: pointer to UART handle
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_ECHO_INSTANCE)
    {
        /* Update write index for second half of buffer (wrap around) */
        uint16_t current_counter = __HAL_DMA_GET_COUNTER(&g_uart_echo.hdma_rx);
        uint16_t received;
        
        if (g_uart_echo.last_dma_counter >= current_counter)
        {
            received = g_uart_echo.last_dma_counter - current_counter;
        }
        else
        {
            received = g_uart_echo.last_dma_counter + (UART_ECHO_RX_BUFFER_SIZE - current_counter);
        }
        
        g_uart_echo.rx_write_index = (g_uart_echo.rx_write_index + received) % UART_ECHO_RX_BUFFER_SIZE;
        g_uart_echo.last_dma_counter = current_counter;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize GPIO for UART
  */
static void UART_Echo_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* TX Pin: PB6 */
    GPIO_InitStruct.Pin = UART_ECHO_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_ECHO_AF;
    HAL_GPIO_Init(UART_ECHO_TX_GPIO_PORT, &GPIO_InitStruct);
    
    /* RX Pin: PB7 */
    GPIO_InitStruct.Pin = UART_ECHO_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_ECHO_AF;
    HAL_GPIO_Init(UART_ECHO_RX_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Initialize DMA for UART TX and RX
  */
static void UART_Echo_DMA_Init(void)
{
    /* DMA TX: DMA2 Stream7 Channel4 */
    g_uart_echo.hdma_tx.Instance = UART_ECHO_DMA_TX_STREAM;
    g_uart_echo.hdma_tx.Init.Channel = UART_ECHO_DMA_TX_CHANNEL;
    g_uart_echo.hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_uart_echo.hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_uart_echo.hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    g_uart_echo.hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_uart_echo.hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_uart_echo.hdma_tx.Init.Mode = DMA_NORMAL;
    g_uart_echo.hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    g_uart_echo.hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&g_uart_echo.hdma_tx);
    
    /* DMA RX: DMA2 Stream5 Channel4 (Circular mode for continuous reception) */
    g_uart_echo.hdma_rx.Instance = UART_ECHO_DMA_RX_STREAM;
    g_uart_echo.hdma_rx.Init.Channel = UART_ECHO_DMA_RX_CHANNEL;
    g_uart_echo.hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_uart_echo.hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_uart_echo.hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_uart_echo.hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_uart_echo.hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_uart_echo.hdma_rx.Init.Mode = DMA_CIRCULAR;  /* Circular mode for continuous reception */
    g_uart_echo.hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    g_uart_echo.hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&g_uart_echo.hdma_rx);
}

/**
  * @brief  Initialize NVIC for UART and DMA interrupts
  */
static void UART_Echo_NVIC_Init(void)
{
    /* USART1 IRQ - for IDLE line detection */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    /* DMA2 Stream7 IRQ - TX Complete */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    
    /* DMA2 Stream5 IRQ - RX */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

/**
  * @brief  Start DMA reception in circular mode
  */
static void UART_Echo_StartReceive(void)
{
    /* Initialize last DMA counter */
    g_uart_echo.last_dma_counter = UART_ECHO_RX_BUFFER_SIZE;
    
    /* Start DMA reception in circular mode */
    HAL_UART_Receive_DMA(&g_uart_echo.huart, g_uart_echo.rx_buffer, UART_ECHO_RX_BUFFER_SIZE);
}
