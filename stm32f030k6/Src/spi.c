/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];
uint8_t SPI_tx_buf[32];
uint8_t SPI_rx_buf[32];
extern uint8_t m;
 uint8_t l,p;
 SPI_InitTypeDef SPI_InitStruct;
#define Read_Enc_A			HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_9)
#define Read_Enc_B			HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_10)
#define	ENC_MAX					3
uint32_t Enc_counter=0xFFFF;
uint16_t Enc_counter1,Enc_counter2=0;
uint8_t Enc_Mode=0;
uint8_t Enc_A_count=0;
uint8_t Enc_A_state=0;
uint8_t Enc_B_count=0;
uint8_t Enc_B_state=0;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

 

}

int32_t SPI1_Initialize (void) {
  
	SPI_InitTypeDef SPI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
  
	/* GPIO Ports Clock Enable */

	__GPIOA_CLK_ENABLE();
   /**SPI1 GPIO Configuration    
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate= GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	
	/* Configure GPIO pins: PD2 - SS Signal */
  GPIO_InitStruct.Pin   = GPIO_PIN_4;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	/* SPI1 Clock Enable */
  __SPI1_CLK_ENABLE();
	
//	HAL_SPI_MspDeInit(&hspi1);
	/* Configure SPI Init */      
  SPI_InitStruct.Mode = SPI_MODE_SLAVE;
	SPI_InitStruct.Direction = SPI_DIRECTION_2LINES; /* default */
	SPI_InitStruct.DataSize = SPI_DATASIZE_8BIT;
	SPI_InitStruct.CLKPolarity = SPI_POLARITY_LOW;
	SPI_InitStruct.CLKPhase = SPI_PHASE_1EDGE; /* default */
	SPI_InitStruct.NSS = SPI_NSS_HARD_INPUT;
	SPI_InitStruct.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	SPI_InitStruct.FirstBit = SPI_FIRSTBIT_MSB; /* default */
	SPI_InitStruct.TIMode = SPI_TIMODE_DISABLED; /* default */
	SPI_InitStruct.CRCCalculation = SPI_CRCCALCULATION_DISABLED; /* default */
	SPI_InitStruct.CRCPolynomial = 0; /* default, no need without CRC */
		
  /* Configure SPI Handler */	  
	hspi1.Instance = SPI1;
	hspi1.Init = SPI_InitStruct;
	hspi1.pRxBuffPtr = &SPI_tx_buf[0];
	hspi1.TxXferSize = 1;
	hspi1.TxXferCount = 1;
	hspi1.pRxBuffPtr = &SPI_rx_buf[0];
	hspi1.RxXferSize = 1;
	hspi1.RxXferCount = 1;
	hspi1.Lock = HAL_UNLOCKED;
	hspi1.State = HAL_SPI_STATE_RESET;
	hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
	
	HAL_SPI_Init(&hspi1);
	
	  /* Peripheral interrupt init*/
   HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
		__HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_RXNE);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
		HAL_SPI_Receive_IT(&hspi1, &l,  1);
		
  return 0;
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(SPI1_IRQn);

  }
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
void Systick_Enc(void)
{
	if (Read_Enc_A == 1)
	{
		if (Enc_A_count < ENC_MAX)
		{
			Enc_A_count++;
		} else {
			if (Enc_A_state == 0)
			{
				if (Enc_Mode == 0)
				{
					Enc_Mode = 1;
					Enc_counter++;
				} else {
					Enc_Mode = 0;
					Enc_counter--;
				}
			}
			Enc_A_state = 1;
		}
	} else {
		if (Enc_A_count > 0)
		{
			Enc_A_count--;
		} else {
			if (Enc_A_state == 1)	
			{
				if (Enc_Mode == 0)
				{
					Enc_Mode = 1;
					Enc_counter++;
				} else {
					Enc_Mode = 0;
					Enc_counter--;
				}
			}
			Enc_A_state = 0;
		}
	}
	
	if (Read_Enc_B == 1)
	{
		if (Enc_B_count < ENC_MAX)
		{
			Enc_B_count++;
		} else {
			if (Enc_B_state == 0)
			{
				if (Enc_Mode == 0)
				{
					Enc_Mode = 1;
					Enc_counter--;
				} else {
					Enc_Mode = 0;
					Enc_counter++;
				}
			}
			Enc_B_state = 1;
		}
	} else {
		if (Enc_B_count > 0)
		{
			Enc_B_count--;
		} else {
			if (Enc_B_state == 1)	
			{
				if (Enc_Mode == 0)
				{
					Enc_Mode = 1;
					Enc_counter--;
				} else {
					Enc_Mode = 0;
					Enc_counter++;
				}
			}
			Enc_B_state = 0;
		}
	}
	Enc_counter1=Enc_counter/4;
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
