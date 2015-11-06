/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include "EtherShield.h"
#define NET_BUF_SIZE (1<<10)
#define UART_BUF_SIZE 1

#define MAX(a,b) ((a)>(b)?(a):(b))
#define BUF_SIZE MAX(NET_BUF_SIZE, UART_BUF_SIZE)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void error (float error_num, char infinite) {
	//printf("%u\r\n", error_num);
	if (infinite)
		while (1) {
			int i = 0;
			while (i++ < ((int)((float)(error_num) / 1) + 1) ) {
				GPIOA->BSRR = GPIO_PIN_5;
				HAL_Delay(1000 / error_num);
				GPIOA->BSRR = GPIO_PIN_5 << 16;
				HAL_Delay(1000 / error_num);
			}
		};

	GPIOA->BSRR = GPIO_PIN_5;
	HAL_Delay(1000);
	GPIOA->BSRR = GPIO_PIN_5 << 16;

	int i=0;
	while (i++ < error_num) {
		GPIOA->BSRR = GPIO_PIN_5;
		HAL_Delay(300);
		GPIOA->BSRR = GPIO_PIN_5 << 16;
		HAL_Delay(200);
	}

	HAL_Delay(500);
	NVIC_SystemReset();

	return;
}

static inline void blink(int times, int delay) {
	int i = 0;
	while (i++ < times) {
		GPIOA->BSRR = GPIO_PIN_5;
		HAL_Delay(delay);
		GPIOA->BSRR = GPIO_PIN_5 << 16;
		HAL_Delay(delay);
	}

	return;
}

void ES_PingCallback(void) {
}

uint16_t get_udp_data_len(uint8_t *buf)
{
	int16_t i;
	i=(((int16_t)buf[IP_TOTLEN_H_P])<<8)|(buf[IP_TOTLEN_L_P]&0xff);
	i-=IP_HEADER_LEN;
	i-=8;
	if (i<=0){
		i=0;
	}
	return((uint16_t)i);
}

static uint16_t info_data_len = 0;
uint16_t packetloop_icmp_udp(uint8_t *buf,uint16_t plen)
{
	if(eth_type_is_arp_and_my_ip(buf,plen)){
		if (buf[ETH_ARP_OPCODE_L_P]==ETH_ARP_OPCODE_REQ_L_V){
			// is it an arp request 
			make_arp_answer_from_request(buf);
		}
		return(0);

	}
	// check if ip packets are for us:
	if(eth_type_is_ip_and_my_ip(buf,plen)==0){
		return(0);
	}

	if(buf[IP_PROTO_P]==IP_PROTO_ICMP_V && buf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
		make_echo_reply_from_request(buf,plen);
		return(0);
	}

	if (buf[IP_PROTO_P]==IP_PROTO_UDP_V) {
		info_data_len=get_udp_data_len(buf);
		return(IP_HEADER_LEN+8+14);
	}

	return(0);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	static uint8_t uart_buf[UART_BUF_SIZE + 1];

	uint8_t mac[] = {02, 03, 04, 05, 06, 07};
	uint8_t ip[]  = {10,4,33,123};
	static uint8_t net_buf[NET_BUF_SIZE + 1];
	static uint8_t buf[BUF_SIZE + 1];

	/* // USART echo node
	while (1)
	{
		if (HAL_UART_Receive(&huart3, uart_buf, UART_BUF_SIZE, 0) == HAL_OK) {
			HAL_UART_Transmit(&huart3, uart_buf, 1, 0xffffffff);
		}
	}
	*/

	ES_enc28j60SpiInit(&hspi1);
	ES_enc28j60Init(mac);

	uint8_t enc28j60_rev = ES_enc28j60Revision();

	if (enc28j60_rev <= 0)
		error(2, 0);

	ES_init_ip_arp_udp_tcp(mac, ip, 80);
	uint16_t dat_p = 0;

	/* // UDP echo server
	while (1)
	{
		dat_p = packetloop_icmp_udp(net_buf, ES_enc28j60PacketReceive(NET_BUF_SIZE, net_buf));
		if (dat_p != 0) {
			memcpy(buf, &net_buf[dat_p], info_data_len);
			make_udp_reply_from_request(net_buf, (char *)buf, info_data_len, 23);
		}
	}
	*/


	while (1)
	{

		while (HAL_UART_Receive(&huart3, uart_buf, UART_BUF_SIZE, 0xff) == HAL_OK) {
			if (dat_p != 0) {
				make_udp_reply_from_request(net_buf, (char *)uart_buf, UART_BUF_SIZE, 23);
			}
			HAL_Delay(400);
		}

		HAL_Delay(1);
		dat_p = packetloop_icmp_udp(net_buf, ES_enc28j60PacketReceive(NET_BUF_SIZE, net_buf));

		if (dat_p == 0)
			continue;

		HAL_UART_Transmit(&huart3, &net_buf[dat_p], info_data_len, 0xffffffff);
		HAL_Delay(400);


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi1);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
