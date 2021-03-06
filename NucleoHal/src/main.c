/**
 ******************************************************************************
 * @file    main.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    17-May-2015
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "main.h"
#include "wifi_interface.h"
#include "stdio.h"
#include "string.h"

int inputAvailable();

/**
 * @mainpage Documentation for X-CUBE-WIFI Software for STM32, Expansion for STM32Cube
 * <b>Introduction</b> <br>
 * X-CUBE-WIFI1 is an expansion software package for STM32Cube.
 * The software runs on STM32 and it can be used for building Wi-Fi applications using the SPWF01Sx device.
 * It is built on top of STM32Cube software technology that eases portability across different STM32 microcontrollers.
 *
 *
 * \htmlinclude extra.html
 */

/** @defgroup WIFI_Examples
 * @{
 */

/** @defgroup WIFI_Example_Client_Socket
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define WIFI_SCAN_BUFFER_LIST           15

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;
static GPIO_InitTypeDef GPIO_InitStruct;
extern UART_HandleTypeDef UartMsgHandle;
extern char print_msg_buff[512];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
WiFi_Status_t wifi_get_AP_settings(void);
void MX_I2S2_Init(void);
void MX_DMA_Init(void);
void ProcessAudio();

/* Private functions ---------------------------------------------------------*/
#ifdef USART_PRINT_MSG

#define print_uart(arg)    { memset(print_msg_buff, 0x00, sizeof(print_msg_buff)); \
                         sprintf((char*)print_msg_buff,arg);   \
                         HAL_UART_Transmit(&UartMsgHandle, (uint8_t*)print_msg_buff, strlen(print_msg_buff), 1000); }

//#define print_uart(arg)
#define print_uart_user(arg)    { memset(print_msg_buff, 0x00, sizeof(print_msg_buff)); \
                         sprintf((char*)print_msg_buff,arg);   \
                         HAL_UART_Transmit(&UartMsgHandle, (uint8_t*)print_msg_buff, strlen(print_msg_buff), 1000); }
#endif                       

typedef enum {
	wifi_state_reset = 0,
	wifi_state_ready,
	wifi_state_idle,
	wifi_state_connected,
	wifi_state_connecting,
	wifi_state_disconnected,
	wifi_state_socket,
	wifi_state_error,
	wifi_undefine_state = 0xFF,
} wifi_state_t;

wifi_state_t wifi_state;
wifi_config config;
wifi_scan net_scan[WIFI_SCAN_BUFFER_LIST];

uint8_t console_input[1], console_count = 0;
char console_ssid[40];
char console_psk[20];
char console_host[20];
wifi_bool set_AP_config = WIFI_FALSE, SSID_found = WIFI_FALSE;

char * ssid = "TP-LINK_POCKET_3040_801526"; // "NETGEAR28";  // "NETGEAR28"; // "AndroidHotspot4629"; // "NETGEAR54";
char * seckey = ""; //"vanillariver618";
WiFi_Priv_Mode mode = None; // WPA_Personal;

uint16_t len; /*Take care to change the length of the text we are sending*/

char *data = "Hello World!";
char *hostname = "192.168.0.101"; // "10.0.0.2";
char *protocol = "t"; //t -> tcp , s-> secure tcp
uint32_t portnumber = 32000;
uint8_t socket_id;
uint8_t socket_open = 0;

/**
 *
 */
void init_blink() {
	/* -1- Enable GPIO Clock (to be able to program the configuration registers) */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/* -2- Configure IO in output push-pull mode to drive external LEDs */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* -3- Toggle IO in an infinite loop
	 while (1)
	 {
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	 Insert delay 100 ms
	 HAL_Delay(100);
	 }
	 */
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
// static long myTick = 0;
// static int32_t value;
uint16_t Data1[1000];
uint16_t Data2[1000];
int32_t Audio[250];
uint16_t Size = 40;
uint8_t Ready=0;
uint16_t *produce_ptr, *consume_ptr;
uint8_t which;
int main(void) {
	uint8_t i = 0;
	uint32_t tickblink = 0;
	uint32_t tickwifi = 0;

	tickblink = HAL_GetTick();
	tickwifi = tickblink;
	len = strlen(data);

	WiFi_Status_t status = WiFi_MODULE_SUCCESS;
	__GPIOA_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;
	__GPIOC_CLK_ENABLE()
	;

	HAL_Init();

	/* Configure the system clock to 64 MHz */
	SystemClock_Config();

	/* configure the timers  */
	Timer_Config();

	UART_Configuration(115200);
#ifdef USART_PRINT_MSG
	UART_Msg_Gpio_Init();
	USART_PRINT_MSG_Configuration(115200);
#endif

	init_blink();
	MX_DMA_Init();
	MX_I2S2_Init();

	/*
	config.power = sleep;
	config.power_level = high;
	config.dhcp = on; //use DHCP IP address
	config.web_server = WIFI_TRUE;

	wifi_state = wifi_state_idle;

	status = wifi_get_AP_settings();
	if (status != WiFi_MODULE_SUCCESS) {
		print_uart("\r\nError in AP Settings");
		return 0;
	}

	wifi_state = wifi_state_reset;
	*/

	produce_ptr=Data1;
	which=1;
	consume_ptr=NULL;
	HAL_I2S_Receive_DMA(&hi2s2, produce_ptr, Size/2);

	while (1) {
    /*
		switch (wifi_state) {
		case wifi_state_reset:
			print_uart("\r\n\nInitializing the wifi module...")
			;

			// Init the wi-fi module
			status = wifi_init(&config);
			if (status != WiFi_MODULE_SUCCESS) {
				print_uart("Error in Config");
				return 0;
			}

			print_uart("\r\n\nConfig OK \r\n\n")
			;
			wifi_state = wifi_state_ready;
			socket_open = 0;
			break;

		case wifi_state_ready:

			print_uart("\r\n >>running WiFi Scan...\r\n")
			;

			status = wifi_network_scan(net_scan, WIFI_SCAN_BUFFER_LIST);

			if (status == WiFi_MODULE_SUCCESS) {
				for (i = 0; i < WIFI_SCAN_BUFFER_LIST; i++) {
					print_uart(net_scan[i].ssid);
					print_uart("\r\n");
					if (((char *) strstr((const char *) net_scan[i].ssid,
							(const char *) console_ssid)) != NULL) {
						print_uart(
								"\r\n >>network present...connecting to AP...\r\n");
						SSID_found = WIFI_TRUE;
						wifi_connect((uint8_t *) console_ssid,
								(uint8_t *) console_psk, mode);

						break;
					}
				}
				if (!SSID_found) {
					print_uart("\r\nGiven SSID not found!\r\n");
					NVIC_SystemReset();
				}
				memset(net_scan, 0x00, sizeof(net_scan));

			}

			wifi_state = wifi_state_idle;

			break;

		case wifi_state_connected:

			HAL_Delay(2000);                //Let module go to sleep

			print_uart("\r\n >>connected...\r\n")
			;

			wifi_wakeup(WIFI_TRUE); // wakeup from sleep if module went to sleep

			wifi_state = wifi_state_idle;
			break;

		case wifi_state_disconnected:
			wifi_state = wifi_state_reset;
			break;

		case wifi_state_socket:
			print_uart("\r\n >>Connecting to socket\r\n")
			;
			if (socket_open == 0) {
				// Read Write Socket data
				WiFi_Status_t status = WiFi_MODULE_SUCCESS;

				status = wifi_socket_client_open((uint8_t *) console_host,
						portnumber, (uint8_t *) protocol, &socket_id);
				if (status == WiFi_MODULE_SUCCESS) {
					print_uart("\r\n >>Socket Open OK \r\n");
					socket_open = 1;
					status = wifi_socket_client_write(socket_id, len, data);
					if (status == WiFi_MODULE_SUCCESS) {
						print_uart("\r\n >>Socket Write OK\r\n");
						produce_ptr=Data1;
						which=1;
						consume_ptr=NULL;

						//__HAL_I2S_ENABLE(&hi2s2);
						//while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0);
						//while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));

						HAL_I2S_Receive_DMA(&hi2s2, produce_ptr, Size/2);
					}
				} else {
					print_uart("Socket connection Error");
					HAL_Delay(1000);
					break; // to start again
				}
			} else {
				print_uart("Socket not opened!");
				NVIC_SystemReset();
			}
			wifi_state = wifi_state_idle;
			break;

		case wifi_state_idle:
			if (HAL_GetTick() - tickwifi > 500) {
				print_uart(".");
				tickwifi = HAL_GetTick();
			}
			break;

		default:
			break;
		}
		*/

		if(Ready==1){
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			int i;
			char buf[100];
			ProcessAudio();
			for(i=0; i<((Size/4)-1); i++){
				// sprintf(buf,"$%d:%d; ",(int)Audio[i],(int)Audio[i+1]);
				sprintf(buf,"$%ld;",Audio[i]);
				print_uart_user(buf);
			}
			print_uart_user("\r\n\r\n");
		}

		// toggle led 2
         /*
		 if (HAL_GetTick() - tickblink > 0) {
		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		 tickblink = HAL_GetTick();
		 }
		 */

/*

		uint32_t Timeout = 10000;
		if ((wifi_state == wifi_state_idle) && (socket_open == 1)) {
			// wait WS toggle
			while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0)
				;
			// print_uart_user("1\r\n");
			while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
				;
			// print_uart_user("0\r\n");
			if (HAL_I2S_Receive(&hi2s2, Data, Size, Timeout) == HAL_OK) {
				myTick++;
				char str[1000];
				char temp[20];
				str[0] = '\0';

				int low, high;

				int exit = 0;
				len=0;

				for (i = 0; i < Size; i += 2) {
					if (exit == -1)
						break;
					if ((i % 4) == 0) {
						low = (int) (Data[i + 1] >> 9);
						high = ((int) ((Data[i] & 0b0011111111111111))) << 9;
						int sign = Data[i] & 0b0100000000000000;
						value = low + high;
						if (sign != 0) {
							value -= (int) (1 << 23);
						}
						sprintf(temp,"$%d;",(int)value);
						// sprintf(str, "%d,%d,%d\r\n", (unsigned int) high, (unsigned int) low, (unsigned int) sign );
						sprintf(&str[len], "%s",  temp);
						len += strlen(temp);
						//print_uart_user(str);
					}
				}
				if (socket_open == 1) {
					status = wifi_socket_client_write(socket_id, len, str);
					if (status != WiFi_MODULE_SUCCESS) {
						print_uart("\r\n >>Socket Write error\r\n");
						NVIC_SystemReset();
					}
				}
			}

		}
*/
	}

}

void ProcessAudio(){
	int i;
	for(i=0; i< (Size-1);i+=2){
		if ((i % 4)==2) {
			uint32_t high = (((uint32_t) (consume_ptr[i] & 0x3fff)) << 9);
			high &= 0x00ffffff;
			uint32_t low = ((consume_ptr[i + 1] & 0xff80) >> 7);
			low &= 0x00ffffff;
			int32_t value = (high + low);
			int sign = consume_ptr[i] & 0x4000;
			if (sign != 0) {
				value = value - (1 << 23);
			}
			Audio[i / 4] = value;
		}
	}
	Ready=0;
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	//__HAL_I2S_DISABLE(hi2s);
	// HAL_I2S_DMAStop(hi2s);
	if (which==1){
		produce_ptr=Data2;
		consume_ptr=Data1;
		which=2;
	} else {
		produce_ptr=Data1;
		consume_ptr=Data2;
		which=1;
	}
	HAL_I2S_Receive_DMA(&hi2s2, produce_ptr, Size/2);
	Ready=1;
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
	MX_I2S2_Init();
	HAL_I2S_RxCpltCallback(hi2s);
}


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSI)
 *            SYSCLK(Hz)                     = 64000000
 *            HCLK(Hz)                       = 64000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            PLLMUL                         = 16
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */

#ifdef USE_STM32F4XX_NUCLEO

void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
// gbr
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
// gbr

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE()
	;

	/* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/* Enable HSI Oscillator and activate PLL with HSI as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 0x10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

//gbr
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
	PeriphClkInitStruct.PLLI2SDivQ = 1;
	PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
// gbr
}
#endif

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %d\r\n", file, (int) line);
	/* Infinite loop */
	while (1) {
	}
}
#endif

/**
 * @brief  Query the User for SSID, password, encryption mode and hostname
 * @param  None
 * @retval WiFi_Status_t
 */
WiFi_Status_t wifi_get_AP_settings(void) {
	int countdown = 10;
	WiFi_Status_t status = WiFi_MODULE_SUCCESS;
	printf("\r\n\n/********************************************************\n");
	printf("\r *                                                      *\n");
	printf("\r * X-CUBE-WIFI1 Expansion Software v1.1.0               *\n");
	printf("\r * X-NUCLEO-IDW01M1 Wi-Fi Configuration.                *\n");
	printf("\r * Client-Socket Example                                *\n");
	printf("\r *                                                      *\n");
	printf("\r *******************************************************/\n");
	printf("\r\nDo you want to setup SSID?(y/n):");
	fflush(stdout);
	while (!inputAvailable()) { // give 10 sec for the operator to chose WIFI tuning
		countdown--;
		if (countdown == 0)
			break;
		HAL_Delay(1000);  // wait 1 sec
	}
	if (countdown != 0) {
		scanf("%s", console_input);
	} else {
		console_input[0] = 'n';
	}
	printf("\r\n");
//HAL_UART_Receive(&UartMsgHandle, (uint8_t *)console_input, 1, 100000);
	if (console_input[0] == 'y') {
		set_AP_config = WIFI_TRUE;
		printf("Enter the SSID:");
		fflush(stdout);

		console_count = 0;
		console_count = scanf("%s", console_ssid);
		printf("\r\n");

		if (console_count == 39) {
			printf("Exceeded number of ssid characters permitted");
			return WiFi_NOT_SUPPORTED;
		}

		//printf("entered =%s\r\n",console_ssid);
		printf("Enter the password:");
		fflush(stdout);
		console_count = 0;

		console_count = scanf("%s", console_psk);
		printf("\r\n");
		//printf("entered =%s\r\n",console_psk);
		if (console_count == 19) {
			printf("Exceeded number of psk characters permitted");
			return WiFi_NOT_SUPPORTED;
		}
		printf(
				"Enter the encryption mode(0:Open, 1:WEP, 2:WPA2/WPA2-Personal):");
		fflush(stdout);
		scanf("%s", console_input);
		printf("\r\n");
		//printf("entered =%s\r\n",console_input);
		switch (console_input[0]) {
		case '0':
			mode = None;
			break;
		case '1':
			mode = WEP;
			break;
		case '2':
			mode = WPA_Personal;
			break;
		default:
			printf("\r\nWrong Entry. Priv Mode is not compatible\n");
			return WiFi_NOT_SUPPORTED;
		}

		printf("Enter the Hostname (Apache Server): ");
		fflush(stdout);
		console_count = 0;

		console_count = scanf("%s", console_host);
		printf("\r\n");
		//printf("entered =%s\r\n",console_host);
		if (console_count == 19) {
			printf("Exceeded number of host characters permitted");
			return WiFi_NOT_SUPPORTED;
		}

	} else {
		printf("\r\n\nModule will connect with default settings.");
		memcpy(console_ssid, (const char*) ssid, strlen((char*) ssid));
		memcpy(console_psk, (const char*) seckey, strlen((char*) seckey));
		memcpy(console_host, (const char*) hostname, strlen((char*) hostname));
	}

	printf(
			"\r\n\n/**************************************************************\n");
	printf(
			"\r * Configuration Complete                                     *\n");
	printf(
			"\r * Port Number:32000, Protocol: TCP/IP                        *\n");
	printf(
			"\r * Please make sure a server is listening at given hostname   *\n");
	printf(
			"\r *************************************************************/\n");

	return status;
}

/******** Wi-Fi Indication User Callback *********/

void ind_wifi_socket_data_received(uint8_t * data_ptr, uint32_t message_size,
		uint32_t chunk_size) {
	printf("\r\nData Receive Callback...\r\n");
	printf((const char*) data_ptr);
}

void ind_wifi_socket_client_remote_server_closed(uint8_t * socket_closed_id) {
	uint8_t id = *socket_closed_id;
	printf("\r\n>>User Callback>>remote server socket closed\r\n");
	printf((const char*) &id); //this will actually print the character/string, not the number
	wifi_state = wifi_state_reset;
}

void ind_wifi_on() {
	printf("\r\nWiFi Initialised and Ready..\r\n");
	wifi_state = wifi_state_ready;
}

void ind_wifi_connected() {
	wifi_state = wifi_state_connected;
}

void ind_wifi_resuming() {
	printf("\r\nwifi resuming from sleep user callback... \r\n");
//Change the state to connect to socket if not connected
	wifi_state = wifi_state_socket;
}

/* I2S2 init function */
void MX_I2S2_Init(void) {

	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s2.Init.Standard = I2S_STANDARD_MSB; // I2S_STANDARD_PCM_LONG;// I2S_STANDARD_MSB; //I2S_STANDARD_PHILIPS; //I2S_STANDARD_LSB; //I2S_STANDARD_MSB;// I2S_STANDARD_PCM_LONG;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	HAL_I2S_Init(&hi2s2);

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
