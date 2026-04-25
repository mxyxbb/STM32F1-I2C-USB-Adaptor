/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "tusb.h" // 引入 TinyUSB 头文件

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEVICE_SIGNATURE "STM32F103_I2C_TUSB_V1"

#define HEADER_BYTE 0xAA
#define TAIL_BYTE 0x55  
#define CMD_GET_SIG 0x01
#define CMD_SET_BAUD 0x02
#define CMD_I2C_WRITE 0x03
#define CMD_I2C_READ 0x04
#define CMD_I2C_RESET 0x05
#define CMD_I2C_WRITEREAD 0x06

#define I2C_TIMEOUT_MS 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t current_i2c_baudrate = 100000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 为 TinyUSB 提供毫秒级系统时间
uint32_t tusb_time_millis_api(void) {
    return HAL_GetTick();
}

// 兼容旧版 TinyUSB 的宏
uint32_t board_millis(void) {
    return HAL_GetTick();
}

// === 发送响应 (TinyUSB 版本，不再需要防死锁，TinyUSB 自己会排队) ===
void send_response(uint8_t cmd, uint8_t *data, uint16_t len) {
    static uint8_t tx_buffer[1030]; 
    
    tx_buffer[0] = HEADER_BYTE;
    tx_buffer[1] = cmd;
    tx_buffer[2] = len & 0xFF;
    tx_buffer[3] = (len >> 8) & 0xFF;
    
    uint8_t checksum = cmd + tx_buffer[2] + tx_buffer[3];
    for (int i = 0; i < len; i++) {
        tx_buffer[4 + i] = data[i];
        checksum += data[i];
    }
    tx_buffer[4 + len] = checksum;
    tx_buffer[5 + len] = TAIL_BYTE;
    
    uint16_t total_len = 6 + len;
    
    // 如果 USB 已连接，直接交给 TinyUSB 发送
    if (tud_cdc_connected()) {
        tud_cdc_write(tx_buffer, total_len);
        tud_cdc_write_flush(); // 强制推入底层发送
    }
}

// === I2C 总线死锁恢复 (硬件级别，依然需要) ===
void I2C_Recover_Bus(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1); 
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); 
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   
    HAL_Delay(1);
}

// === 重新初始化 I2C ===
void Reinit_I2C1(uint32_t baudrate) {
    HAL_I2C_DeInit(&hi2c1);
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = baudrate;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

// === 协议解析函数 (完全不变) ===
void process_frame(uint8_t cmd, uint8_t *data, uint16_t len) {
    static uint8_t reply_data[1024]; 
    HAL_StatusTypeDef status;
    uint16_t dev_addr_shifted;
    
    switch (cmd) {
        case CMD_GET_SIG:
            send_response(cmd | 0x80, (uint8_t*)DEVICE_SIGNATURE, strlen(DEVICE_SIGNATURE));
            break;

        case CMD_SET_BAUD:
            if (len == 4) {
                current_i2c_baudrate = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                Reinit_I2C1(current_i2c_baudrate);
                reply_data[0] = 0x00; 
                send_response(cmd | 0x80, reply_data, 1);
            }
            break;

        case CMD_I2C_RESET:
            I2C_Recover_Bus(); 
            Reinit_I2C1(current_i2c_baudrate); 
            reply_data[0] = 0x00; 
            send_response(cmd | 0x80, reply_data, 1);
            break;

        case CMD_I2C_WRITE:
            if (len >= 1) {
                dev_addr_shifted = (uint16_t)(data[0] << 1); 
                status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr_shifted, &data[1], len - 1, I2C_TIMEOUT_MS);
                reply_data[0] = (status == HAL_OK) ? 0x00 : 0xFF; 
                send_response(cmd | 0x80, reply_data, 1);
            }
            break;

        case CMD_I2C_READ:
            if (len == 3) {
                dev_addr_shifted = (uint16_t)(data[0] << 1);
                uint16_t read_len = data[1] | (data[2] << 8);
                if (read_len <= 512) { 
                    status = HAL_I2C_Master_Receive(&hi2c1, dev_addr_shifted, reply_data, read_len, I2C_TIMEOUT_MS);
                    if (status == HAL_OK) send_response(cmd | 0x80, reply_data, read_len);
                    else send_response(cmd | 0x80, NULL, 0); 
                }
            }
            break;

        case CMD_I2C_WRITEREAD:
            if (len >= 3) {
                dev_addr_shifted = (uint16_t)(data[0] << 1);
                uint16_t read_len = data[1] | (data[2] << 8);
                uint16_t write_len = len - 3; 

                if (write_len == 1 || write_len == 2) {
                    uint16_t mem_addr = (write_len == 1) ? data[3] : (data[3] << 8 | data[4]);
                    uint16_t mem_addr_size = (write_len == 1) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
                    
                    status = HAL_I2C_Mem_Read(&hi2c1, dev_addr_shifted, mem_addr, mem_addr_size, reply_data, read_len, I2C_TIMEOUT_MS);
                    if (status == HAL_OK) send_response(cmd | 0x80, reply_data, read_len);
                    else send_response(cmd | 0x80, NULL, 0);
                } 
                else {
                    int success = 1;
                    if (write_len > 0) {
                        status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr_shifted, &data[3], write_len, I2C_TIMEOUT_MS);
                        if (status != HAL_OK) success = 0;
                    }
                    if (success && read_len > 0) {
                        status = HAL_I2C_Master_Receive(&hi2c1, dev_addr_shifted, reply_data, read_len, I2C_TIMEOUT_MS);
                        if (status == HAL_OK) send_response(cmd | 0x80, reply_data, read_len);
                        else success = 0;
                    } else if (success && read_len == 0) {
                        reply_data[0] = 0x00;
                        send_response(cmd | 0x80, reply_data, 1);
                    }
                    if (!success) send_response(cmd | 0x80, NULL, 0);
                }
            }
            break;
    }
}

void Force_USB_Re_enumeration(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_12; // USB D+
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 强行拉低 100ms，告诉电脑 "我拔线了"
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(100); 

    // 恢复原样，准备让真正的 USB 外设接管
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    HAL_Delay(10); 
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	Force_USB_Re_enumeration();
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
 // MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
	// 1. 强制给 USB 外设总线通电 (极其重要！)
  __HAL_RCC_USB_CLK_ENABLE();

  // 2. 强制在 NVIC 嵌套向量中断控制器中开启 USB 中断
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	
	tusb_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	int state = 0;
    uint8_t rx_cmd = 0, rx_calc_checksum = 0;
    uint8_t len_lsb = 0;
    uint16_t rx_len = 0;
    uint8_t rx_buf[1024]; 
    int rx_idx = 0;

    // LED 及超时控制
    uint32_t led_on_tick = 0;
    uint8_t led_is_on = 0;
    uint32_t last_rx_tick = HAL_GetTick();
		
  while (1)
  {
		
		// 1. 喂养 TinyUSB 任务调度器 (必须放在主循环最高频执行)
        tud_task();

        // 2. 非阻塞处理 PC13 LED 熄灭
        if (led_is_on && (HAL_GetTick() - led_on_tick >= 30)) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            led_is_on = 0;
        }

        // 3. 超时重置逻辑 (如果接收一半卡住，100ms 后复位状态机)
        if (state != 0 && (HAL_GetTick() - last_rx_tick > 100)) {
            state = 0;
        }

        // 4. 从 TinyUSB 读取数据 (极其优雅，自带流控和 FIFO)
        if (tud_cdc_available()) {
            uint8_t byte = tud_cdc_read_char();
            last_rx_tick = HAL_GetTick(); // 更新最后一次收到数据的时间

            switch (state) {
                case 0: 
                    if (byte == HEADER_BYTE) state = 1;
                    break;
                case 1: 
                    rx_cmd = byte;
                    rx_calc_checksum = byte;
                    state = 2;
                    break;
                case 2: 
                    len_lsb = byte;
                    rx_calc_checksum += byte;
                    state = 3;
                    break;
                case 3: 
                    rx_len = len_lsb | (byte << 8);
                    rx_calc_checksum += byte;
                    rx_idx = 0;
                    if (rx_len > sizeof(rx_buf)) state = 0; 
                    else state = (rx_len > 0) ? 4 : 5;
                    break;
                case 4: 
                    rx_buf[rx_idx++] = byte;
                    rx_calc_checksum += byte;
                    if (rx_idx >= rx_len) state = 5;
                    break;
                case 5: 
                    if (byte == rx_calc_checksum) state = 6; 
                    else state = 0; 
                    break;
                case 6: 
                    if (byte == TAIL_BYTE) {
                        // 点亮 LED
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                        led_on_tick = HAL_GetTick();
                        led_is_on = 1;

                        // 执行 I2C 动作
                        process_frame(rx_cmd, rx_buf, rx_len);
                    }
                    state = 0; 
                    break;
            }
        }
				
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
