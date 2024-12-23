/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "i2c_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
I2C_LCD_HandleTypeDef lcd1;
#define IR_PIN_1    GPIO_PIN_3
#define IR_PORT_1   GPIOA

#define ENABLE_PIN  GPIO_PIN_4
#define ENABLE_PORT GPIOA
#define DIR_PIN     GPIO_PIN_5
#define DIR_PORT    GPIOA
#define DIR_COUNTERCLOCKWISE      GPIO_PIN_RESET
#define MOTOR_ENABLED      GPIO_PIN_RESET
#define MOTOR_DISABLED      GPIO_PIN_SET

#define BUTTON GPIO_PIN_5
#define BUTTON_PORT GPIOB
// khai bao cho ir va mau sac
char data[50];
int IR_1;
int flag_red = 0;
int flag_green = 0;
int flag_blue = 0;
int flag_error = 0;
int flag =0;

int counting_red = 0;
int counting_green = 0;
int counting_blue = 0;
int flag_IR_1 = 0;
#define RGB_SAMPLES 10

uint32_t red_values[RGB_SAMPLES] = {0};
uint32_t green_values[RGB_SAMPLES] = {0};
uint32_t blue_values[RGB_SAMPLES] = {0};

//khai bao servo
uint32_t new_angle = 0;
uint32_t current_angle = 750;
uint16_t Pulse_up = 500;
//khai bao button
uint8_t Mode = 1;
volatile uint8_t motor_state = 1;
uint8_t toggle_state = 0; // Biến lưu trạng thái bật/tắt
uint8_t last_button_state = 1; // Biến lưu trạng thái nút trước đó (giả sử nút có Pull-up)
//uint32_t speed = 0;
//uint32_t currentspeed = 0;
//uint32_t min_speed = 350;
//uint32_t max_speed = 450;

// khai bao lcd
char buffer_red[50];
char buffer_green[50];
char buffer_blue[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TCS34725_ADDRESS          (0x29 << 1) /* I2C address */
/* Datasheet is at https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)
/* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
/* set the correct delay time in void getRawData() for TCS34725_INTEGRATIONTIME */
#define TCS34725_INTEGRATIONTIME_50MS   0xEB  /* 50ms  - 20 cycles */
#define TCS34725_GAIN_4X                0x01  /* 4x gain  */

uint8_t _tcs34725Initialised = 0;
int red, green, blue;

void write8 (uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
void enable(void);
void disable(void);
void setIntegrationTime(uint8_t it);
void setGain(uint8_t gain);
void tcs3272_init( void );
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(int *R, int *G, int *B);
void AverageFiltered(int* red, int* green, int* blue);
/* Writes a register and an 8 bit value over I2C */
void write8 (uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 2, 100);
}

/* Reads an 8 bit value over I2C */
uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

/* Reads a 16 bit values over I2C */
uint16_t read16(uint8_t reg)
{
  uint16_t ret;
    uint8_t txBuffer[1],rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
  return ret;
}

void enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  HAL_Delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  HAL_Delay(50);
}

void disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
  /* Make sure we're actually connected */
  uint8_t readValue = read8(TCS34725_ID);
  if ((readValue != 0x44) && (readValue != 0x10))
  {
    return;
  }
  _tcs34725Initialised = 1;
  /* Set default integration time and gain */
  setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  setGain(TCS34725_GAIN_4X);
  /* Note: by default, the device is in power down mode on bootup */
  enable();
}

/* Get raw data */
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (_tcs34725Initialised == 0) tcs3272_init();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  /* Delay time is from page no 16/26 from the datasheet  (256 − ATIME)* 2.4ms */
  HAL_Delay(50); // Set delay for (256 − 0xEB)* 2.4ms = 50ms
}

/* Get Red, Green and Blue color from Raw Data */
void getRGB(int *R, int *G, int *B)
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);
    if(rawClear == 0)
    {
      *R = 0;
      *G = 0;
      *B = 0;
    }
    else
    {
      *R = (int)rawRed * 255 / rawClear;
      *G = (int)rawGreen * 255 / rawClear;
      *B = (int)rawBlue * 255 / rawClear;
    }
}
void AverageFiltered(int* red, int* green, int* blue)
{
    int sum_red = 0, sum_green = 0, sum_blue = 0;
    for (int i = 0; i < RGB_SAMPLES; i++) {

        int current_red, current_green, current_blue;
        getRGB(&current_red, &current_green, &current_blue);

        sum_red += current_red;
        sum_green += current_green;
        sum_blue += current_blue;
    }

    *red = sum_red / RGB_SAMPLES;
    *green = sum_green / RGB_SAMPLES;
    *blue = sum_blue / RGB_SAMPLES;
}

void StepperMotor_Start(void)
{
    //	HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, MOTOR_ENABLED);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Pulse_up);
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, DIR_COUNTERCLOCKWISE);
}

void StepperMotor_Stop(void) {

    HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, MOTOR_DISABLED );
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
}

void checkColor(int red, int green, int blue)
{
//    if ((red > 160) )
//    {
//        flag_red = 1;
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//    }else if( green > 103 )
//    {
//    	flag_green = 1;
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//    }else if( blue > 85)
//    {
//    	flag_blue = 1;
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//    }
//

    if ((red > 190 && green < 40 && blue <= 35) )
      {
          flag_red = 1;
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
      }else if(red >= 90 && green > 105 && blue > 41)
      {
      	flag_green = 1;
      	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
      }else if(red < 90 && (green > 88 && green < 100)  && blue > 85)
      {
      	flag_blue = 1;
      	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
      }else{
      	flag_error = 1;
      }
//    	//flag_error = 1;
//    // detect yellow
//        else if ((red >= 145) && (red <= 160) && (green >= 60 && green <= 70) && (blue >= 22 && blue <= 36))
//        {
//        	flag_error = 1;
//        }
//            // detect orange
//        else if ((red >= 160) && (red < 190) && (green >= 40 && green <= 60) && (blue >= 22 && blue <= 36))
//        {
//        	flag_error = 1;
//        }
//        // detect pink
//        else if ((red >= 188) && (red <= 200) && ( green < 44) && (blue > 35 && blue <= 40))
//        {
//        	flag_error = 1;
//        }
//
//        else
//        {
//        	flag_error = 1;
//        }

}

void updateAngle(uint32_t new_angle)
{
    if (current_angle != new_angle)
    {
        if (current_angle < new_angle)
        {
            for (int y = current_angle; y <= new_angle; y += 3)
            {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, y);
                HAL_Delay(3);
            }
        }
        else if (current_angle > new_angle)
        {
            for (int y = current_angle; y >= new_angle; y -= 3)
            {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, y);
                HAL_Delay(3);
            }
        }
        current_angle = new_angle;
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_angle);
    }
}

void controlServo(int red, int green, int blue)
{
    if (flag_red == 1)
    {
        new_angle = 750;
        updateAngle(new_angle);
        flag_red = 0;
        counting_red++;
    }
    else if (flag_green == 1)
    {
        new_angle = 250;
        updateAngle(new_angle);
        flag_green = 0;
        counting_green++;
    }
    else if (flag_blue == 1)
    {
        new_angle = 1175;
        updateAngle(new_angle);
        flag_blue = 0;
        counting_blue++;
    }
    else if(flag_error == 1)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    	HAL_Delay(1000);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    	flag_error = 0;
    	StepperMotor_Stop();
    	motor_state = 0;

    }
}


void update_number(int red, int green, int blue)
{
 	  sprintf(buffer_red, "%u" , red);
 	  lcd_gotoxy(&lcd1, 0, 1);
 	  lcd_puts(&lcd1, buffer_red);
 	  sprintf(buffer_green, "%u" , green);
 	  lcd_gotoxy(&lcd1, 7, 1);
 	  lcd_puts(&lcd1, buffer_green);
 	  sprintf(buffer_blue, "%u" , blue);
 	  lcd_gotoxy(&lcd1, 13, 1);
 	  lcd_puts(&lcd1, buffer_blue);
}

void print_out(int red, int green, int blue)
{
 		lcd_gotoxy(&lcd1, 0, 0);
 		lcd_puts(&lcd1, "RED  GREEN  BLUE");
 		update_number(red, green, blue);
}

void checkIRAndControl()
{
    IR_1 = HAL_GPIO_ReadPin(IR_PORT_1, IR_PIN_1);

    if (IR_1 == 0)
    {
        flag_IR_1 = 1;
    }

    if (IR_1 == 1 && flag_IR_1 == 1)
    {
        controlServo(red, green, blue);
        char buffer_counting[100];
        int length_counting = snprintf(buffer_counting, sizeof(buffer_counting), "Red objects: %d, Green objects: %d, Blue objects: %d\r\n", counting_red, counting_green, counting_blue);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer_counting, length_counting, 1000);
        print_out(counting_red, counting_green, counting_blue);
        flag_IR_1 = 0;
    }
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  lcd1.hi2c = &hi2c2;
  lcd1.address = 0x4E;
  lcd_init(&lcd1);
  print_out(counting_red, counting_green, counting_blue);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1) ;
  StepperMotor_Start();
//  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (motor_state)
	  {
		  //StepperMotor_Stop();
		  StepperMotor_Start();
	  }
	  else if (motor_state == 0)
	  {
		  StepperMotor_Stop();
	  }
      AverageFiltered(&red, &green, &blue);
	  checkColor(red, green, blue);
	  checkIRAndControl();
	  char buffer[50];
	  int length = snprintf(buffer, sizeof(buffer), "Red: %d, Green: %d, Blue: %d\r\n", red, green, blue);
	  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length, 1000);
//	  AverageFiltered(&red, &green, &blue);
	  // Kiểm tra IR và control servo
//      IR_1 = HAL_GPIO_ReadPin(IR_PORT_1, IR_PIN_1);
//      if(red  > 140)
//      {
//      flag_red = 1;
//      }
//      else if(green > 105)
//      {
//      flag_green = 1;
//      }
//      else if(blue > 90)
//      {
//      flag_blue = 1;
//      }
//      if(IR_1 == 0 ){
//      flag_IR_1=1;
//      }
//
//      if(IR_1 == 1 && flag_IR_1 == 1 ){
//      if(flag_red == 1)
//      {
//    	  new_angle = 750;
//    	      	  if (current_angle != new_angle)
//    	      	  {
//    	      		  if (current_angle < new_angle)
//    	      	      {
//    	      			  for(int y = current_angle; y <= new_angle;  y+=3)
//    	      			  {
//    	      				  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, y);
//    	      			        HAL_Delay(3);
//    	      			  }
//    	      	      }
//
//    	      	      else if (current_angle > new_angle)
//    	      	      {
//    	      	    	  for(int y = current_angle; y >= new_angle;  y-=3)
//    	      	    	  {
//    	      	    		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, y);
//    	      	    		  HAL_Delay(3);
//    	      	    	  }
//
//    	      	      }
//    	      	   current_angle = new_angle;
//
//    	      	   }
//    	      	  else {
//    	      		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_angle);
//    	      	  }
//
//    	  flag_red = 0;
//    	  counting_red++;
//      }
//      else if(flag_green == 1)
//      {
//    	  new_angle = 250;
//    	      	  if (current_angle != new_angle)
//    	      	  {
//    	      		  if (current_angle < new_angle)
//    	      	      {
//    	      			  for(int y = current_angle; y <= new_angle;  y+=3)
//    	      			  {
//    	      				  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, y);
//    	      			        HAL_Delay(3);
//    	      			  }
//    	      	      }
//
//    	      	      else if (current_angle > new_angle)
//    	      	      {
//    	      	    	  for(int y = current_angle; y >= new_angle;  y-=3)
//    	      	    	  {
//    	      	    		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, y);
//    	      	    		  HAL_Delay(3);
//    	      	    	  }
//
//    	      	      }
//    	      	   current_angle = new_angle;
//
//    	      	   }
//    	      	  else {
//    	      		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_angle);
//    	      	  }
//
//          flag_green = 0;
//          counting_green ++;
//      }
//
//      else if(flag_blue == 1 )
//      {
//    	  new_angle = 1175;
//    	      	      	  if (current_angle != new_angle)
//    	      	      	  {
//    	      	      		  if (current_angle < new_angle)
//    	      	      	      {
//    	      	      			  for(int y = current_angle; y <= new_angle; y+=3)
//    	      	      			  {
//    	      	      				  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, y);
//    	      	      			        HAL_Delay(3);
//    	      	      			  }
//    	      	      	      }
//
//    	      	      	      else if (current_angle > new_angle)
//    	      	      	      {
//    	      	      	    	  for(int y = current_angle; y >= new_angle;  y-=3)
//    	      	      	    	  {
//    	      	      	    		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, y);
//    	      	      	    		  HAL_Delay(3);
//    	      	      	    	  }
//
//    	      	      	      }
//    	      	      	   current_angle = new_angle;
//
//    	      	      	   }
//    	      	      	  else {
//    	      	      		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, new_angle);
//    	      	      	  }
//    	  flag_blue = 0;
//    	  counting_blue ++;
//
//      }
//      char buffer_counting[100];
//      int length_counting = snprintf(buffer_counting, sizeof(buffer_counting), "Red objects: %d, Green objects: %d, Blue objects: %d\r\n", counting_red, counting_green, counting_blue);
//      HAL_UART_Transmit(&huart1, (uint8_t*)buffer_counting, length_counting, 1000);
//      flag_IR_1 = 0;
//      }
//
//



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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 144-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 800-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == BUTTON) // Kiểm tra ngắt từ nút nhấn
    {
    	motor_state = 1;
        //motor_state = !motor_state; // muôn bat tat thi dung doan nay
       // StepperMotor_Start();
//        if (motor_state == 0)
//        {
//        	//StepperMotor_Start();
//        	StepperMotor_Stop();
//
//        }
    }
}
//long map(long x, long in_min, long in_max, long out_min, long out_max)
//{
//  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
//}
//uint32_t readFilteredADC(void) {
//   uint32_t sum = 0;
// for (int i = 0; i < 500; i++) {
//     HAL_ADC_Start(&hadc1);
//      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//       sum += HAL_ADC_GetValue(&hadc1);
//   }
//  return sum / 500;
//}
//
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	if(hadc->Instance == ADC1)
//    {
//
//        uint32_t adc_value = readFilteredADC();
//        uint32_t speed = map(adc_value, 0, 4095, min_speed, max_speed);
//
//        while(speed > currentspeed  - 5 && speed < currentspeed  + 5) // Khử nhiễu
//        {
//         	adc_value = readFilteredADC();
//         	speed = map(adc_value, 0, 4095, min_speed, max_speed);
//        }
//    	if (currentspeed  != speed) // kiểm tra xem biến trở có thay đổi giá trị không
//    	{
//    		uint32_t arr_value = (1000000 * 60) / (speed * 200) - 1;
//    		 __HAL_TIM_SET_AUTORELOAD(&htim3, arr_value);
//    		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_value/2);
//
//    		 currentspeed = speed;
//
//     }
//
//	}
//}

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
