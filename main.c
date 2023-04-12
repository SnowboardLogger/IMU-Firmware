/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "stdbool.h"
#include "Math.h"


#define I2C_Addr 0x50
/* PAGE ZERO
 * At power-on Page 0 is selected, PAGE_ID register
 * can be used to identify the current selected page and
 * change between page 0 and page 1
 */
#define MAG_RADIUS_MSB 0x6A
#define MAG_RADIUS_LSB 0x69
#define ACC_RADIUS_MSB 0x68
#define ACC_RADIUS_LSB 0x67
#define GYR_OFFSET_Z_MSB 0x66
#define GYR_OFFSET_Z_LSB 0x65
#define GYR_OFFSET_Y_MSB 0x64
#define GYR_OFFSET_Y_LSB 0x63
#define GYR_OFFSET_X_MSB 0x62
#define GYR_OFFSET_X_LSB 0x61
#define MAG_OFFSET_Z_MSB 0x60
#define MAG_OFFSET_Z_LSB 0x5F
#define MAG_OFFSET_Y_MSB 0x5E
#define MAG_OFFSET_Y_LSB 0x5D
#define MAG_OFFSET_X_MSB 0x5C
#define MAG_OFFSET_X_LSB 0x5B
#define ACC_OFFSET_Z_MSB 0x5A
#define ACC_OFFSET_Z_LSB 0x59
#define ACC_OFFSET_Y_MSB 0x58
#define ACC_OFFSET_Y_LSB 0x57
#define ACC_OFFSET_X_MSB 0x56
#define ACC_OFFSET_X_LSB 0x55

#define AXIS_MAP_SIGN 0x42
#define AXIS_MAP_CONFIG 0x41
#define TEMP_SOURCE 0x40
#define SYS_TRIGGER 0x3F
#define PWR_MODE 0x3E
#define OPR_MODE 0x3D
#define UNIT_SEL 0x3B
#define SYS_ERR 0x3A
#define SYS_STATUS 0x39
#define SYS_CLK_STATUS 0x38
#define INT_STA 0x37
#define ST_RESULT 0x36
#define CALIB_STAT 0x35
#define TEMP 0x34
// Gravity Vector
#define GRV_DATA_Z_MSB 0x33
#define GRV_DATA_Z_LSB 0x32
#define GRV_DATA_Y_MSB 0x31
#define GRV_DATA_Y_LSB 0x30
#define GRV_DATA_X_MSB 0x2F
#define GRV_DATA_X_LSB 0x2E
// Linear Acceleration
#define LIA_DATA_Z_MSB 0x2D
#define LIA_DATA_Z_LSB 0x2C
#define LIA_DATA_Y_MSB 0x2B
#define LIA_DATA_Y_LSB 0x2A
#define LIA_DATA_X_MSB 0x29
#define LIA_DATA_X_LSB 0x28
// Quaternion
#define QUA_DATA_Z_MSB 0x27
#define QUA_DATA_Z_LSB 0x26
#define QUA_DATA_Y_MSB 0x25
#define QUA_DATA_Y_LSB 0x24
#define QUA_DATA_X_MSB 0x23
#define QUA_DATA_X_LSB 0x22
#define QUA_DATA_W_MSB 0x21
#define QUA_DATA_W_LSB 0x20
// Pitch
#define EUL_PITCH_MSB 0x1F
#define EUL_PITCH_LSB 0x1E
// Roll
#define EUL_ROLL_MSB 0x1D
#define EUL_ROLL_LSB 0x1C
// Heading
#define EUL_HEADING_MSB 0x1B
#define EUL_HEADING_LSB 0x1A
// Sensor Data
#define GYR_DATA_Z_MSB 0x19
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_X_LSB 0x14

#define MAG_DATA_Z_MSB 0x13
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_X_MSB 0xF
#define MAG_DATA_X_LSB 0xE

#define ACC_DATA_Z_MSB 0xD
#define ACC_DATA_Z_LSB 0xC
#define ACC_DATA_Y_MSB 0xB
#define ACC_DATA_Y_LSB 0xA
#define ACC_DATA_X_MSB 0x9
#define ACC_DATA_X_LSB 0x8

// Extra
#define PAGE_ID 0x7
#define BL_REV_ID 0x6
#define SW_REV_ID_MSB 0x5
#define SW_REV_ID_LSB 0x4
#define GYR_ID 0x3
#define MAG_ID 0x2
#define ACC_ID 0x1
#define CHIP_ID 0x0


/*
 * Page One. Use the PAGE_ID register to
 * switch and see values between the two pages
*/
#define GYR_AM_SET 0x1F
#define GYR_AM_THRES 0x1E
#define GYR_DUR_Z 0x1D
#define GYR_HR_Z_SET 0x1C
#define GYR_DUR_Y 0x1B
#define GYR_HR_Y_SET 0x1A
#define GYR_DUR_X 0x19
#define GYR_HR_X_SET 0x18
#define GYR_INT_SETING 0x17
#define ACC_NM_SET 0x16
#define ACC_NM_THRE 0x15
#define ACC_HG_THRES 0x14
#define ACC_HG_DURATION 0x13
#define ACC_INT_SETTINGS 0x12
#define ACC_AM_THRES 0x11
#define INT_EN 0x10
#define INT_MSK 0xF
#define GYR_SLEEP_CONFIG 0xD
#define ACC_SLEEP_CONFIG 0xC
#define GYR_CONFIG_1 0xB
#define GYR_CONFIG_0 0xA
#define MAG_CONFIG 0x9
#define ACC_CONFIG 0x8
#define PAGE_ID 0x7

#define BNO055_SAMPLERATE_DELAY_MS (100)

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef ret;
  uint8_t buf[20];

  // SETUP for IMU
  	/* Set Config Mode */
  	buf[0] = OPR_MODE;
  	buf[1] = 0x00;
  	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);
  	HAL_Delay(30);

  	/* Reset */
	buf[0] = SYS_TRIGGER;
	buf[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(30);

	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	while (buf[0] != 0xA0) {
		HAL_Delay(10);
		HAL_I2C_Master_Receive(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	}

	HAL_Delay(50);

	/* Normal Power Mode */
	buf[0] = PWR_MODE;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(10);

	/* Set Page Number to 0 */
	buf[0] = PAGE_ID;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	/* Reset 8 */
	buf[0] = SYS_TRIGGER;
	buf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(10);

	/* Set Operation Mode */
	buf[0] = OPR_MODE;
	buf[1] = 0x0C;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);
	HAL_Delay(30);

	volatile int16_t xGyro = 0, yGyro = 0, zGyro = 0;
	volatile int16_t xAcc = 0, yAcc = 0, zAcc = 0;
	volatile int16_t xMag = 0, yMag = 0, zMag = 0;
	volatile int32_t time = 0;


	uint16_t MEuler[3] = {0, 0, 0};
	float CEuler[3] = {0, 0, 0};

	uint16_t MQuat[4] = {0, 0, 0, 0};
	float CQuat[4] = {0, 0, 0, 0};

	float acceleration[3] = {0, 0, 0};
	float magnometer[3] = {0, 0, 0};
	float gyroscope[3] = {0, 0, 0};

	/* Temperature reading */
	buf[0] = TEMP;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	int8_t temp = buf[0];

	// Calibration for the sensor
	buf[0] = ACC_OFFSET_X_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_X_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Y_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Y_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Z_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_OFFSET_Z_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_X_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_X_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Y_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Y_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Z_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_OFFSET_Z_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_X_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_X_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Y_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Y_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Z_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = GYR_OFFSET_Z_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_RADIUS_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = ACC_RADIUS_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_RADIUS_LSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	buf[0] = MAG_RADIUS_MSB;
	buf[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 2, 1000);

	float thetaM;
	float phiM;
	float thetaFold = 0;
	float thetaFnew;
	float phiFold = 0;
	float phiFnew;

	float thetaG = 0;
	float phiG = 0;

	float theta;
	float phi;

	float thetaRad;
	float phiRad;

	float Xm;
	float Ym;
	float psi;

	float dt;
	unsigned long millisOld = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

	/*
	 * Reading in Basic Data
	*/
	buf[0] = ACC_DATA_X_LSB;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, I2C_Addr, &buf[0], 18, 1000);

	xAcc = buf[0] | (buf[1] << 8);
	yAcc = buf[2] | (buf[3] << 8);
	zAcc = buf[4] | (buf[5] << 8);
	xMag = buf[6] | (buf[7] << 8);
	yMag = buf[8] | (buf[9] << 8);
	zMag = buf[10] | (buf[11] << 8);
	xGyro = buf[12] | (buf[13] << 8);
	yGyro = buf[14] | (buf[15] << 8);
	zGyro = buf[16] | (buf[17] << 8);

	acceleration[0] = (double) xAcc / 100.0;
	acceleration[1] = (double) yAcc / 100.0;
	acceleration[2] = (double) zAcc / 100.0;

	magnometer[0] = (double) xMag / 16.0;
	magnometer[1] = (double) yMag / 16.0;
	magnometer[2] = (double) zMag / 16.0;

	gyroscope[0] =  (double) xGyro / 16.0;
	gyroscope[1] =  (double) yGyro / 16.0;
	gyroscope[2] =  (double) zGyro / 16.0;

	/*
	 * CALIBRATING THE SYSTEM
	*/
	uint8_t system, gyro, accel, mg = 0;
	buf[0] = CALIB_STAT;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	system = (buf[0] >> 6) & 0x03;
	gyro = (buf[0] >> 4) & 0x03;
	accel = (buf[0] >> 2) & 0x03;
	mg = buf[0] & 0x03;

	/*
	 * Obtaining Yaw, Roll, Pitch
	*/
	buf[0] = EUL_HEADING_LSB;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, I2C_Addr, &buf[0], 6, 1000);
	MEuler[0] = buf[0] | (buf[1] << 8); // Heading
	MEuler[1] = buf[2] | (buf[3] << 8); // Roll
	MEuler[2] = buf[4] | (buf[5] << 8); // Pitch

	CEuler[0] = (double) MEuler[0] / 16.0;
	CEuler[1] = (double) MEuler[1] / 16.0;
	CEuler[2] = (double) MEuler[2] / 16.0;

	// Quaternion Data
	buf[0] = QUA_DATA_W_LSB;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_Addr, &buf[0], 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, I2C_Addr, &buf[0], 8, 1000);
	MQuat[0] = buf[0] | (buf[1] << 8); // W
	MQuat[1] = buf[2] | (buf[3] << 8); // X
	MQuat[2] = buf[4] | (buf[5] << 8); // Y
	MQuat[3] = buf[6] | (buf[7] << 8); // Z

	const double scale = (1.0 / (1 << 14));

	CQuat[0] = (double) MQuat[0] * scale;
	CQuat[1] = (double) MQuat[1] * scale;
	CQuat[2] = (double) MQuat[2] * scale;
	CQuat[3] = (double) MQuat[3] * scale;

	/*
	 * Tilt
	*/
	//thetaM = atan2(acceleration[0] / 9.8, acceleration[2] / 9.8) / 2 / 3.141592654 * 360;
	//phiM = atan2(acceleration[1] / 9.8, acceleration[2] / 9.8) / 2 / 3.141592654 * 360;
	/* Low Pass Filter */
	//phiFnew = 0.95 * phiFold  + 0.05 * phiM;
	//thetaFnew = 0.95 * thetaFold + 0.05 * thetaM;

	//dt = (HAL_GetTick() - millisOld) / 1000;
	//millisOld = HAL_GetTick();

	//theta = (theta + gyroscope[1] * dt) * 0.95 + thetaM * 0.05;
	//phi = (phi - gyroscope[0] * dt) * 0.95 + phiM * 0.05;
	//thetaG = thetaG + gyroscope[1] * dt;
	//phiG = phiG - gyroscope[0] * dt;

	//phiRad = phi / 360 * (2 * 3.141592654);
	//thetaRad = theta / 360 * (2 * 3.141592654);

	//Xm = magnometer[0] * cos(thetaRad) - magnometer[1] * sin(phiRad) * sin(thetaRad) + magnometer[2] * cos(phiRad) * sin(thetaRad);
	//Ym = magnometer[1] * cos(phiRad) + magnometer[2] * sin(phiRad);

	//psi = atan2(Ym, Xm) / (2 * 3.14) * 360;

	//phiFold = phiFnew;
	//thetaFold = thetaFnew;

	//printf("Acc Cal: %d Gyro Cal: %d Mag Cal: %d Sys Cal: %d \n", accel, gyro, mg, system);
	//printf("Acc X: %f, Acc Y: %f Acc Z: %f \n", acceleration[0], acceleration[1], acceleration[2]);
	//printf("%f %f %f %f %d %d %d %d \n", CQuat[0], CQuat[1], CQuat[2], CQuat[3], accel, gyro, mg, system);
	printf("%f %f %f %d %d %d %d \n", CEuler[1], CEuler[2], CEuler[0], accel, gyro, mg, system); // Roll, Pitch, Yaw
	HAL_Delay(BNO055_SAMPLERATE_DELAY_MS);
	/* End of Basic Data Reading */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}



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
