/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Ball Receiver UI — RPM 3-10 (instant start/stop)
 *                   LCD: 2004A (20x4, HD44780 over PCF8574 I2C backpack)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RevRobot1DOF.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RPM_MIN           3
#define RPM_MAX           10
#define RPM_DEFAULT       5

#define LONG_PRESS_MS     1500
#define DEBOUNCE_MS       30
#define ENCODER_DETENT    4

#define LCD_BACKLIGHT     0x08
#define LCD_EN            0x04
#define LCD_RW            0x02
#define LCD_RS            0x01

#define STOPPED_HOLD_MS   1500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RevRobot1DOF_t BallReciever;
float power = 0.0f;
float target_angle = 0.0f;
float target_RPM = 0.0f;
float last_target_RPM = 0.0f;
float Tracking_Error = 0.0f;

// State machine
typedef enum {
	APP_READY = 0,
	APP_SELECT,
	APP_RUNNING,
	APP_STOPPED
} AppState_t;

static AppState_t app_state = APP_READY;
static uint8_t rpm_selected = RPM_DEFAULT;

static int16_t last_enc_count = 0;
static uint32_t btn_press_tick = 0;
static uint8_t btn_prev_state = 1;
static uint8_t btn_long_fired = 0;
static uint32_t blink_tick = 0;
static uint8_t blink_on = 1;
static uint32_t state_enter_tick = 0;

static uint8_t lcd_addr = 0x00;
static uint8_t lcd_addr_8bit = 0x00;

volatile uint8_t  dbg_lcd_addr_found = 0;
volatile uint32_t dbg_loop_count     = 0;
volatile uint8_t  dbg_app_state      = 0;
volatile uint8_t  dbg_rpm_selected   = 0;
volatile float    dbg_target_rpm     = 0.0f;
volatile float    dbg_meas_rpm       = 0.0f;
volatile uint32_t dbg_motor_send_cnt = 0;
volatile uint8_t  dbg_robot_mode     = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t LCD_Scan_Address(void);
static void LCD_Send4Bits(uint8_t nibble, uint8_t rs);
static void LCD_SendByte(uint8_t b, uint8_t rs);
static void LCD_Cmd(uint8_t cmd);
static void LCD_Data(uint8_t data);
static void LCD_Init(void);
static void LCD_PrintAt(uint8_t col, uint8_t row, const char *s);

static void Display_Ready(void);
static void Display_Select(uint8_t rpm, uint8_t blink_on);
static void Display_Running(float current_rpm, uint8_t target_rpm);
static void Display_Stopped(void);

static int8_t Encoder_ReadDelta(void);
static void Button_Poll(uint8_t *short_press, uint8_t *long_press);
static void App_Tick(void);

static void LED_Blink(uint8_t times, uint16_t delay_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void LED_Blink(uint8_t times, uint16_t delay_ms) {
	for (uint8_t i = 0; i < times; i++) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_ms);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_ms);
	}
}

// =====================================================================
// LCD2004A I2C driver (HD44780 over PCF8574 backpack)
// =====================================================================

static uint8_t LCD_Scan_Address(void) {
	static const uint8_t common[] = { 0x27, 0x3F, 0x20, 0x38 };
	for (uint8_t i = 0; i < sizeof(common); i++) {
		for (uint8_t retry = 0; retry < 3; retry++) {
			if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(common[i] << 1), 3, 50) == HAL_OK)
				return common[i];
			HAL_Delay(50);
		}
	}
	for (uint8_t a = 0x08; a < 0x78; a++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 20) == HAL_OK)
			return a;
	}
	return 0;
}

static void LCD_Send4Bits(uint8_t nibble, uint8_t rs) {
	uint8_t data       = (nibble & 0xF0) | LCD_BACKLIGHT | (rs ? LCD_RS : 0);
	uint8_t with_en    = data | LCD_EN;
	uint8_t without_en = data & ~LCD_EN;
	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr_8bit, &without_en, 1, 10);
	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr_8bit, &with_en,    1, 10);
	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr_8bit, &without_en, 1, 10);
	HAL_Delay(1);
}

static void LCD_SendByte(uint8_t b, uint8_t rs) {
	LCD_Send4Bits(b & 0xF0, rs);
	LCD_Send4Bits((uint8_t)(b << 4) & 0xF0, rs);
}

static void LCD_Cmd(uint8_t cmd)   { LCD_SendByte(cmd, 0); }
static void LCD_Data(uint8_t data) { LCD_SendByte(data, 1); }

static void LCD_Init(void) {
	HAL_Delay(50);
	LCD_Send4Bits(0x30, 0); HAL_Delay(5);
	LCD_Send4Bits(0x30, 0); HAL_Delay(1);
	LCD_Send4Bits(0x30, 0); HAL_Delay(1);
	LCD_Send4Bits(0x20, 0); HAL_Delay(1);
	LCD_Cmd(0x28);   // 4-bit, 2-line (HD44780 treats 2004A as 2x40 internally)
	LCD_Cmd(0x0C);   // Display ON, cursor OFF, blink OFF
	LCD_Cmd(0x06);   // Entry mode: increment, no shift
	LCD_Cmd(0x01);   // Clear display
	HAL_Delay(2);
}

/**
 * @brief  Print string at (col, row) on the 2004A.
 *         Row offsets for 20x4: row0=0x00, row1=0x40, row2=0x14, row3=0x54
 */
static void LCD_PrintAt(uint8_t col, uint8_t row, const char *s) {
	static const uint8_t row_offsets[4] = { 0x00, 0x40, 0x14, 0x54 };
	LCD_Cmd((uint8_t)(0x80 | (row_offsets[row & 3] + col)));
	while (*s) LCD_Data((uint8_t)*s++);
}

// =====================================================================
// Display layer  (20 chars wide, 4 rows)
// =====================================================================

static void Display_Ready(void) {
	LCD_PrintAt(0, 0, "====================");
	LCD_PrintAt(0, 1, "   Ball Receiver    ");
	LCD_PrintAt(0, 2, "   Press to Start   ");
	LCD_PrintAt(0, 3, "====================");
}

static void Display_Select(uint8_t rpm, uint8_t blink_on) {
	char line2[21];
	if (blink_on)
		snprintf(line2, sizeof(line2), "  Target RPM:  %2u   ", rpm);
	else
		snprintf(line2, sizeof(line2), "  Target RPM:       ");

	LCD_PrintAt(0, 0, "====================");
	LCD_PrintAt(0, 1, "   Configure RPM    ");
	LCD_PrintAt(0, 2, line2);
	LCD_PrintAt(0, 3, " Turn knob, hold OK ");
}

static void Display_Running(float current_rpm, uint8_t target_rpm) {
	char line2[21];
	char line3[21];

	float meas = current_rpm < 0.0f ? -current_rpm : current_rpm;
	if (meas > 99.9f) meas = 99.9f;

	uint16_t meas_x10 = (uint16_t)(meas * 10.0f + 0.5f);
	uint8_t  meas_int  = meas_x10 / 10;
	uint8_t  meas_frac = meas_x10 % 10;

	snprintf(line2, sizeof(line2), "  Meas : %2u.%u RPM   ", meas_int, meas_frac);
	snprintf(line3, sizeof(line3), "  Target:  %2u RPM   ", target_rpm);

	LCD_PrintAt(0, 0, "====================");
	LCD_PrintAt(0, 1, "      Running       ");
	LCD_PrintAt(0, 2, line2);
	LCD_PrintAt(0, 3, line3);
}

static void Display_Stopped(void) {
	LCD_PrintAt(0, 0, "====================");
	LCD_PrintAt(0, 1, "                    ");
	LCD_PrintAt(0, 2, "      Stopped       ");
	LCD_PrintAt(0, 3, "====================");
}

// =====================================================================
// Encoder reader
// =====================================================================

static int8_t Encoder_ReadDelta(void) {
	int16_t cur  = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	int16_t diff = cur - last_enc_count;
	if (diff >=  ENCODER_DETENT) { last_enc_count = cur; return +1; }
	if (diff <= -ENCODER_DETENT) { last_enc_count = cur; return -1; }
	return 0;
}

// =====================================================================
// Button polling
// =====================================================================

static void Button_Poll(uint8_t *short_press, uint8_t *long_press) {
	*short_press = 0;
	*long_press  = 0;
	uint8_t  cur = HAL_GPIO_ReadPin(BTN_SW_GPIO_Port, BTN_SW_Pin);
	uint32_t now = HAL_GetTick();

	if (btn_prev_state == 1 && cur == 0) {
		btn_press_tick = now;
		btn_long_fired = 0;
	} else if (btn_prev_state == 0 && cur == 0) {
		if (!btn_long_fired && (now - btn_press_tick) >= LONG_PRESS_MS) {
			*long_press    = 1;
			btn_long_fired = 1;
		}
	} else if (btn_prev_state == 0 && cur == 1) {
		uint32_t held = now - btn_press_tick;
		if (held >= DEBOUNCE_MS && held < LONG_PRESS_MS && !btn_long_fired)
			*short_press = 1;
	}
	btn_prev_state = cur;
}

// =====================================================================
// Application state machine
// =====================================================================

static void App_Tick(void) {
	uint8_t sp = 0, lp = 0;
	Button_Poll(&sp, &lp);
	int8_t   enc = Encoder_ReadDelta();
	uint32_t now = HAL_GetTick();
	dbg_loop_count++;

	// Debug mirror
	dbg_app_state    = (uint8_t)app_state;
	dbg_rpm_selected = rpm_selected;
	dbg_target_rpm   = target_RPM;
	dbg_meas_rpm     = BallReciever.RPM;
	dbg_robot_mode   = (uint8_t)BallReciever.Mode;

	static uint32_t last_lcd_update = 0;
	uint8_t do_refresh = 0;
	if (now - last_lcd_update >= 100) {
		do_refresh      = 1;
		last_lcd_update = now;
	}

	switch (app_state) {

	// ============================================================
	case APP_READY:
		if (sp) {
			blink_tick = now;
			blink_on   = 1;
			app_state  = APP_SELECT;
			Display_Select(rpm_selected, 1);
		}
		break;

	// ============================================================
	case APP_SELECT:
		if (now - blink_tick >= 350) {
			blink_tick = now;
			blink_on  ^= 1;
			Display_Select(rpm_selected, blink_on);
		}
		if (enc != 0) {
			int8_t v = (int8_t)rpm_selected + enc;
			if (v < RPM_MIN) v = RPM_MAX;
			if (v > RPM_MAX) v = RPM_MIN;
			rpm_selected = (uint8_t)v;
			blink_on     = 1;
			blink_tick   = now;
			Display_Select(rpm_selected, 1);
		}
		if (lp) {
			target_RPM = (float)rpm_selected;
			app_state  = APP_RUNNING;
			Display_Running(BallReciever.RPM, rpm_selected);
		}
		break;

	// ============================================================
	case APP_RUNNING:
		if (do_refresh)
			Display_Running(BallReciever.RPM, rpm_selected);
		if (lp) {
			target_RPM       = 0.0f;
			state_enter_tick = now;
			app_state        = APP_STOPPED;
			Display_Stopped();
		}
		break;

	// ============================================================
	case APP_STOPPED:
		if (now - state_enter_tick >= STOPPED_HOLD_MS) {
			app_state = APP_READY;
			Display_Ready();
		}
		break;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	LED_Blink(2, 200);

	// ==========================================================
	// Modular RevRobot1DOF Setup
	// ==========================================================
	RevRobot1DOF_init(&BallReciever);
	RevRobot1DOF_init_Motor(&BallReciever, MOTOR_PWM_HTIM, MOTOR_CH1, MOTOR_CH2,
			MOTOR_EN_PORT, MOTOR_EN_PIN);
	RevRobot1DOF_init_Encoder(&BallReciever, ENCODER_QEI_HTIM, OBSERVER_HTIM,
			ENCODER_PPR, ENCODER_X, ENCODER_OVERFLOW, OBSERVER_PERIOD);
	RevRobot1DOF_init_Filter(&BallReciever, VELO_CUTOFF_HZ, OBSERVER_PERIOD);
	RevRobot1DOF_init_Position_PID(&BallReciever, 0.09f, 0.00001f, 0.0f, 100.0f,
			POSITION_CONTROL_HTIM, 0.01f);
	RevRobot1DOF_init_Velocity_PID(&BallReciever, 5.0f, 0.01f, 0.0f, 100.0f,
			VELOCITY_CONTROL_HTIM);

	RevRobot1DOF_set_RPM(&BallReciever, target_RPM);
	last_target_RPM = target_RPM;

	// ==========================================================
	// UI Setup — encoder + button
	// ==========================================================
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	last_enc_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	btn_prev_state = HAL_GPIO_ReadPin(BTN_SW_GPIO_Port, BTN_SW_Pin);

	// ==========================================================
	// LCD2004A Init
	// ==========================================================
	HAL_Delay(500);
	for (uint8_t retry = 0; retry < 5; retry++) {
		lcd_addr = LCD_Scan_Address();
		if (lcd_addr != 0) break;
		HAL_Delay(200);
	}
	dbg_lcd_addr_found = lcd_addr;

	if (lcd_addr == 0) {
		// LCD not found — blink error pattern indefinitely
		while (1) {
			LED_Blink(5, 80);
			HAL_Delay(800);
		}
	}

	lcd_addr_8bit = (uint8_t)(lcd_addr << 1);
	LCD_Init();
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	Display_Ready();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (target_RPM != last_target_RPM) {
			RevRobot1DOF_set_RPM(&BallReciever, target_RPM);
			last_target_RPM = target_RPM;
			dbg_motor_send_cnt++;
		}

		Tracking_Error = BallReciever.TargetDegree
				- fmodf(BallReciever.Degree, 360.0f);

		// Heartbeat LED — 500 ms toggle while running
		static uint32_t led_tick = 0;
		uint32_t now = HAL_GetTick();
		if (now - led_tick >= 500) {
			led_tick = now;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}

		App_Tick();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	RevRobot1DOF_update(&BallReciever, htim);
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
#ifdef USE_FULL_ASSERT
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
