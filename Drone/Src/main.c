#include "main.h"
#include "TIMER.h"
#include "SPI.h"
#include "UART.h"
#include "I2C.h"
#include "GPIO.h"
#include "RCC.h"
#include "GY80.h"
#include "ADC.h"
#include "PID.h"


extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;


static void setup_motor (void);
double DeltaT = 1;
double time;
GY80_t GY80;

uint8_t Thurst ;//force for drone start move
PID_t pid_pitch, pid_roll, pid_yaw, pid_altitude;

uint16_t test=0 ;

uint32_t t1=0,t=0,t2=0;

uint16_t set_pitch =45, set_roll=45;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
	MX_ADC1_Init();
  MX_USART1_UART_Init();
            	
	ADXL_Init(&hi2c1);
	L3G4_Init(&hi2c1);
	HMC5883L_Init(&hi2c1);
	BMP180_Start();
	HAL_Delay(200);
	//PID value set KP, KI, KD and caculation ampha betal gamma
	setK(1,0.3,0.3,&pid_pitch);
	setK(1,1,1,&pid_roll);
	setK(1,1,1,&pid_yaw);
	setK(1,1,1,&pid_altitude);
	parameter_calculation(&pid_pitch);
	parameter_calculation(&pid_roll);
	parameter_calculation(&pid_yaw);
	parameter_calculation(&pid_altitude);
	//TIM2 PWM controller motors
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	//Set up motor automatically
	//setup_motor();
	Calib(&GY80);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim1);
            	
  while (1)
		{
			time = get_time();
			Angle_GYRO(&hi2c1,&GY80);
			Angle_ACC(&hi2c1,&GY80);
			Kalman_angle_solve(&hi2c1,&GY80);
		}
}
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM1)
    {
				t+=1;  	
  	}
    if(htim->Instance==TIM3)
  	{ 
		get_out(&pid_pitch,set_pitch,GY80.KalmanAngleX);
		get_out(&pid_roll,set_roll,GY80.KalmanAngleY);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	
  	}
}
 
static void PWM_caculation (int8_t pitch, int8_t roll,int8_t yaw, int8_t altitude , double proportion){
	TIM2->CCR1 = Thurst - pitch + roll - yaw + altitude;
	TIM2->CCR2 = Thurst - pitch - roll + yaw + altitude;
	TIM2->CCR3 = Thurst + pitch - roll - yaw + altitude;
	TIM2->CCR4 = Thurst + pitch + roll + yaw + altitude;
}

double get_time (void){
		t2=t1;
		t1=t;
		return (t1-t2)/1000.000;
}
 
static void setup_motor (void)
{
		uint16_t pwm_value;
		uint16_t ADC_Value[1];
		
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		//Set Max value of PWM
		ADC_Value[0] = 3040;//Maximun of ADC Read from NRF
		pwm_value = ADC_Value[0]/1.5 +2000;
		TIM4->CCR1 = pwm_value;
		TIM4->CCR2 = pwm_value;
		TIM4->CCR3 = pwm_value;
		TIM4->CCR4 = pwm_value;
										
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);
										
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		ADC_Value[0] = 61;//Minimun of ADC Read from NRF
		pwm_value = ADC_Value[0]/1.5 +2000;
		TIM4->CCR1 = pwm_value;
		TIM4->CCR2 = pwm_value;
		TIM4->CCR3 = pwm_value;
		TIM4->CCR4 = pwm_value;
		HAL_Delay(5000);        	
}
 
 
void Error_Handler(void){
  __disable_irq();
  while (1)
  {
  }
 
}
 
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *     	where the assert_param error has occurred.
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
