/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "maquinaEstados.h"
#include "sensores.h"
#include "pantalla.h"


//#include <stdio.h>
//#include <stdlib.h>


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

/* USER CODE BEGIN PV */

//Estados de la máquina de estados
static enum estados_posibles{ALCOHOL, ALERTAR, MEDIR, BATERIA_BAJA, CARGANDO, MOSTRAR_ERROR};

/*
 * Definición de la máquina de estados
 * {Estado inicial, Condición para cambio de estado, Estado final, Acción en estado final}
 */
static maq_transiciones transi_posibles[16][4] = {
  {ALCOHOL, medida_mal_alcohol, ALERTAR, imprimirYvibrar},
  {ALCOHOL, medida_bien, MEDIR, medirSensores},
  {ALCOHOL, bat_baja, BATERIA_BAJA, imprimirAviso},
  {ALCOHOL, conectado, CARGANDO, imprimirAviso},
  {MEDIR, medida_mal, ALERTAR, imprimirYvibrar},
  {MEDIR, bat_baja, BATERIA_BAJA, imprimirAviso},
  {MEDIR, conectado, CARGANDO, imprimirAviso},
  {ALERTAR, alerta_medir_dada, MEDIR, medirSensores},
  {ALERTAR, bat_baja, BATERIA_BAJA, imprimirAviso},
  {BATERIA_BAJA, alerta_medir_dada, MEDIR, medirSensores},
  {BATERIA_BAJA, alerta_alcohol_dada, ALCOHOL, medirAlcohol},
  {BATERIA_BAJA, conectado, CARGANDO, imprimirAviso},
  {CARGANDO, desconectado, BATERIA_BAJA, imprimirAviso},
  {CARGANDO, sistema_ON, MOSTRAR_ERROR, imprimirAviso},
  {MOSTRAR_ERROR, alerta_dada, CARGANDO, imprimirAviso},
  {-1, NULL, -1, NULL },  //Caso de error, no hay funciones (punteros a null) y los estados origen y destino valen -1
};


/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  inicializarPantalla(); //Ejecutar todo lo necesario para poder utilizar la pantalla

  estado_maq* situacion_maq;
  medidasSensores* medidas_sens;
  //estado_maq* situacion_maq[] = {{0,0,0}};
  //imprimirBasico(8);
  //HAL_Delay(1000); //En ms
  maq_estados* maquina_estados = crear_maq(situacion_maq, transi_posibles, medidas_sens);

  //imprimirBasico(7);
   //HAL_Delay(1000); //En ms
  //wait(500);

 //imprimirPantalla();  //QUITAAAAAAAAAAR!!!
  /* USER CODE END 2 */
// uint32_t t1 = HAL_GetTick();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //uint32_t temp;
  /*uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
  bufferTemp[0] = 0x01; //Dirección de registro de configuración
  bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuración
  while(HAL_I2C_IsDeviceReady(&hi2c3, 0X90, 2, 10) != HAL_OK);
  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 2, 10);*/
  //float alcohol, ro;
  //float gsr;

// HAL_GPIO_WritePin(EnableSW_5V_GPIO_Port, EnableSW_5V_Pin, GPIO_PIN_SET);
// HAL_GPIO_WritePin(EnableLIN_1V8_GPIO_Port, EnableLIN_1V8_Pin, GPIO_PIN_SET);

// uint8_t ppg;

 // HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF
  //HAL_Delay(18000); //Para que caliente el heater
 // HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF
 // ro = calibracionAlcohol();
  //ro=1200;
  //imprimirPruebas(0, ro, 0);
  //HAL_Delay(3000); //En ms
  //imprimirBasico(9);
  //HAL_Delay(1000); //En ms
  //imprimirSecuencia(5);
  while (1)
  {

	  ejecutar_maq(maquina_estados);

//ro = 1000;
	 // imprimirSecuencia(6);

	  //temp = medirTemp1();
	  //imprimirBasico(4);
	 /* FILE * fp;

	     fp = fopen ("fileprueba.txt", "w+");
	     fprintf(fp, "%s %s %s %d", "We", "are", "in", 2012);

	     fclose(fp);*/

	 // gsr = medirGSR();
	  //alcohol = medirAlcohol();
	 // alcohol = 4.5;
//	  alcohol = calibracionAlcohol();

	  /*bufferTemp[0] = 0x00; //Dirección de registro de temperatura
	  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 1, 10);
	  HAL_I2C_Master_Receive(&hi2c3, 0x90, bufferTemp, 2, 10);

	  temp = bufferTemp[0]*256 + bufferTemp[1];  //Primero devuelve el más significativo, como son 8 bits cada registro, para obtener el valor de ambos bytes hay que multiplicar el primero por 256

	 temp = temp * 0.00390625;  // Porque cada bit no vale 1ºC sino esos grados
	  //temp = 0x100;
	  //imprimirSecuencia(6);
	 //imprimirAlertaSensor(187, 45, 1);
//	  imprimirAlertaSensor(temp, 45, 1);*/
	  //imprimirPruebas(3, temp, 0);
//	  imprimirPruebas(3, 90, 0);
//	  imprimirPruebas(0, alcohol, 0);
	  //imprimirPruebas(4, gsr, 0);

//	  ppg = medirTensionPulso();
//	  imprimirPruebas(4, ppg, 0);

	  /*//PARA PRUEBAS!!!
	  HAL_Delay(3000); //En ms
	  imprimirBasico(4);
	  HAL_Delay(3000); //En ms*/

//	  HAL_GPIO_TogglePin(LED_test_GPIO_Port, LED_test_Pin);
//	  HAL_Delay(180);
	/* USER CODE END WHILE */
	/*  if(HAL_GetTick()-t1 < 400){
		  imprimirPantalla(1);  //QUITAAAAAAAAAAR!!!
	  }else{
		  imprimirPantalla(0);
		  if(HAL_GetTick()-t1 > 600){
			  t1 = HAL_GetTick();
			  HAL_GPIO_TogglePin(LED_test_GPIO_Port, LED_test_Pin);
		  }
	  }*/
    /* USER CODE BEGIN 3 */

	  //Prueba parpadeo LED
/*	  HAL_GPIO_TogglePin(LED_test_GPIO_Port, LED_test_Pin);
	  HAL_Delay(250);*/
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

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
