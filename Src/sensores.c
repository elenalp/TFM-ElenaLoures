#include "sensores.h"
#include "main.h"
#include "pantalla.h"
#include "adc.h"

#define INTERVALO_CALIB_ALCOHOL 500  //Milisegundos entre dos muestras del nivel de alcohol durante la calibración
#define INTERVALO_MED_ALCOHOL 5  //Milisegundos entre dos muestras del nivel de alcohol durante la medida
#define INTERVALO_MED_ALCOHOL_GUARDADAS 10  //Número de muestras entre valores medios guardados
#define MUESTRAS_CALIB_ALCOHOL 50  //Número de muestras para la calibración del sensor del nivel de alcohol
#define MUESTRAS_MED_ALCOHOL 50  //Número de muestras para la medida del nivel de alcohol
#define V_REF 3.3  //Valor de referencia de tensión
#define V_BAT_MIN 3.2 //Tensión mínima de la batería para que no se estropee
#define V_BAT_MAX 4.2 //Tensión máxima de la batería
#define TIMEOUT_ADC 100  //Valor de timeout para llamadas a HAL_ADC_PollForConversion()
#define ADC_MAX 4096  //Valor máximo del ADC


//Calibración del sensor del nivel de alcohol
uint32_t calibracionAlcohol(maq_estados* maquina_est){

	uint32_t valorADC1, valor_calibracion, t_inicial;
	uint8_t i;

	/*
	 * Leer nivel de alcohol (ADC)
	 */
	for(i=0; i<MUESTRAS_CALIB_ALCOHOL; i++){
		if(HAL_GetTick() >= t_inicial + INTERVALO_CALIB_ALCOHOL){  //Para que tome una muestra cada X tiempo
			HAL_ADC_Start(&hadc1);
			t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
			//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería

			medirBateria(maquina_est, valorADC1);

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			valor_calibracion = valor_calibracion + HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

			//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

			HAL_ADC_Stop(&hadc1);
		}
		valor_calibracion = valor_calibracion/MUESTRAS_CALIB_ALCOHOL;
	}
	return valor_calibracion;
}


/*
 * Activa el sensor de alcohol y almacena en la estructura el valor detectado
 * Se toman valores, se hacen varias medias y luego la media de esas medias excluyendo la más baja (son probablemente las medidas de cuando no está soplando)
 */
void medirAlcohol(maq_estados* maquina_est){

	//uint32_t valorADC1, valorADC2;  //, valorADC3;  //Lecturas del ADC
	uint32_t valorADC1, valorADC2;   //Lecturas del ADC
	uint32_t t_inicial;  //, t_final;  //Valor del instante actual en milisegundos
	uint32_t referencia;  //Valor de referencia para la medición del nivel de alcohol, offset
	//float tension_bat;   //Valor de tensión del divisor de la batería
	uint8_t i, j, k, l;
	//uint32_t medidasAlcohol[INTERVALO_MED_ALCOHOL_GUARDADAS];
	uint32_t medias[MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS];
	uint32_t medidaAlcohol;  // Medida de alcohol tras hacer las medias sin aplicar el offset
	uint32_t min_medida;  //Medida mínima de alcohol de las medias

	imprimirAviso(maquina_est, 0);    //Imprimir bienvenida
	imprimirAviso(maquina_est, 1);    //Imprimir que va a calibrar

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF

	referencia = calibracionAlcohol(maquina_est);  //Calibración del sensor del nivel de alcohol
	imprimirAviso(maquina_est, 2);    //Imprimir inicio de medición del nivel de alcohol

	t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

	/*
	 * Leer nivel de alcohol (ADC)
	 */
	for(i=0; i<INTERVALO_MED_ALCOHOL; i++){
		j++;

		HAL_ADC_Start (&hadc1);

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
		//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería

		medirBateria(maquina_est, valorADC1);

		/*tension_bat = 2*(valorADC1/4096)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensión Vin_bat
		maquina_est->nivel_bateria = (uint16_t)(((tension_bat-V_BAT_MIN)/(V_BAT_MAX-V_BAT_MIN))*100);  //Sacar el porcentaje de batería que le queda (estimación en tensión)
	*/
		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		//valorADC2 = HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
		//medidasAlcohol[j] = HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
		valorADC2 = valorADC2 + HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

		if(j == INTERVALO_MED_ALCOHOL_GUARDADAS){  //Para hacer la media y almacenar un valor
			j = 0;
			medias[k] = valorADC2/INTERVALO_MED_ALCOHOL_GUARDADAS;  //Hace la media de los valores almacenados hasta ese punto
			valorADC2 = 0;
			k++;
		}

		//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

		HAL_ADC_Stop(&hadc1);
	}

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_RESET);  //Poner a 0 HeaterON_OFF

	for(l=0; l<MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS; l++){
		medidaAlcohol = medidaAlcohol + medias[l];
		if(min_medida > medias[l]){  //Para saber qué medida descartar
			min_medida = medias[l];
		}
	}
	medidaAlcohol = medidaAlcohol - min_medida;  //Para descartar la mínima media obtenida

	maquina_est->medidas_sens->alcohol = (medidaAlcohol - referencia)/ADC_MAX;  //CAMBIAR!!!!

	imprimirAviso(maquina_est, 3); //Imprimir fin de medición del nivel de alcohol

	//COMPROBAR SI ESTÁ DENTRO DE LOS RANGOS!!!
	//0.25mg/l
	//tension_bat = 2*(valorADC1/ADC_MAX)*V_REF;

	//PENDIENTE DE MIRAR CÓMO OBTENER EL VALOR!!!

}

//Activa todos los sensores excepto el sensor de alcohol y almacena en la estructura los valores detectados
void medirSensores(maq_estados* maquina_est){
	uint32_t valorADC1;   //Lecturas del ADC

//	imprimirMedidas();  //Pantalla de medidas de sensores
	medirTemp(maquina_est);
	medirBateria(maquina_est, valorADC1);
}

/*
//Saca por pantalla todas las medidas de los sensores
void imprimirMedidas(maq_estados* maquina_est){
	//ES IGUAL QUE IMPRIMIR SENSORES!!!!! CAMBIAR EL NOMBRE DE LA OTRA Y DEJARLA EN PANTALLA.C
}
*/

//Activa el motor para alertar mediante vibración
void vibracion(void){

}

//Imprime por pantalla el parámetro por el cual se realiza la alerta y activa la vibración
void imprimirYvibrar(maq_estados* maquina_est){
	//LLAMAR A LA FUNCIÓN DE ALERTA DE PANTALLA.C Y A VIBRACION()

	vibracion();
	//imprimirAlertaSensor(maquina_est->medidas_sens, maquina_est->medidas_sens->medida_mala);
	imprimirAviso(maquina_est, 5);
}

/*
//Imprime un mensaje por pantalla en función del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si batería baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido
void imprimirAviso(maq_estados* maquina_est, int tipoAviso){
	//LLAMAR A LAS FUNCIONES DE IMPRIMIR SEGÚN EL TIPO DE AVISO Y CAMBIAR ESTA DEFINICIÓN PARA QUE ESTÉ EN PANTALLA.C

}
*/

//Realiza la medición de la temperatura corporal
void medirTemp(maq_estados* maquina_est){
	/*uint32_t temp;
	  uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
	  bufferTemp[0] = 0x01; //Dirección de registro de configuración
	  bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuración
	  while(HAL_I2C_IsDeviceReady(&hi2c3, 0X90, 2, 10) != HAL_OK);
	  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 2, 10);

	  while (1)
	  {

		  bufferTemp[0] = 0x00; //Dirección de registro de temperatura
		  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 1, 10);
		  HAL_I2C_Master_Receive(&hi2c3, 0x90, bufferTemp, 2, 10);

		  temp = bufferTemp[0]*256 + bufferTemp[1];  //Primero devuelve el más significativo, como son 8 bits cada registro, para obtener el valor de ambos bytes hay que multiplicar el primero por 256

		 // temp = temp * 0.00390625;  // Porque cada bit no vale 1ºC sino esos grados

		  //imprimirSecuencia(6);
		 //imprimirAlertaSensor(187, 45, 1);
		  imprimirAlertaSensor(temp, 45, 1);*/
}

//Estima la batería que tiene
void medirBateria(maq_estados* maquina_est, uint32_t valorADC1){
	float tension_bat;   //Valor de tensión del divisor de la batería

	HAL_ADC_Start (&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
	HAL_ADC_Stop(&hadc1);

	tension_bat = 2*(valorADC1/ADC_MAX)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensión Vin_bat
	maquina_est->nivel_bateria = (uint16_t)(((tension_bat-V_BAT_MIN)/(V_BAT_MAX-V_BAT_MIN))*100);  //Sacar el porcentaje de batería que le queda (estimación en tensión) en uint porque no tenemos precisión

}
