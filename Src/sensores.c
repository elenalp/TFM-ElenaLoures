#include "sensores.h"
#include "main.h"
#include "pantalla.h"
#include "adc.h"

#define INTERVALO_CALIB_ALCOHOL 500  //Milisegundos entre dos muestras del nivel de alcohol durante la calibraci�n
#define INTERVALO_MED_ALCOHOL 5  //Milisegundos entre dos muestras del nivel de alcohol durante la medida
#define INTERVALO_MED_ALCOHOL_GUARDADAS 10  //N�mero de muestras entre valores medios guardados
#define MUESTRAS_CALIB_ALCOHOL 50  //N�mero de muestras para la calibraci�n del sensor del nivel de alcohol
#define MUESTRAS_MED_ALCOHOL 50  //N�mero de muestras para la medida del nivel de alcohol
#define V_REF 3.3  //Valor de referencia de tensi�n
#define V_BAT_MIN 3.2 //Tensi�n m�nima de la bater�a para que no se estropee
#define V_BAT_MAX 4.2 //Tensi�n m�xima de la bater�a
#define TIMEOUT_ADC 100  //Valor de timeout para llamadas a HAL_ADC_PollForConversion()
#define ADC_MAX 4096  //Valor m�ximo del ADC


//Calibraci�n del sensor del nivel de alcohol
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
			valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
			//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a

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
 * Se toman valores, se hacen varias medias y luego la media de esas medias excluyendo la m�s baja (son probablemente las medidas de cuando no est� soplando)
 */
void medirAlcohol(maq_estados* maquina_est){

	//uint32_t valorADC1, valorADC2;  //, valorADC3;  //Lecturas del ADC
	uint32_t valorADC1, valorADC2;   //Lecturas del ADC
	uint32_t t_inicial;  //, t_final;  //Valor del instante actual en milisegundos
	uint32_t referencia;  //Valor de referencia para la medici�n del nivel de alcohol, offset
	//float tension_bat;   //Valor de tensi�n del divisor de la bater�a
	uint8_t i, j, k, l;
	//uint32_t medidasAlcohol[INTERVALO_MED_ALCOHOL_GUARDADAS];
	uint32_t medias[MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS];
	uint32_t medidaAlcohol;  // Medida de alcohol tras hacer las medias sin aplicar el offset
	uint32_t min_medida;  //Medida m�nima de alcohol de las medias

	imprimirAviso(maquina_est, 0);    //Imprimir bienvenida
	imprimirAviso(maquina_est, 1);    //Imprimir que va a calibrar

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF

	referencia = calibracionAlcohol(maquina_est);  //Calibraci�n del sensor del nivel de alcohol
	imprimirAviso(maquina_est, 2);    //Imprimir inicio de medici�n del nivel de alcohol

	t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

	/*
	 * Leer nivel de alcohol (ADC)
	 */
	for(i=0; i<INTERVALO_MED_ALCOHOL; i++){
		j++;

		HAL_ADC_Start (&hadc1);

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
		//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a

		medirBateria(maquina_est, valorADC1);

		/*tension_bat = 2*(valorADC1/4096)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensi�n Vin_bat
		maquina_est->nivel_bateria = (uint16_t)(((tension_bat-V_BAT_MIN)/(V_BAT_MAX-V_BAT_MIN))*100);  //Sacar el porcentaje de bater�a que le queda (estimaci�n en tensi�n)
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
		if(min_medida > medias[l]){  //Para saber qu� medida descartar
			min_medida = medias[l];
		}
	}
	medidaAlcohol = medidaAlcohol - min_medida;  //Para descartar la m�nima media obtenida

	maquina_est->medidas_sens->alcohol = (medidaAlcohol - referencia)/ADC_MAX;  //CAMBIAR!!!!

	imprimirAviso(maquina_est, 3); //Imprimir fin de medici�n del nivel de alcohol

	//COMPROBAR SI EST� DENTRO DE LOS RANGOS!!!
	//0.25mg/l
	//tension_bat = 2*(valorADC1/ADC_MAX)*V_REF;

	//PENDIENTE DE MIRAR C�MO OBTENER EL VALOR!!!

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

//Activa el motor para alertar mediante vibraci�n
void vibracion(void){

}

//Imprime por pantalla el par�metro por el cual se realiza la alerta y activa la vibraci�n
void imprimirYvibrar(maq_estados* maquina_est){
	//LLAMAR A LA FUNCI�N DE ALERTA DE PANTALLA.C Y A VIBRACION()

	vibracion();
	//imprimirAlertaSensor(maquina_est->medidas_sens, maquina_est->medidas_sens->medida_mala);
	imprimirAviso(maquina_est, 5);
}

/*
//Imprime un mensaje por pantalla en funci�n del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si bater�a baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido
void imprimirAviso(maq_estados* maquina_est, int tipoAviso){
	//LLAMAR A LAS FUNCIONES DE IMPRIMIR SEG�N EL TIPO DE AVISO Y CAMBIAR ESTA DEFINICI�N PARA QUE EST� EN PANTALLA.C

}
*/

//Realiza la medici�n de la temperatura corporal
void medirTemp(maq_estados* maquina_est){
	/*uint32_t temp;
	  uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
	  bufferTemp[0] = 0x01; //Direcci�n de registro de configuraci�n
	  bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuraci�n
	  while(HAL_I2C_IsDeviceReady(&hi2c3, 0X90, 2, 10) != HAL_OK);
	  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 2, 10);

	  while (1)
	  {

		  bufferTemp[0] = 0x00; //Direcci�n de registro de temperatura
		  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 1, 10);
		  HAL_I2C_Master_Receive(&hi2c3, 0x90, bufferTemp, 2, 10);

		  temp = bufferTemp[0]*256 + bufferTemp[1];  //Primero devuelve el m�s significativo, como son 8 bits cada registro, para obtener el valor de ambos bytes hay que multiplicar el primero por 256

		 // temp = temp * 0.00390625;  // Porque cada bit no vale 1�C sino esos grados

		  //imprimirSecuencia(6);
		 //imprimirAlertaSensor(187, 45, 1);
		  imprimirAlertaSensor(temp, 45, 1);*/
}

//Estima la bater�a que tiene
void medirBateria(maq_estados* maquina_est, uint32_t valorADC1){
	float tension_bat;   //Valor de tensi�n del divisor de la bater�a

	HAL_ADC_Start (&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
	HAL_ADC_Stop(&hadc1);

	tension_bat = 2*(valorADC1/ADC_MAX)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensi�n Vin_bat
	maquina_est->nivel_bateria = (uint16_t)(((tension_bat-V_BAT_MIN)/(V_BAT_MAX-V_BAT_MIN))*100);  //Sacar el porcentaje de bater�a que le queda (estimaci�n en tensi�n) en uint porque no tenemos precisi�n

}
