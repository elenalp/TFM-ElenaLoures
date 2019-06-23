#include "sensores.h"
#include "main.h"
#include "pantalla.h"
#include "adc.h"
#include "i2c.h"
#include <math.h>

#define INTERVALO_CALIB_ALCOHOL 500  //Milisegundos entre dos muestras del nivel de alcohol durante la calibraci�n
#define INTERVALO_MED_ALCOHOL 5  //Milisegundos entre dos muestras del nivel de alcohol durante la medida
#define INTERVALO_MED_ALCOHOL_GUARDADAS 10  //N�mero de muestras entre valores medios guardados
#define MUESTRAS_CALIB_ALCOHOL 50.0  //N�mero de muestras para la calibraci�n del sensor del nivel de alcohol (.0 para que interprete que es float)
#define MUESTRAS_MED_ALCOHOL 50  //N�mero de muestras para la medida del nivel de alcohol
#define V_REF 3.3  //Valor de referencia de tensi�n
#define V_BAT_MIN 3.2 //Tensi�n m�nima de la bater�a para que no se estropee
#define V_BAT_MAX 4.2 //Tensi�n m�xima de la bater�a
#define TIMEOUT_ADC 100  //Valor de timeout para llamadas a HAL_ADC_PollForConversion()
#define ADC_MAX 4096  //Valor m�ximo del ADC
#define V_ALCOHOL 5  //Tensi�n con que se alimenta el sensor de gases
#define RL 750  //Resistencia de carga del circuito del sensor de gases
#define RELACION_RSRO 20.5  //Relaci�n entre resistencias del sensor de gases en aire
#define A_TRAMO1 10.375  //Primer par�metro de la ecuaci�n del tramo 1 del alcohol
#define B_TRAMO1 87.032  //Segundo par�metro de la ecuaci�n del tramo 1 del alcohol
#define C_TRAMO1 230.55  //Tercer par�metro de la ecuaci�n del tramo 1 del alcohol
#define A_TRAMO2 163204  //Primer par�metro de la ecuaci�n del tramo 2 del alcohol
#define B_TRAMO2 688610  //Segundo par�metro de la ecuaci�n del tramo 2 del alcohol
#define C_TRAMO2 exp(6)  //Tercer par�metro de la ecuaci�n del tramo 2 del alcohol
#define D_TRAMO2 983296  //Cuarto par�metro de la ecuaci�n del tramo 2 del alcohol
#define E_TRAMO2 441823  //Quinto par�metro de la ecuaci�n del tramo 2 del alcohol
#define F_TRAMO2 100218  //Sexto par�metro de la ecuaci�n del tramo 2 del alcohol
#define G_TRAMO2 10120  //S�ptimo par�metro de la ecuaci�n del tramo 2 del alcohol
#define A_TRAMO3 6666.7  //Primer par�metro de la ecuaci�n del tramo 3 del alcohol
#define B_TRAMO3 2666.7  //Segundo par�metro de la ecuaci�n del tramo 3 del alcohol
#define CTE_PPM2MG 1.8843*pow(10,-3)  //Cte para hacer la correspondencia de PPM a mg/l
#define MAX_ALCOHOL 0.25  //Tasa m�xima de alcohol permitida en aire expirado y conductores generales (mg/l)


//Calibraci�n del sensor del nivel de alcohol
//uint32_t calibracionAlcohol(maq_estados* maquina_est){
float calibracionAlcohol(void){
	float valorADC1, valor_calibracion, t_inicial, rs, ro;
	uint8_t i;

//HAL_Delay(18000); //Para que caliente el heater

	/*
	 * Leer nivel de alcohol (ADC)
	 */
//	t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)
	/*for(i=0; i<MUESTRAS_CALIB_ALCOHOL; i++){
		if(HAL_GetTick() >= t_inicial + INTERVALO_CALIB_ALCOHOL){  //Para que tome una muestra cada X tiempo
			HAL_ADC_Start(&hadc1);
			t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
			//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a

//			medirBateria(maquina_est, valorADC1);

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			valor_calibracion = valor_calibracion + (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

			//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

			HAL_ADC_Stop(&hadc1);
		}
		//valor_calibracion = valor_calibracion/MUESTRAS_CALIB_ALCOHOL;
	}*/

/*	i=0;
	//if((HAL_GetTick() >= t_inicial + INTERVALO_CALIB_ALCOHOL) && (i<MUESTRAS_CALIB_ALCOHOL)){  //Para que tome una muestra cada X tiempo
	//do{
		i++;
		HAL_ADC_Start(&hadc1);
		t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
		//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a

//			medirBateria(maquina_est, valorADC1);

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valor_calibracion = valor_calibracion + (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
		//valor_calibracion = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

		//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

		HAL_ADC_Stop(&hadc1);
	//}while((HAL_GetTick() >= t_inicial + INTERVALO_CALIB_ALCOHOL) && (i<MUESTRAS_CALIB_ALCOHOL));  //Para que tome una muestra cada X tiempo)
	 *
	 */
	//valor_calibracion = valor_calibracion/MUESTRAS_CALIB_ALCOHOL;
	//valor_calibracion = 0;

	for(i=0; i<MUESTRAS_CALIB_ALCOHOL; i++){

		HAL_ADC_Start(&hadc1);
		//t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
		//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a

	//			medirBateria(maquina_est, valorADC1);

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valor_calibracion = valor_calibracion + (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
		//valor_calibracion = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

		//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

		HAL_ADC_Stop(&hadc1);
	}

	valor_calibracion = (float)(valor_calibracion/MUESTRAS_CALIB_ALCOHOL);
	valor_calibracion = (float)((valor_calibracion/ADC_MAX)*V_REF);  //Porque los ADCs son de 12 bits (4096) que marcan los 3.3V (Vcc, la referencia del micro)

	rs = (float)(V_ALCOHOL*(RL/valor_calibracion))-RL;


	ro = (float)(rs/RELACION_RSRO);

/*	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valor_calibracion = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
	HAL_ADC_Stop(&hadc1);
	valor_calibracion = (valor_calibracion/ADC_MAX)*1.1;  //Porque los ADCs son de 12 bits (4096) que marcan los 5V (Vcc, la referencia)
	*/
	//return valor_calibracion;
	return ro;
}


/*
 * Activa el sensor de alcohol y almacena en la estructura el valor detectado
 * Se toman valores, se hacen varias medias y luego la media de esas medias excluyendo la m�s baja (son probablemente las medidas de cuando no est� soplando)
 */
//void medirAlcohol(maq_estados* maquina_est){
float medirAlcohol(void){   //CAMBIAR FUNCI�N Y SOBRE TODO FORMA DE APLICAR LA REFERENCIA!!!!!!!  SACAR V, LUEGO RS Y CON LA RO SACAR DE LA EXPONENCIAL LAS PPM Y HACER CONVERSI�N A MG/L
	//uint32_t valorADC1, valorADC2;  //, valorADC3;  //Lecturas del ADC
	float valorADC1, valorADC2;   //Lecturas del ADC
	uint32_t t_inicial;  //, t_final;  //Valor del instante actual en milisegundos
	float ro;  //Valor de ro obtenido de la calibraci�n para el ambiente en que se encuentra el sensor
	//float tension_bat;   //Valor de tensi�n del divisor de la bater�a
	uint8_t i, j, k, l;
	//uint32_t medidasAlcohol[INTERVALO_MED_ALCOHOL_GUARDADAS];
	float medias[MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS];
	float medidaAlcohol;  // Medida de alcohol tras hacer las medias sin aplicar el offset
	float min_media;  //Medida m�nima de alcohol de las medias
	float vrl;  //Tensi�n correspondiente a la medida de alcohol obtenida
	float rs;  //Valor de la resistencia del sensor en la situaci�n medida
	float relacion_r;  //Cociente de rs/ro para el caso medido
	float ppm, mg;  //Valor de medida de alcohol en ppm y en mg/l

//	imprimirAviso(maquina_est, 0);    //Imprimir bienvenida
//	imprimirAviso(maquina_est, 1);    //Imprimir que va a calibrar

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF
	HAL_Delay(18000); //Para que caliente el heater
//	ro = calibracionAlcohol(maquina_est);  //Calibraci�n del sensor del nivel de alcohol
	ro = calibracionAlcohol();  //Calibraci�n del sensor del nivel de alcohol
//	imprimirAviso(maquina_est, 2);    //Imprimir inicio de medici�n del nivel de alcohol

	t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

	/*
	 * Leer nivel de alcohol (ADC)
	 */
	j = 0;
	k = 0;
	for(i=0; i<MUESTRAS_MED_ALCOHOL; i++){
		j++;

		HAL_ADC_Start(&hadc1);
		//t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
	//			medirBateria(maquina_est, valorADC1);

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC2 = valorADC2 + (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

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

	/*for(i=0; i<MUESTRAS_MED_ALCOHOL; i++){
		if(HAL_GetTick() >= t_inicial + INTERVALO_MED_ALCOHOL){
			j++;

			HAL_ADC_Start (&hadc1);

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a
			//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel bater�a

	//		medirBateria(maquina_est, valorADC1);

			//tension_bat = 2*(valorADC1/4096)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensi�n Vin_bat
			//maquina_est->nivel_bateria = (uint16_t)(((tension_bat-V_BAT_MIN)/(V_BAT_MAX-V_BAT_MIN))*100);  //Sacar el porcentaje de bater�a que le queda (estimaci�n en tensi�n)

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			//valorADC2 = HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
			//medidasAlcohol[j] = HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
			valorADC2 = valorADC2 + (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

			if(j == INTERVALO_MED_ALCOHOL_GUARDADAS){  //Para hacer la media y almacenar un valor
				j = 0;
				medias[k] = valorADC2/INTERVALO_MED_ALCOHOL_GUARDADAS;  //Hace la media de los valores almacenados hasta ese punto
				valorADC2 = 0;
				k++;
			}
			medidaAlcohol = medidaAlcohol - min_medida;  //Para descartar la m�nima media obtenida

			//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

			HAL_ADC_Stop(&hadc1);
		}

	}*/

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_RESET);  //Poner a 0 HeaterON_OFF

	min_media = medias[0]; //Para que tenga un valor inicial con que comparar

	for(l=0; l<MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS; l++){
		medidaAlcohol = medidaAlcohol + medias[l];
		if(min_media > medias[l]){  //Para saber qu� medida descartar
			min_media = medias[l];
		}
	}

	medidaAlcohol = medidaAlcohol - min_media;  //Para descartar la m�nima media obtenida
	medidaAlcohol = (float)(medidaAlcohol/((MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS)-1)); //Hace la media de las medias (sin tener en cuenta la que se ha quitado, la m�nima

	vrl = (float)((medidaAlcohol/ADC_MAX)*V_REF);  //Porque los ADCs son de 12 bits (4096) que marcan los 3.3V (Vcc, la referencia del micro). Saca el valor de la tensi�n V_RL

	rs = (V_ALCOHOL*(RL/vrl)) - RL;  //C�lculo de Rs en funci�n de V_RL

	relacion_r = rs/ro;

	/*
	 * Hacer ajuste por tramos de la curva de correspondencia Rs/Ro vs PPM
	 * ppm : Obtener valor de ppm al que corresponde la relaci�n rs/ro
	 */
	if((relacion_r < 4) && (relacion_r >= 2)){  //Tramo 1
		//Ecuaci�n de la curva del tramo 1: y = 10.375x2 - 87.032x + 230.55;
		ppm = A_TRAMO1*pow(relacion_r, 2) - B_TRAMO1*relacion_r + C_TRAMO1;
	}else if((relacion_r < 2) && (relacion_r >= 0.2)){  //Tramo 2
		//Ecuaci�n de la curva del tramo 2: y = 163204*x6 - 688610x5 + 1E+06x4 - 983296x3 + 441823x2 - 100218x + 10120;
		ppm = A_TRAMO2*pow(relacion_r, 6) - B_TRAMO2*pow(relacion_r, 5) + C_TRAMO2*pow(relacion_r, 4) - D_TRAMO2*pow(relacion_r, 3) + E_TRAMO2*pow(relacion_r, 2) - F_TRAMO2*relacion_r + G_TRAMO2;
	}
	else if(relacion_r < 0.2){  //Tramo 3
		//Ecuaci�n de la curva del tramo 3: y = -6666.7x + 2666.7;
		ppm = -A_TRAMO3*relacion_r + B_TRAMO3;
	}else if(relacion_r > 4){
		//Aire
		ppm = 0;
	}

	mg = relacion_r*CTE_PPM2MG;  //Correspondencia de ppm en mg/l para etanol

//	maquina_est->medidas_sens->alcohol = mg;

//	imprimirAviso(maquina_est, 3); //Imprimir fin de medici�n del nivel de alcohol

	/*
	 * Comprobaci�n de si est� dentro de los rangos
	 */
	/* Ya se hace en otra funci�n
	 * if(!rangoAlcohol(maquina_est)){   //Si no est� dentro de rango
		maq_estados->medidas_sens->medida_mala;
		imprimirAviso(maquina_est, 5);  //Imprimir aviso de alcohol
	}
	*/

	return mg;  //SOLO PARA PRUEBAS
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
void medirTemp(maq_estados* maquina_est){}
uint32_t medirTemp1(void){
	uint32_t temp;
	uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
	bufferTemp[0] = 0x01; //Direcci�n de registro de configuraci�n
	bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuraci�n
	while(HAL_I2C_IsDeviceReady(&hi2c3, 0X90, 2, 10) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 2, 10);

//	while (1){

	  bufferTemp[0] = 0x00; //Direcci�n de registro de temperatura
	  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 1, 10);
	  HAL_I2C_Master_Receive(&hi2c3, 0x90, bufferTemp, 2, 10);

	  temp = bufferTemp[0]*256 + bufferTemp[1];  //Primero devuelve el m�s significativo, como son 8 bits cada registro, para obtener el valor de ambos bytes hay que multiplicar el primero por 256

	 // temp = temp * 0.00390625;  // Porque cada bit no vale 1�C sino esos grados
//	}
	  //imprimirSecuencia(6);
	 //imprimirAlertaSensor(187, 45, 1);
	//		  imprimirAlertaSensor(temp, 45, 1);
	return temp;
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

//Devuelve si la medida de alcohol est� o no dentro del rango
int rangoAlcohol(maq_estados* maquina_est){
	if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 1;
	}else{
		return 0;
	}
}

//Devuelve 1 si alguna medida se sale del rango establecido
int medida_mal(maq_estados* maquina_est){
	/*
	 * Comprobaci�n alcohol
	 */
	if(maquina_est->medidas_sens->alcohol >= MAX_ALCOHOL){
		maquina_est->medidas_sens->medida_mala = 1;  //Marcar como mala la medida del alcohol
		return 1;
	}else if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 0;
	}

	//COMPROBAR EL RESTO DE SENSORES
}

//Devuelve 1 si la medida de alcohol se encuentran en el rango establecido
int medida_bien(maq_estados* maquina_est){
	if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 1;
	}else{
		return 0;
	}
}

