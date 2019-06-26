#include "sensores.h"
#include "main.h"
#include "pantalla.h"
#include "adc.h"
#include "i2c.h"
#include <math.h>

#define INTERVALO_CALIB_ALCOHOL 500  //Milisegundos entre dos muestras del nivel de alcohol durante la calibración
#define INTERVALO_MED_ALCOHOL 5  //Milisegundos entre dos muestras del nivel de alcohol durante la medida
#define INTERVALO_MED_ALCOHOL_GUARDADAS 10  //Número de muestras entre valores medios guardados
#define MUESTRAS_CALIB_ALCOHOL 50  //Número de muestras para la calibración del sensor del nivel de alcohol
#define MUESTRAS_MED_ALCOHOL 50  //Número de muestras para la medida del nivel de alcohol
#define MUESTRAS_CALIB_GSR 50 //Número de muestras para la calibración del GSR
#define V_REF 3.3  //Valor de referencia de tensión
#define V_BAT_MIN 3.2 //Tensión mínima de la batería para que no se estropee
#define V_BAT_MAX 4.2 //Tensión máxima de la batería
#define TIMEOUT_ADC 100  //Valor de timeout para llamadas a HAL_ADC_PollForConversion()
#define ADC_MAX 4096  //Valor máximo del ADC
#define V_ALCOHOL 5  //Tensión con que se alimenta el sensor de gases
#define RL 750  //Resistencia de carga del circuito del sensor de gases
#define RELACION_RSRO 20.5  //Relación entre resistencias del sensor de gases en aire
#define A_TRAMO1 10.375  //Primer parámetro de la ecuación del tramo 1 del alcohol
#define B_TRAMO1 87.032  //Segundo parámetro de la ecuación del tramo 1 del alcohol
#define C_TRAMO1 230.55  //Tercer parámetro de la ecuación del tramo 1 del alcohol

//y = -32548x5 + 102490x4 - 119134x3 + 63102x2 - 16426x + 2839,6
#define A_TRAMO2 32548  //Primer parámetro de la ecuación del tramo 2 del alcohol
#define B_TRAMO2 102490  //Segundo parámetro de la ecuación del tramo 2 del alcohol
#define C_TRAMO2 119134  //Tercer parámetro de la ecuación del tramo 2 del alcohol
#define D_TRAMO2 63102  //Cuarto parámetro de la ecuación del tramo 2 del alcohol
#define E_TRAMO2 16426  //Quinto parámetro de la ecuación del tramo 2 del alcohol
#define F_TRAMO2 2839.6  //Sexto parámetro de la ecuación del tramo 2 del alcohol

/*#define A_TRAMO2 163204  //Primer parámetro de la ecuación del tramo 2 del alcohol
#define B_TRAMO2 688610  //Segundo parámetro de la ecuación del tramo 2 del alcohol
//#define C_TRAMO2 exp(6)  //Tercer parámetro de la ecuación del tramo 2 del alcohol
#define C_TRAMO2 pow(10, 6)  //Tercer parámetro de la ecuación del tramo 2 del alcohol
#define D_TRAMO2 983296  //Cuarto parámetro de la ecuación del tramo 2 del alcohol
#define E_TRAMO2 441823  //Quinto parámetro de la ecuación del tramo 2 del alcohol
#define F_TRAMO2 100218  //Sexto parámetro de la ecuación del tramo 2 del alcohol
#define G_TRAMO2 10120  //Séptimo parámetro de la ecuación del tramo 2 del alcohol*/
#define A_TRAMO3 6666.7  //Primer parámetro de la ecuación del tramo 3 del alcohol
#define B_TRAMO3 2666.7  //Segundo parámetro de la ecuación del tramo 3 del alcohol
#define CTE_PPM2MG (1.8843*pow(10,(-3)))  //Cte para hacer la correspondencia de PPM a mg/l
//#define CTE_PPM2MG (1.8843)  //Cte para hacer la correspondencia de PPM a mg/l
#define MAX_ALCOHOL 0.25  //Tasa máxima de alcohol permitida en aire expirado y conductores generales (mg/l)
#define MAX_ESTRES "Alto" //Valor con el que hacer saltar la alerta por estrés
#define MIN_DIF_GSR 0.1 //Mínima diferencia de valor de GSR con respecto a la referencia para considerar estrés normal
#define MAX_DIF_GSR 0.4 //Máxim diferencia de valor de GSR con respecto a la referencia para considerar estrés normal
#define MIN_DIF_PULSO 0.1 //Mínima diferencia de valor de pulso con respecto a la referencia para considerar estrés normal
#define MAX_DIF_PULSO 0.4 //Máxim diferencia de valor de pulso con respecto a la referencia para considerar estrés normal


//Calibración del sensor del nivel de alcohol
//uint32_t calibracionAlcohol(maq_estados* maquina_est){
float calibracionAlcohol(void){
	float valorADC1, valor_calibracion, t_inicial, rs, ro;
	uint8_t i;

//HAL_Delay(10000); //Para que caliente el heater y se cargue el condensador

	/*
	 * Leer nivel de alcohol (ADC)
	 */
//	t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)
	/*for(i=0; i<MUESTRAS_CALIB_ALCOHOL; i++){
		if(HAL_GetTick() >= t_inicial + INTERVALO_CALIB_ALCOHOL){  //Para que tome una muestra cada X tiempo
			HAL_ADC_Start(&hadc1);
			t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

			HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
			//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería

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
		valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
		//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería

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
	valor_calibracion = 0.0;

	for(i=0; i<MUESTRAS_CALIB_ALCOHOL; i++){

		HAL_ADC_Start(&hadc1);
		//t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
		//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería

	//			medirBateria(maquina_est, valorADC1);

		HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		valor_calibracion = valor_calibracion + (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
		//valor_calibracion = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

		//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
		//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

		HAL_ADC_Stop(&hadc1);
	}

	valor_calibracion = (float)(valor_calibracion/(float)MUESTRAS_CALIB_ALCOHOL);
	valor_calibracion = (float)(((float)(valor_calibracion/ADC_MAX))*V_REF);  //Porque los ADCs son de 12 bits (4096) que marcan los 3.3V (Vcc, la referencia del micro)

	rs = (float)(V_ALCOHOL*((float)(RL/valor_calibracion)))-RL;

	ro = (float)(rs/RELACION_RSRO);
	//imprimirBasico(9); //SOLO PARA PRUEBAS!!!

/*	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
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
 * Se toman valores, se hacen varias medias y luego la media de esas medias excluyendo la más baja (son probablemente las medidas de cuando no está soplando)
 */
//void medirAlcohol(maq_estados* maquina_est){
float medirAlcohol(void){
	//uint32_t valorADC1, valorADC2;  //, valorADC3;  //Lecturas del ADC
	float valorADC1, valorADC2;   //Lecturas del ADC
	uint32_t t_inicial;  //, t_final;  //Valor del instante actual en milisegundos
	float ro;  //Valor de ro obtenido de la calibración para el ambiente en que se encuentra el sensor
	//float tension_bat;   //Valor de tensión del divisor de la batería
	uint8_t i, j, k, l;
	//uint32_t medidasAlcohol[INTERVALO_MED_ALCOHOL_GUARDADAS];
	float medias[(int)(MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS)];
	float medidaAlcohol;  // Medida de alcohol tras hacer las medias sin aplicar el offset
	float min_media;  //Medida mínima de alcohol de las medias
	float vrl;  //Tensión correspondiente a la medida de alcohol obtenida
	float rs;  //Valor de la resistencia del sensor en la situación medida
	float relacion_r;  //Cociente de rs/ro para el caso medido
	float ppm, mg;  //Valor de medida de alcohol en ppm y en mg/l

//	imprimirAviso(maquina_est, 0);    //Imprimir bienvenida
//	imprimirAviso(maquina_est, 1);    //Imprimir que va a calibrar

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF
	HAL_Delay(10000); //Para que caliente el heater y se cargue el condensador (10 seg)
//	ro = calibracionAlcohol(maquina_est);  //Calibración del sensor del nivel de alcohol
	ro = calibracionAlcohol();  //Calibración del sensor del nivel de alcohol
//	imprimirAviso(maquina_est, 2);    //Imprimir inicio de medición del nivel de alcohol

	//PARA PRUEBAS!!!
	imprimirBasico(9);
	HAL_Delay(3000); //En ms

	//t_inicial = HAL_GetTick(); //Tomar valor de tiempo actual (en milisegundos)

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
		valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
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
			valorADC1 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
			//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería

	//		medirBateria(maquina_est, valorADC1);

			//tension_bat = 2*(valorADC1/4096)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensión Vin_bat
			//maquina_est->nivel_bateria = (uint16_t)(((tension_bat-V_BAT_MIN)/(V_BAT_MAX-V_BAT_MIN))*100);  //Sacar el porcentaje de batería que le queda (estimación en tensión)

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
			medidaAlcohol = medidaAlcohol - min_medida;  //Para descartar la mínima media obtenida

			//HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
			//valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

			HAL_ADC_Stop(&hadc1);
		}

	}*/

//	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_RESET);  //Poner a 0 HeaterON_OFF

	min_media = medias[0]; //Para que tenga un valor inicial con que comparar

	for(l=0; l<MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS; l++){
		medidaAlcohol = medidaAlcohol + medias[l];
		if(min_media > medias[l]){  //Para saber qué medida descartar
			min_media = medias[l];
		}
	}

	medidaAlcohol = medidaAlcohol - min_media;  //Para descartar la mínima media obtenida
	medidaAlcohol = (float)(medidaAlcohol/((MUESTRAS_MED_ALCOHOL/INTERVALO_MED_ALCOHOL_GUARDADAS)-1)); //Hace la media de las medias (sin tener en cuenta la que se ha quitado, la mínima

	vrl = (float)((medidaAlcohol/ADC_MAX)*V_REF);  //Porque los ADCs son de 12 bits (4096) que marcan los 3.3V (Vcc, la referencia del micro). Saca el valor de la tensión V_RL

	rs = (V_ALCOHOL*(RL/vrl)) - RL;  //Cálculo de Rs en función de V_RL

	relacion_r = (float)(rs/ro);
	//imprimirBasico(7); //SOLO PARA PRUEBAS!!!!
	/*
	 * Hacer ajuste por tramos de la curva de correspondencia Rs/Ro vs PPM
	 * ppm : Obtener valor de ppm al que corresponde la relación rs/ro
	 */
	//if((relacion_r < 4) && (relacion_r >= 2)){  //Tramo 1
	if((relacion_r < 4) && (relacion_r > 1.1)){  //Tramo 1
		//Ecuación de la curva del tramo 1: y = 10.375x2 - 87.032x + 230.55;
		ppm = (float)(A_TRAMO1*pow(relacion_r, 2) - B_TRAMO1*relacion_r + C_TRAMO1);
		//ppm = 1;  //PARA PRUEBAS!
	//}else if((relacion_r < 2) && (relacion_r >= 0.2)){  //Tramo 2
	}else if((relacion_r <= 1.1) && (relacion_r >= 0.2)){  //Tramo 2
		//Ecuación de la curva del tramo 2: y = 163204*x6 - 688610x5 + 1E+06x4 - 983296x3 + 441823x2 - 100218x + 10120;
		//ppm = (float)(A_TRAMO2*pow(relacion_r, 6) - B_TRAMO2*pow(relacion_r, 5) + C_TRAMO2*pow(relacion_r, 4) - D_TRAMO2*pow(relacion_r, 3) + E_TRAMO2*pow(relacion_r, 2) - F_TRAMO2*relacion_r + G_TRAMO2);
		//Ecuación de la curva del tramo 2: y = -32548x5 + 102490x4 - 119134x3 + 63102x2 - 16426x + 2839,6
		ppm = (float)((-A_TRAMO2)*pow(relacion_r, 5) + B_TRAMO2*pow(relacion_r, 4) - C_TRAMO2*pow(relacion_r, 3) + D_TRAMO2*pow(relacion_r, 2) - E_TRAMO2*relacion_r + F_TRAMO2);
		//ppm = 2;  //PARA PRUEBAS!
	}
	else if(relacion_r < 0.2){  //Tramo 3
		//Ecuación de la curva del tramo 3: y = -6666.7x + 2666.7;
		ppm = (float)(-A_TRAMO3*relacion_r + B_TRAMO3);
		//ppm = 3;  //PARA PRUEBAS!
	}else if(relacion_r > 4){
		//Aire
		ppm = 0.0;
	}

	mg = (float)(ppm*CTE_PPM2MG);  //Correspondencia de ppm en mg/l para etanol

//	maquina_est->medidas_sens->alcohol = mg;

//	imprimirAviso(maquina_est, 3); //Imprimir fin de medición del nivel de alcohol

	/*
	 * Comprobación de si está dentro de los rangos
	 */
	/* Ya se hace en otra función
	 * if(!rangoAlcohol(maquina_est)){   //Si no está dentro de rango
		maq_estados->medidas_sens->medida_mala;
		imprimirAviso(maquina_est, 5);  //Imprimir aviso de alcohol
	}
	*/

	return mg;
}

//Toma la medida de GSR en estado de reposo como referencia para el cálculo del estres posterior
void calibracionGSR(maq_estados* maquina_est){
	uint8_t i;
	float ref_GSR;

	for(i=0; i<MUESTRAS_CALIB_GSR; i++){
		ref_GSR = ref_GSR + (float)medirGSR();
	}

	ref_GSR = (float)(ref_GSR/(float)MUESTRAS_CALIB_GSR);

	maquina_est->medidas_sens->ref_GSR = ref_GSR;
}

//Toma la medida del pulso en estado de reposo como referencia para el cálculo del estres posterior
void calibracionPulso(maq_estados* maquina_est){
	float refPulso;
	//TO-DO!!!!
	maquina_est->medidas_sens->refPulso = refPulso;
}

/*
//Toma la medida de GSR en estado de reposo como referencia para el cálculo del estres posterior
void calibracionGSR(maq_estados* maquina_est){
	maquina_est->medidas_sens->ref_GSR = medirGSR();
}*/

//Activa todos los sensores excepto el sensor de alcohol y almacena en la estructura los valores detectados
void medirSensores(maq_estados* maquina_est){
	uint32_t valorADC1;   //Lecturas del ADC

	calibracionGSR(maquina_est);
	calibracionPulso(maquina_est);

//	imprimirMedidas();  //Pantalla de medidas de sensores
	medirTemp(maquina_est);
	medirBateria(maquina_est, valorADC1);
	medirEstres(maquina_est);
}

/*
//Saca por pantalla todas las medidas de los sensores
void imprimirMedidas(maq_estados* maquina_est){
	//ES IGUAL QUE IMPRIMIR SENSORES!!!!! CAMBIAR EL NOMBRE DE LA OTRA Y DEJARLA EN PANTALLA.C
}
*/


/*
//Imprime un mensaje por pantalla en función del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si batería baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido
void imprimirAviso(maq_estados* maquina_est, int tipoAviso){
	//LLAMAR A LAS FUNCIONES DE IMPRIMIR SEGÚN EL TIPO DE AVISO Y CAMBIAR ESTA DEFINICIÓN PARA QUE ESTÉ EN PANTALLA.C

}
*/

//Realiza la medición de la temperatura corporal
void medirTemp(maq_estados* maquina_est){}
uint32_t medirTemp1(void){
	uint32_t temp;
	uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
	bufferTemp[0] = 0x01; //Dirección de registro de configuración
	bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuración
	while(HAL_I2C_IsDeviceReady(&hi2c3, 0X90, 2, 10) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 2, 10);

//	while (1){

	  bufferTemp[0] = 0x00; //Dirección de registro de temperatura
	  HAL_I2C_Master_Transmit(&hi2c3, 0x90, bufferTemp, 1, 10);
	  HAL_I2C_Master_Receive(&hi2c3, 0x90, bufferTemp, 2, 10);

	  temp = bufferTemp[0]*256 + bufferTemp[1];  //Primero devuelve el más significativo, como son 8 bits cada registro, para obtener el valor de ambos bytes hay que multiplicar el primero por 256

	 // temp = temp * 0.00390625;  // Porque cada bit no vale 1ºC sino esos grados
//	}
	  //imprimirSecuencia(6);
	 //imprimirAlertaSensor(187, 45, 1);
	//		  imprimirAlertaSensor(temp, 45, 1);
	return temp;
}

//Mide la resistencia de la piel y almacena en la estructura el valor leído
//void medirGSR(maq_estados* maquina_est){
uint32_t medirGSR(void){
	uint32_t valorADC1, valorADC3;

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
	//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
	//medirBateria(maquina_est, valorADC1);

	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
	//valorADC2 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

	HAL_ADC_Stop(&hadc1);

return valorADC3;

}

//Mide el nivel de estrés a partir del pulso y el GSR
void medirEstres(maq_estados* maquina_est){
	//float gsr, pulso;
	uint32_t gsr;
	float pulso, ref_GSR, dif_GSR, refPulso, difPulso;

	gsr = medirGSR();
	pulso = medirPulso();

	ref_GSR = maquina_est->medidas_sens->ref_GSR;
	refPulso = maquina_est->medidas_sens->refPulso;

	dif_GSR = fabsf((float)(gsr - ref_GSR));
	difPulso = fabsf((float)(pulso - refPulso));

	/*
	 * Obtener valor de estrés en función del GSR y el pulso
	 */
	/*if((dif_GSR <= (ref_GSR*MIN_DIF_GSR))&&(difPulso <= (refPulso*MIN_DIF_PULSO))){
		maquina_est->medidas_sens->estres = "Bajo";
	//}else if(((dif_GSR < (ref_GSR*MAX_DIF_GSR))&&(difPulso < (refPulso*MAX_DIF_PULSO)))||(((dif_GSR >= (ref_GSR*MIN_DIF_GSR))&&(dif_GSR < (ref_GSR*MAX_DIF_GSR)))||(difPulso >= (refPulso*MIN_DIF_PULSO))&&(difPulso < (refPulso*MAX_DIF_PULSO)))){
	}else if(((dif_GSR >= (ref_GSR*MIN_DIF_GSR))&&(dif_GSR < (ref_GSR*MAX_DIF_GSR)))||((difPulso >= (refPulso*MIN_DIF_PULSO))&&(difPulso < (refPulso*MAX_DIF_PULSO)))){
		maquina_est->medidas_sens->estres = "Normal";
	}else if((dif_GSR >= (ref_GSR*MAX_DIF_GSR))||(difPulso >= (refPulso*MAX_DIF_PULSO))){
		maquina_est->medidas_sens->estres = "Alto";
	}*/

	if((dif_GSR <= (ref_GSR*MIN_DIF_GSR))&&(difPulso <= (refPulso*MIN_DIF_PULSO))){
		maquina_est->medidas_sens->estres = "Bajo";
	}else if((dif_GSR >= (ref_GSR*MAX_DIF_GSR))||(difPulso >= (refPulso*MAX_DIF_PULSO))){
		maquina_est->medidas_sens->estres = "Alto";
	}else{
		maquina_est->medidas_sens->estres = "Normal";
	}

	//TO-DO PONER MÁRGENES DE PULSOS POR DIFERENCIAS IGUAL QUE GSR!!!
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

//Devuelve si la medida de alcohol está o no dentro del rango
uint8_t rangoAlcohol(maq_estados* maquina_est){

	if(maquina_est->medidas_sens->alcohol >= MAX_ALCOHOL){  //Medida fuera de rango
		maquina_est->medidas_sens->medida_mala = 1;  //Marcar como mala la medida del alcohol
		return 0;
	//}else if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){  //Medida dentro del rango
	}else{  //Medida dentro del rango
		maquina_est->medidas_sens->medida_mala = 0;  //Para saber que ya se comprobó el nivel de alcohol
		return 1;
	}
}

//Devuelve si la medida de estrés está o no dentro del rango
uint8_t rangoEstres(maq_estados* maquina_est){
	if(maquina_est->medidas_sens->estres == MAX_ESTRES){  //Medida fuera de rango
		maquina_est->medidas_sens->medida_mala = 5;  //Marcar como mala la medida del estrés
		return 0;
	}else{  //Medida dentro del rango
		return 1;
	}
}

//Devuelve 1 si alguna medida se sale del rango establecido
uint8_t medida_mal(maq_estados* maquina_est){
	uint8_t tension, pulso, temp, estres;

	if(maquina_est->medidas_sens->medida_mala == 30){   //Aún no se comprobó el alcohol
		/*
		 * Comprobación alcohol
		 */
		return !rangoAlcohol(maquina_est);
	}else{  //Comprobar el resto de medidas

		/*
		 * Comprobación tensión
		 */
		tension = !rangoTension(maquina_est);

		/*
		 * Comprobación pulso
		 */
		pulso = !rangoPulso(maquina_est);

		/*
		 * Comprobación temperatura
		 */
		temp = !rangoTemp(maquina_est);

		/*
		 * Comprobación estrés
		 */
		estres = !rangoEstres(maquina_est);

		if(tension || pulso || temp || estres){
			return 1;
		}else{
			return 0;
		}
	}


	/* if(maquina_est->medidas_sens->alcohol >= MAX_ALCOHOL){
		maquina_est->medidas_sens->medida_mala = 1;  //Marcar como mala la medida del alcohol
		return 1;
	}else if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 0;
	} */

}

//Devuelve 1 si la medida de alcohol se encuentran en el rango establecido
uint8_t medida_bien(maq_estados* maquina_est){

	return rangoAlcohol(maquina_est);

	/*if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 1;
	}else{
		return 0;
	}*/
}


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
