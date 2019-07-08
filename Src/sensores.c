#include "sensores.h"
#include "main.h"
#include "pantalla.h"
#include "adc.h"
#include "i2c.h"
#include "ppg.h"
#include <math.h>
#include <string.h> //Para el strcpy

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
#define MIN_DIF_PULSO 0.4 //Mínima diferencia de valor de pulso con respecto a la referencia para considerar estrés normal
#define MAX_DIF_PULSO 0.8 //Máxima diferencia de valor de pulso con respecto a la referencia para considerar estrés normal
#define MAX_TEMP 37.5  //Valor máximo de temperatura para que salte la alerta
#define MIN_TEMP 34.5  //Valor mínimo de temperatura para que salte la alerta
#define MAX_TENS_SIS 140  //Tensión sistólica a partir de la cual se considera tensión alta
#define MIN_TENS_SIS 90  //Tensión sistólica a partir de la cual se considera tensión baja
#define MAX_TENS_DIAS 90  //Tensión diastólica a partir de la cual se considera tensión alta
#define MIN_TENS_DIAS 60  //Tensión diastólica a partir de la cual se considera tensión baja
#define DIR_TEMP 0X90  //Dirección del sensor de temperatura como esclavo I2C
#define DIR_PPG 0XAE  //Dirección del AFE como esclavo I2C
#define NUM_BYTES_PPG 90 //Número de bytes leídos del AFE (en multi-LED se tienen 3 bytes por cada canal, cada LED)
#define ESPERA_PANTALLA 1000  //Tiempo de espera para que la pantalla se pueda ver (en milisegundos)
#define POT_LEDS_PPG 0x0C //Potencia de LEDs para PPG
#define NUM_MUESTRAS_MEDIA_PPG 4 //Número de muestras con que se hace la media para obtener la señal de PPG
#define MODO_LEDS_PPG 3  //Para decidir cuántos LEDs encender y cuáles de ellos
#define TASA_PPG 400  //Número de muestras por segundo (siendo "muestra" los 3 bytes del conjunto de todos los LEDs)
//#define TASA_PPG 100  //Número de muestras por segundo (siendo "muestra" los 3 bytes del conjunto de todos los LEDs)
#define ANCHO_PULSO_PPG 411  //Ancho de pulso para PPG
#define RANGO_ADC_PPG 4096  //Para decidir cuántos bits de resolución tendrá la lectura
#define AMPLI_ROJO_PPG 0x02  //Amplitud del LED rojo
#define AMPLI_IR_PPG 0x0C  //Amplitud del LED IR
#define AMPLI_VERDE_PPG 0X00  //Amplitud del LED verde
#define RATE_SIZE 4 //Increase this for more averaging. 4 is good
#define NUM_MEDIAS_TENSION 5 //Número de muestras usadas para obtener el valor de tensión arterial media
//#define FACTOR_CORRECCION_TENSION 0.16  //Factor de corrección adaptado a mi tensión (94 medidos en tensiómetro/600 valor leído del PPG sin corregir)
#define FACTOR_CORRECCION_TENSION 0.2  //Factor de corrección adaptado a mi tensión (94 medidos en tensiómetro/600 valor leído del PPG sin corregir)
#define OFFSET_TEMP 1  //Factor de corrección de la temperatura corporal en grados
#define FACTOR_CORRECCION_TEMP 0.00390625  //Factor de corrección de la lectura de temperatura
#define RETARDO_VIBRACION 2000  //Tiempo en milisegundos entre vibración y no vibración
#define NUM_REPETICIONES_VIBRAR 10  //Número de veces que vibra y deja de vibrar el motor
#define MIN_BATERIA 30 //Porcentaje de batería a partir del que se considera batería baja. Con un 30% aproximadamente tiene 3,5V
#define DURACION_ALERTA 2000  //Tiempo en milisegundos que debe esperarse para dejar de mostrar la alerta


//BORRAAAAAAAAAAAAAAAAAAAAAAAAAR!!!!
//const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
/*#define RATE_SIZE 4 //Increase this for more averaging. 4 is good.
uint8_t rates[RATE_SIZE]; //Array of heart rates
uint8_t rateSpot = 0;
int lastBeat = 0; //Time at which the last beat occurred
int beatAvg;*/


//Calibración del sensor del nivel de alcohol
//uint32_t calibracionAlcohol(maq_estados* maquina_est){
float calibracionAlcohol(maq_estados* maquina_est){
//float calibracionAlcohol(void){
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

		medirBateria(maquina_est, valorADC1);

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
void medirAlcohol(maq_estados* maquina_est){
//float medirAlcohol(void){
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

	HAL_GPIO_WritePin(EnableSW_5V_GPIO_Port, EnableSW_5V_Pin, GPIO_PIN_SET);

	imprimirAviso(maquina_est, 0);    //Imprimir bienvenida
	HAL_Delay(ESPERA_PANTALLA);
	imprimirAviso(maquina_est, 1);    //Imprimir que va a calibrar
	//HAL_Delay(ESPERA_PANTALLA);

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_SET);  //Poner a 1 HeaterON_OFF
	HAL_Delay(30000); //Para que caliente el heater y se cargue el condensador (30 seg)
//	HAL_Delay(10000); //Para que caliente el heater y se cargue el condensador (10 seg)
	ro = calibracionAlcohol(maquina_est);  //Calibración del sensor del nivel de alcohol
//	ro = calibracionAlcohol();  //Calibración del sensor del nivel de alcohol
	imprimirAviso(maquina_est, 2);    //Imprimir inicio de medición del nivel de alcohol
	//HAL_Delay(ESPERA_PANTALLA);

	//PARA PRUEBAS!!!
	//imprimirBasico(9);
	//HAL_Delay(3000); //En ms

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
		medirBateria(maquina_est, valorADC1);

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

	HAL_GPIO_WritePin(HeaterON_OFF_GPIO_Port, HeaterON_OFF_Pin, GPIO_PIN_RESET);  //Poner a 0 HeaterON_OFF
	HAL_GPIO_WritePin(EnableSW_5V_GPIO_Port, EnableSW_5V_Pin, GPIO_PIN_RESET);  //Desconectar la alimentación de 5V

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

	//float* tmp;
	//tmp = maquina_est->medidas_sens;

	maquina_est->medidas_sens->alcohol = mg;
	//tmp = mg;

	imprimirAviso(maquina_est, 3); //Imprimir fin de medición del nivel de alcohol
	HAL_Delay(ESPERA_PANTALLA);

	/*
	 * Comprobación de si está dentro de los rangos
	 */
	/* Ya se hace en otra función
	 * if(!rangoAlcohol(maquina_est)){   //Si no está dentro de rango
		maq_estados->medidas_sens->medida_mala;
		imprimirAviso(maquina_est, 5);  //Imprimir aviso de alcohol
	}
	*/

	//return mg;
}

//Toma la medida de GSR en estado de reposo como referencia para el cálculo del estres posterior
void calibracionGSR(maq_estados* maquina_est){
	uint8_t i;
	float ref_GSR;

	for(i=0; i<MUESTRAS_CALIB_GSR; i++){
		ref_GSR = ref_GSR + (float)medirGSR(maquina_est);
	}

	ref_GSR = (float)(ref_GSR/(float)MUESTRAS_CALIB_GSR);

	maquina_est->medidas_sens->ref_GSR = ref_GSR;
}

//Toma la medida del pulso en estado de reposo como referencia para el cálculo del estres posterior
void calibracionPulso(maq_estados* maquina_est){
	/*float refPulso;

	medirTensionPulso(maquina_est);

	refPulso = maquina_est->medidas_sens->pulso;

	maquina_est->medidas_sens->refPulso = refPulso;*/

	/*
	 * Activar 5V y 1,8V
	 */
	HAL_GPIO_WritePin(EnableSW_5V_GPIO_Port, EnableSW_5V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EnableLIN_1V8_GPIO_Port, EnableLIN_1V8_Pin, GPIO_PIN_SET);

	HAL_Delay(250);

	inicializarPPG(POT_LEDS_PPG, NUM_MUESTRAS_MEDIA_PPG, MODO_LEDS_PPG, TASA_PPG, ANCHO_PULSO_PPG, RANGO_ADC_PPG, AMPLI_ROJO_PPG, AMPLI_IR_PPG, AMPLI_VERDE_PPG);


	do{
		medirTensionPulso(maquina_est);
	}while((maquina_est->medidas_sens->pulso < 50)||(maquina_est->medidas_sens->pulso > 300));


	maquina_est->medidas_sens->refPulso = maquina_est->medidas_sens->pulso;

}

/*
//Toma la medida de GSR en estado de reposo como referencia para el cálculo del estres posterior
void calibracionGSR(maq_estados* maquina_est){
	maquina_est->medidas_sens->ref_GSR = medirGSR();
}*/

//Activa todos los sensores excepto el sensor de alcohol y almacena en la estructura los valores detectados
void medirSensores(maq_estados* maquina_est){
	uint32_t valorADC1;   //Lecturas del ADC

	//PARA PRUEBAS!!!
	//imprimirBasico(7);
	//HAL_Delay(ESPERA_PANTALLA);

//	calibracionGSR(maquina_est);
	//calibracionPulso(maquina_est);

	medirTensionPulso(maquina_est);
	medirTemp(maquina_est);
	medirBateria(maquina_est, valorADC1);
	medirEstres(maquina_est);
	//imprimirMedidas(maquina_est);  //Pantalla de medidas de sensores
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
void medirTemp(maq_estados* maquina_est){
//uint32_t medirTemp1(void){
	float temp, temp_ponderada;
	uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
	bufferTemp[0] = 0x01; //Dirección de registro de configuración
	bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuración
	while(HAL_I2C_IsDeviceReady(&hi2c3, DIR_TEMP, 2, 10) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c3, DIR_TEMP, bufferTemp, 2, 10);

//	while (1){

	  bufferTemp[0] = 0x00; //Dirección de registro de temperatura
	  HAL_I2C_Master_Transmit(&hi2c3, DIR_TEMP, bufferTemp, 1, 10);
	  HAL_I2C_Master_Receive(&hi2c3, DIR_TEMP, bufferTemp, 2, 10);  //Sobreescribe lo que hay en el array porque ya no se necesita y mete los datos leidos ahí

	  temp = bufferTemp[0]*256 + bufferTemp[1];  //Primero devuelve el más significativo, como son 8 bits cada registro, para obtener el valor de ambos bytes hay que multiplicar el primero por 256

	 // temp = temp * 0.00390625;  // Porque cada bit no vale 1ºC sino esos grados

	  temp_ponderada = (float)(temp*(float)FACTOR_CORRECCION_TEMP);
	  temp_ponderada = (float)(temp_ponderada + OFFSET_TEMP);

//	}
	  //imprimirSecuencia(6);
	 //imprimirAlertaSensor(187, 45, 1);
	//		  imprimirAlertaSensor(temp, 45, 1);

	 maquina_est->medidas_sens->temperatura = temp_ponderada;
//	 imprimirMedidas(maquina_est);
	//return temp;
}

//Mide la resistencia de la piel y almacena en la estructura el valor leído
//void medirGSR(maq_estados* maquina_est){
uint32_t medirGSR(maq_estados* maquina_est){
	uint32_t valorADC1, valorADC3;

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
	//HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
	medirBateria(maquina_est, valorADC1);

	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol
	//valorADC2 = (float)HAL_ADC_GetValue(&hadc1);  //ADC_IN9, nivel alcohol

	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC3 = HAL_ADC_GetValue(&hadc1);  //ADC_IN10, valor GSR

	HAL_ADC_Stop(&hadc1);

return valorADC3;

}

//Mide la tensión sistólica y diastólica, y el pulso a partir de la señal del PPG
void medirTensionPulso(maq_estados* maquina_est){
//uint8_t medirTensionPulso(void){

	/*uint8_t bufferPPG[NUM_BYTES_PPG];  //Buffer de datos a enviar y leer por I2C
	bufferPPG[0] = 0xFF; //Dirección del registro para modo Multi-LED
	bufferPPG[1] = 0x09; //Dirección del registro para configurar el modo

	//while(HAL_I2C_IsDeviceReady(&hi2c1, DIR_PPG, 2, 10) != HAL_OK);

	//bufferTemp[0] = 0x00; //Dirección de registro de temperatura
	HAL_I2C_Master_Transmit(&hi2c1, DIR_PPG, bufferPPG[0], 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, DIR_PPG, bufferPPG[1], 1, 10);  //Sobreescribe lo que hay en el array porque ya no se necesita y mete los datos leidos ahí

	return bufferPPG[1];*/

	/*uint8_t bufferPPG[NUM_BYTES_PPG];  //Buffer de datos a enviar y leer por I2C
	bufferPPG[0] = 0x11; //Dirección del registro para modo Multi-LED
	bufferPPG[1] = 0x09; //Dirección del registro para configurar el modo

	while(HAL_I2C_IsDeviceReady(&hi2c1, DIR_PPG, 2, 10) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1, DIR_PPG, bufferPPG[0], 2, 10);

//	while (1){

	  //bufferTemp[0] = 0x00; //Dirección de registro de temperatura
	  HAL_I2C_Master_Transmit(&hi2c1, DIR_PPG, bufferPPG[1], 1, 10);
	  HAL_I2C_Master_Receive(&hi2c1, DIR_PPG, bufferPPG, NUM_BYTES_PPG, 10);  //Sobreescribe lo que hay en el array porque ya no se necesita y mete los datos leidos ahí

	  //HACER LA MEDIA DE LAS 3 MUESTRAS RECOGIDAS DE LOS 3 COLORES??? O SEPARAR EN 3 ARRAYS PARA COMPARAR RESULTADOS???
	  calculoTension(maquina_est, bufferPPG);
	  calculoPulso(maquina_est, bufferPPG);*/

	float extremosPPG[2];

	/*
	 * Activar 5V y 1,8V
	 */
	//HAL_GPIO_WritePin(EnableSW_5V_GPIO_Port, EnableSW_5V_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(EnableLIN_1V8_GPIO_Port, EnableLIN_1V8_Pin, GPIO_PIN_SET);

	//inicializarPPG(POT_LEDS_PPG, NUM_MUESTRAS_MEDIA_PPG, MODO_LEDS_PPG, TASA_PPG, ANCHO_PULSO_PPG, RANGO_ADC_PPG, AMPLI_ROJO_PPG, AMPLI_IR_PPG, AMPLI_VERDE_PPG);

	//sense.head = 0;

	calculoPulso(maquina_est, extremosPPG);
	//calculoTension(maquina_est, extremosPPG);

	//imprimirMedidas(maquina_est);
}

//Mide el nivel de estrés a partir del pulso y el GSR
void medirEstres(maq_estados* maquina_est){
	//float gsr, pulso;
	uint32_t gsr;
	float pulso, ref_GSR, dif_GSR, refPulso, difPulso;

	gsr = medirGSR(maquina_est);
//	pulso = medirPulso();
	pulso = maquina_est->medidas_sens->pulso;

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
		strcpy(maquina_est->medidas_sens->estres, "Bajo");
	}else if((dif_GSR >= (ref_GSR*MAX_DIF_GSR))||(difPulso >= (refPulso*MAX_DIF_PULSO))){
		strcpy(maquina_est->medidas_sens->estres, "Alto");
	}else{
		strcpy(maquina_est->medidas_sens->estres, "Normal");
	}
}

//Estima la batería que tiene
void medirBateria(maq_estados* maquina_est, uint32_t valorADC1){
	float tension_bat;   //Valor de tensión del divisor de la batería

	HAL_ADC_Start (&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TIMEOUT_ADC);
	valorADC1 = HAL_ADC_GetValue(&hadc1);  //ADC_IN8, nivel batería
	HAL_ADC_Stop(&hadc1);

	tension_bat = 2*(valorADC1/ADC_MAX)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensión Vin_bat
	maquina_est->nivel_bateria = (uint16_t)(((tension_bat - V_BAT_MIN)/(V_BAT_MAX - V_BAT_MIN))*100);  //Sacar el porcentaje de batería que le queda (estimación en tensión) en uint porque no tenemos precisión

}

/*//Inicialización del AFE de PPG
void inicializarPPG(void){

	//void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
	MAX30101_setupPPG(0x0C, 4, 3, 400, 411, 4096);  //3 LEDs, 400 samples/second y 411 de ancho de pulso. Se bajó la potencia para adaptarlo a pieles claras
	MAX30101_setPulseAmplitudeRed(0x02); //Turn Red LED to low to indicate sensor is running
	MAX30101_setPulseAmplitudeGreen(0x00); //Turn off Green LED

}*/

//Inicialización del AFE de PPG
void inicializarPPG(uint8_t potenciaLED, uint8_t mediaMuestras, uint8_t modoLEDs, int tasaMuestreo, int anchoPulso, int rangoADC, uint8_t amplitudRojo, uint8_t amplitudIR, uint8_t amplitudVerde){

	//void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
	MAX30101_setupPPG(potenciaLED, mediaMuestras, modoLEDs, tasaMuestreo, anchoPulso, rangoADC);  //3 LEDs a 0x0C de potencia, 400 samples/second, 411 de ancho de pulso y 18 bits. Se bajó la potencia para adaptarlo a pieles claras
	MAX30101_setPulseAmplitudeRed(amplitudRojo); //Turn Red LED to low to indicate sensor is running
	MAX30101_setPulseAmplitudeIR(amplitudIR); //Turn off IR LED
	MAX30101_setPulseAmplitudeGreen(amplitudVerde); //Turn off Green LED

}

//Realiza el cálculo del valor de tensión en función de la lectura del AFE
//void calculoTension(maq_estados* maquina_est, uint8_t bufferPPG){
void calculoTension(maq_estados* maquina_est, float extremosPPG[2]){
	float amplitudPPG;
	uint8_t i;
	static uint8_t indice = 0;
	static float media_tension, mediaPonderada;
	static float amplitudes[NUM_MEDIAS_TENSION]; //Array de amplitudes

	amplitudPPG = extremosPPG[0] - extremosPPG[1];



	amplitudes[indice++] = (float)amplitudPPG;  //Para ir rellenando el array de amplitudes
	indice %= NUM_MEDIAS_TENSION;  //Para que cuando llegue al final, vuelva a ponerse a cero

	media_tension = 0;
	for(i = 0 ; i < NUM_MEDIAS_TENSION ; i++){
		media_tension += amplitudes[i];
	}
	media_tension /= NUM_MEDIAS_TENSION;

	mediaPonderada = media_tension*FACTOR_CORRECCION_TENSION;

	maquina_est->medidas_sens->tension_sis = mediaPonderada;

	//SOLO PARA PRUEBAS!!!
	//imprimirPruebas(2, amplitudPPG, 0);
}

//Realiza el cálculo del valor de pulso en función de la lectura del AFE
//void calculoPulso(maq_estados* maquina_est, uint8_t bufferPPG[NUM_BYTES_PPG]){
void calculoPulso(maq_estados* maquina_est, float extremosPPG[2]){

	/*uint8_t min1, min2, min3;  //Valores mínimos de la curva devuelta por el AFE
	uint8_t i;

	min1 = 0;
	min2 = 0;
	min3 = 0;

	for(i=0; i < NUM_BYTES_PPG; i++){    //CAMBIAR EL LÍMITE DEL BUCLE SI SE DIVIDE EN 3 EL ARRAY O SI SE HACE LA MEDIA DE LOS LEDS
		if(bufferPPG[i] < min1){
			min1 = bufferPPG[i];
		}
	}*/


	//static const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
	static uint8_t rates[RATE_SIZE]; //Array of heart rates
	static uint8_t rateSpot = 0;
	static int lastBeat = 0; //Time at which the last beat occurred
	static int beatAvg;


	float beatsPerMinute;


	beatsPerMinute = 0;

	int irValue = MAX30101_getIR();  //Provoca una nueva recogida de datos de todos los LEDs que se hayan configurado como encendidos. El IR es el que mejor coge el pulso

	//printf("IR: %d\n", irValue); //OJO, sin \n no imprime nada

	/*
	 * Para medir la frecuencia con que se leen los sensores con el osciloscopio
	 */
	//HAL_GPIO_TogglePin(TestPoint_output2_GPIO_Port, TestPoint_output2_Pin);

	uint8_t beat = MAX30101_checkForBeat(irValue, extremosPPG);
	if(beat == 1){
		//We sensed a beat!
		int delta = HAL_GetTick() - lastBeat;
		lastBeat = HAL_GetTick();

		beatsPerMinute = 60 / (delta / 1000.0);

		if(beatsPerMinute < 255 && beatsPerMinute > 20){
		  rates[rateSpot++] = (uint8_t)beatsPerMinute; //Store this reading in the array
		  rateSpot %= RATE_SIZE; //Wrap variable

		  //Take average of readings
		  beatAvg = 0;
		  for(uint8_t x = 0 ; x < RATE_SIZE ; x++){
			beatAvg += rates[x];
		  }
		  beatAvg /= RATE_SIZE;
		}

	//	imprimirLatidos(irValue, beatsPerMinute, beatAvg);

		maquina_est->medidas_sens->medida_mala = 10; //Para garantizar que no da una medida mala constante porque no detectó pulso

		maquina_est->medidas_sens->pulso = beatAvg;
		calculoTension(maquina_est, extremosPPG);
//PARA PRUEBAS!!!!!!
		//imprimirPruebas(2, maquina_est->medidas_sens->pulso, 0);
		//imprimirPruebas(2, beatAvg, 0);
		  //HAL_Delay(2000); //En ms
		//imprimirPruebas(10, maquina_est->medidas_sens->pulso, maquina_est->medidas_sens->tension_sis);
		imprimirMedidas(maquina_est);
	}
}

//Devuelve si la medida de alcohol está o no dentro del rango
uint8_t rangoAlcohol(maq_estados* maquina_est){
	//PARA PRUEBAS!!!
	//imprimirBasico(7);
	//HAL_Delay(ESPERA_PANTALLA);
	if(maquina_est->medidas_sens->alcohol >= MAX_ALCOHOL){  //Medida fuera de rango
		maquina_est->medidas_sens->medida_mala = 1;  //Marcar como mala la medida del alcohol
		//PARA PRUEBAS!!!
			//imprimirBasico(4);
			//HAL_Delay(ESPERA_PANTALLA);
		return 0;
	//}else if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){  //Medida dentro del rango
	}else{  //Medida dentro del rango
		//PARA PRUEBAS!!!
			//imprimirBasico(8);
			//HAL_Delay(ESPERA_PANTALLA);
		//HAL_GPIO_WritePin(LED_test_GPIO_Port, LED_test_Pin, GPIO_PIN_RESET);
		//HAL_Delay(ESPERA_PANTALLA);
		maquina_est->medidas_sens->medida_mala = 0;  //Para saber que ya se comprobó el nivel de alcohol
		return 1;
	}
}

//Devuelve si la medida de estrés está o no dentro del rango
uint8_t rangoEstres(maq_estados* maquina_est){
	//if(maquina_est->medidas_sens->estres == MAX_ESTRES){  //Medida fuera de rango
	if((strcmp(maquina_est->medidas_sens->estres, MAX_ESTRES) == 0)&&(maquina_est->medidas_sens->medida_mala != 5)){  //Medida fuera de rango. Comparar ambos strings
		maquina_est->medidas_sens->medida_mala = 5;  //Marcar como mala la medida del estrés
		return 0;
	}else{  //Medida dentro del rango
		return 1;
	}
}

//Devuelve si la medida del pulso está o no dentro del rango
uint8_t rangoPulso(maq_estados* maquina_est){
	float pulso;
	float dif_pulso, ref_pulso;

	pulso = maquina_est->medidas_sens->pulso;
	ref_pulso = maquina_est->medidas_sens->refPulso;

	dif_pulso = fabsf((float)((pulso-ref_pulso)/ref_pulso));

	//if((dif_pulso >= MAX_DIF_PULSO)||(dif_pulso <= MIN_DIF_PULSO)){  //Medida fuera de rango
	//if((dif_pulso >= MAX_DIF_PULSO)){  //Medida fuera de rango
	if((dif_pulso >= MAX_DIF_PULSO)&&(maquina_est->medidas_sens->medida_mala != 3)){  //Medida fuera de rango y la última alerta no fue del pulso
		maquina_est->medidas_sens->medida_mala = 3;  //Marcar como mala la medida de pulso
		return 0;
	}else{  //Medida dentro del rango
		return 1;
	}
}

//Devuelve si la medida de la tensión está o no dentro del rango
uint8_t rangoTension(maq_estados* maquina_est){
	/*uint32_t tension_sis, tension_dias;

	tension_sis = maquina_est->medidas_sens->tension_sis;
	tension_dias = maquina_est->medidas_sens->tension_dia;

	if((tension_sis >= MAX_TENS_SIS)||(tension_sis <= MIN_TENS_SIS)||(tension_dias <= MAX_TENS_DIAS)||(tension_dias <= MIN_TENS_DIAS)){  //Medida fuera de rango
		maquina_est->medidas_sens->medida_mala = 2;  //Marcar como mala la medida de tensión
		return 0;
	}else{  //Medida dentro del rango
		return 1;
	}*/

	uint32_t tension_sis;

	tension_sis = maquina_est->medidas_sens->tension_sis;

	if((tension_sis >= MAX_TENS_SIS)||(tension_sis <= MIN_TENS_SIS)){  //Medida fuera de rango
		maquina_est->medidas_sens->medida_mala = 2;  //Marcar como mala la medida de tensión
		return 0;
	}else{  //Medida dentro del rango
		return 1;
	}
}

//Devuelve si la medida de temperatura está o no dentro del rango
uint8_t rangoTemp(maq_estados* maquina_est){
	if((maquina_est->medidas_sens->temperatura >= MAX_TEMP)||(maquina_est->medidas_sens->temperatura <= MIN_TEMP)){  //Medida fuera de rango
		maquina_est->medidas_sens->medida_mala = 4;  //Marcar como mala la medida de temperatura
		return 0;
	}else{  //Medida dentro del rango
		return 1;
	}
}

//Devuelve 1 si alguna medida se sale del rango establecido
uint8_t medida_mal_alcohol(maq_estados* maquina_est){
	uint8_t tension, pulso, temp, estres;
	//PARA PRUEBAS
	//imprimirBasico(7);
	//HAL_Delay(1000); //En ms

	//PARA PRUEBAS
	//imprimirBasico(8);
	//HAL_Delay(1000); //En ms
	medirAlcohol(maquina_est);
	/*
	 * Comprobación alcohol
	 */
	return !rangoAlcohol(maquina_est);
}

//Devuelve 1 si alguna medida se sale del rango establecido
uint8_t medida_mal(maq_estados* maquina_est){
	uint8_t tension, pulso, temp, estres;
	//PARA PRUEBAS
	//imprimirBasico(7);
	//HAL_Delay(1000); //En ms

//	if(maquina_est->medidas_sens->medida_mala == 30){   //Aún no se comprobó el alcohol
		//PARA PRUEBAS
//		imprimirBasico(8);
//		HAL_Delay(1000); //En ms
//		medirAlcohol(maquina_est);
		/*
		 * Comprobación alcohol
		 */
//		return !rangoAlcohol(maquina_est);
//	}else{  //Comprobar el resto de medidas

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
//	}


	/* if(maquina_est->medidas_sens->alcohol >= MAX_ALCOHOL){
		maquina_est->medidas_sens->medida_mala = 1;  //Marcar como mala la medida del alcohol
		return 1;
	}else if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 0;
	} */

}

//Devuelve 1 si la medida de alcohol se encuentran en el rango establecido
uint8_t medida_bien(maq_estados* maquina_est){
	//PARA PRUEBAS!!!
		//imprimirBasico(8);
		//HAL_Delay(ESPERA_PANTALLA);
	//HAL_GPIO_TogglePin(LED_test_GPIO_Port, LED_test_Pin);
	//HAL_GPIO_WritePin(LED_test_GPIO_Port, LED_test_Pin, GPIO_PIN_SET);
	//return rangoAlcohol(maquina_est);
	if(rangoAlcohol(maquina_est)){
		calibracionGSR(maquina_est);
		calibracionPulso(maquina_est);
		return 1;
	}else{
		return 0;
	}

	/*if(maquina_est->medidas_sens->alcohol < MAX_ALCOHOL){
		return 1;
	}else{
		return 0;
	}*/
}


//Activa el motor para alertar mediante vibración
void vibracion(void){

	int i;

	/*for(i = 0; i < NUM_REPETICIONES_VIBRAR; i++){
		HAL_GPIO_WritePin(MotorON_OFF_GPIO_Port, MotorON_OFF_Pin, GPIO_PIN_SET);
		HAL_Delay(RETARDO_VIBRACION);
		HAL_GPIO_WritePin(MotorON_OFF_GPIO_Port, MotorON_OFF_Pin, GPIO_PIN_RESET);
	}*/

	//HAL_GPIO_WritePin(MotorON_OFF_GPIO_Port, MotorON_OFF_Pin, GPIO_PIN_SET);
	//HAL_Delay(RETARDO_VIBRACION);

	for(i = 0; i < NUM_REPETICIONES_VIBRAR; i++){
		HAL_GPIO_TogglePin(MotorON_OFF_GPIO_Port, MotorON_OFF_Pin);
		HAL_Delay(RETARDO_VIBRACION);

	}

	HAL_GPIO_WritePin(MotorON_OFF_GPIO_Port, MotorON_OFF_Pin, GPIO_PIN_RESET); //Para asegurar que no se quede vibrando
}

//Imprime por pantalla el parámetro por el cual se realiza la alerta y activa la vibración
void imprimirYvibrar(maq_estados* maquina_est){
	//LLAMAR A LA FUNCIÓN DE ALERTA DE PANTALLA.C Y A VIBRACION()

	imprimirAviso(maquina_est, 5);
	vibracion();
	//imprimirAlertaSensor(maquina_est->medidas_sens, maquina_est->medidas_sens->medida_mala);
	//imprimirAviso(maquina_est, 5);
}


uint8_t bat_baja(maq_estados* maquina_est){

	return (maquina_est->nivel_bateria <= MIN_BATERIA);
}  //Devuelve 1 si el nivel de la batería se encuentra por debajo del umbral establecido

//Devuelve 1 si se ha puesto a cargar el sistema
uint8_t conectado(maq_estados* maquina_est){
	return !HAL_GPIO_ReadPin(TestPoint_input_GPIO_Port, TestPoint_input_Pin);
}

//Devuelve 1 si dio la alerta tras una medida fuera de rango
uint8_t alerta_medir_dada(maq_estados* maquina_est){
	if(maquina_est->medidas_sens->medida_mala == 1){  //Para saber si la alerta fue por el alcohol
		return 0;
	}
	return alerta_dada(maquina_est);
}

//Devuelve 1 si dio la alerta tras una medida de nivel de alcohol fuera de rango
uint8_t alerta_alcohol_dada(maq_estados* maquina_est){
	return alerta_dada(maquina_est);
}

//Devuelve 0 si se ha dejado de cargar el sistema
uint8_t desconectado(maq_estados* maquina_est){
	return !conectado(maquina_est);
}

uint8_t sistema_ON(maq_estados* maquina_est){
	return 1; //Si no está encendido la máquina de estados no estaría funcionando
}

uint8_t alerta_dada(maq_estados* maquina_est){
	uint32_t t_inicial, t_final;
	t_inicial = HAL_GetTick();
	t_final = t_inicial;  //Para que al principio tenga un valor

	while((t_final - t_inicial) < DURACION_ALERTA){  //Para obligar a que la alerta dure un tiempo
		t_final = HAL_GetTick();
	}

	return 1;
}
