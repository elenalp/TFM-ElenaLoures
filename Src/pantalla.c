#include "pantalla.h"
#include "i2c.h"
#include "maquinaEstados.h"

#define REPETICIONES 5  //Número de veces que parpadean los iconos (ciclos de parpadeo)
#define LIMITE_ON 400  //Valor de tiempo en el que deja de aparecer el icono
#define LIMITE_OFF 600  //Valor de tiempo en el que deja de estar oculto el icono
#define DIM_RECTANGULO 32  //Dimensiones del rectángulo que oculta los iconos
#define LIMITE_PRIMERO 400  //Valor de tiempo en el que deja de aparecer el icono1
#define LIMITE_SEGUNDO 800  //Valor de tiempo en el que deja de aparecer el icono2
#define LIMITE_TERCERO 1200  //Valor de tiempo en el que deja de aparecer el icono3

/*
 * Funciones para escribir mediante I2C en la pantalla
 */
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr){
	return 1;
}

uint8_t u8x8_byte_my_hw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr){
	static uint8_t buff[32];
	static uint8_t length=0;
	uint8_t *data = arg_ptr;

	switch (msg){
	  case U8X8_MSG_BYTE_SEND:

		  for(int i=0; i<arg_int; i++){
			buff[length] = data[i];
			length++;
		  }
		  break;
	  case U8X8_MSG_BYTE_INIT:
		  break;
	  case U8X8_MSG_BYTE_SET_DC:
		  break;
	  case U8X8_MSG_BYTE_START_TRANSFER:
		  length = 0;
		  break;
	  case U8X8_MSG_BYTE_END_TRANSFER:
		  //no parece necesario while(HAL_I2C_GetState (&hi2c1) != HAL_I2C_STATE_READY)
		  HAL_I2C_Master_Transmit(&hi2c3, (0x3C <<1), buff, length, 10);
		  break;
	  default:
		  return 0;
	}
	return 1;
}

//Ejecutar las funciones neesarias para comunicarse por I2C con la pantalla
void inicializarPantalla(void){
	//Para controlador ssd1306
	//u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_my_hw_i2c, u8x8_stm32_gpio_and_delay);

	//Para controlador sh1106 (el de la pantalla que se usa)
	u8g2_Setup_sh1106_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_my_hw_i2c, u8x8_stm32_gpio_and_delay);

	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

}

/*
 * Funciones con diferentes pantallas posibles a imprimir
 * Coordenadas: x (horizontal) [0, 127]; y (vertical) [0, 64]
 * (0,0) en la esquina superior izquierda
 * El parámetro "y" es el borde inferior de los caracteres
 * Fuentes que se leen bien:
 * 		u8g2_font_calibration_gothic_nbp_tf   Bonita y se lee pero grande (tiene tildes)
 * 		u8g2_font_crox3h_tf   Pequeña y se lee pero no muy bonita (sin tildes)
 * 		u8g2_font_6x12_mf   Muy pequeña y con tildes
 * 		u8g2_font_7x14_tf   Pequeña y con tildes
 * 		u8g2_font_profont12_tf    Muy pequeña y con tildes
 * 		u8g2_font_helvB08_tf    Pequeña y gruesa (con tildes)
 * 		u8g2_font_open_iconic_all_4x_t   Iconos grandes
 * 		u8g2_font_open_iconic_all_2x_t   Iconos pequeños
 * Iconos: https://github.com/olikraus/u8g2/wiki/fntgrpiconic
 * Los textos tienen el origen en su parte inferior
 * Los rectángulos tienen el origen en la esquina superior izquierda
 */

/*
 * Pantalla en la que solo aparece texto e icono de forma estática
 * (pantallas 4, 7 y 9)
 */
void imprimirBasico(uint8_t caso){

	char  textoSup[16], textoInf[16];
	uint16_t icono;   //Código ASCII en decimal del icono en la fuente
	u8g2_uint_t x_sup, x_inf, x_ic, y_sup, y_inf, y_ic;
	const uint8_t *fuente_sup;  //Fuente usada para la primera línea
	const uint8_t *fuente_inf;  //Fuente usada para la segunda línea
	const uint8_t *fuente_ic;  //Fuente usada para el icono

	switch(caso){
		case 4:  //Es el número de pantalla al que corresponde
			strcpy(textoSup, "BIENVENIDO");  //Para asignar "BIENVENIDO" a "textoSup"
			strcpy(textoInf, " ");
			icono = 259;  //Sol
			x_sup = 20;  //Valores obtenidos mirando cómo queda (ensayo y error)
			x_inf = 0;
			x_ic = 45;
			y_sup = 20;
			y_inf = 30;
			y_ic = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
		case 7:
			strcpy(textoSup, "APÁGAME");
			strcpy(textoInf, "Por favor");
			icono = 280;  //Señal de precaución
			x_sup = 25;
			x_inf = 28;
			x_ic = 55;
			y_sup = 20;
			y_inf = 40;
			y_ic = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_crox3h_tf;
			fuente_ic = u8g2_font_open_iconic_all_2x_t;
			break;
		case 9:
			strcpy(textoSup, "Fin de la");
			strcpy(textoInf, "medición");
			icono = 268;  //Pulgar arriba
			x_sup = 15;
			x_inf = 15;
			x_ic = 90;
			y_sup = 20;
			y_inf = 42;
			y_ic = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
		default:
			strcpy(textoSup, "Algo falla");
			strcpy(textoInf, " ");
			icono = 121;  //Cruz en círculo relleno
			x_sup = 25;
			x_inf = 5;
			x_ic = 47;
			y_sup = 20;
			y_inf = 40;
			y_ic = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
	}

	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetFont(&u8g2, fuente_sup);  //Fuente para el texto
		u8g2_DrawStr(&u8g2, x_sup, y_sup, textoSup);
		u8g2_SetFont(&u8g2, fuente_inf);  //Fuente para el texto
		u8g2_DrawStr(&u8g2, x_inf, y_inf, textoInf);
		u8g2_SetFont(&u8g2, fuente_ic);  //Fuente para el icono
		u8g2_DrawGlyph(&u8g2, x_ic, y_ic, icono);
	}while(u8g2_NextPage(&u8g2));
}


/*
 * Pantalla en la que aparece un texto y un icono con parpadeo
 * (pantallas 3 y 8)
 */
void parpadeoSimple(uint8_t caso, uint8_t encendido){

	char  textoSup[16], textoInf[16];
	uint16_t icono;   //Código ASCII en decimal del icono en la fuente
	u8g2_uint_t x_sup, x_inf, x_ic, y_sup, y_inf, y_ic;
	const uint8_t *fuente_sup;  //Fuente usada para la primera línea
	const uint8_t *fuente_inf;  //Fuente usada para la segunda línea
	const uint8_t *fuente_ic;  //Fuente usada para el icono


	switch(caso){
		case 3:  //Es el número de pantalla al que corresponde
			strcpy(textoSup, "BATERÍA BAJA");
			strcpy(textoInf, "");
			icono = 90;  //Batería vacía
			x_sup = 10;
			x_inf = 0;
			x_ic = 45;
			y_sup = 20;
			y_inf = 30;
			y_ic = 60;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
		case 8:
			strcpy(textoSup, "Midiendo nivel");
			strcpy(textoInf, "de alcohol");
			icono = 269;  //Cronómetro
			x_sup = 10;
			x_inf = 10;
			x_ic = 95;
			y_sup = 20;
			y_inf = 42;
			y_ic = 64;
			fuente_sup = u8g2_font_crox3h_tf;
			fuente_inf = u8g2_font_crox3h_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
		default:
			strcpy(textoSup, "Algo falla");
			strcpy(textoInf, " ");
			icono = 121;  //Cruz en círculo relleno
			x_sup = 25;
			x_inf = 5;
			x_ic = 47;
			y_sup = 20;
			y_inf = 40;
			y_ic = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
	}

	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetDrawColor(&u8g2, 1);  //Para poner las letras en color (deshace la configuración de borrar el icono)
		u8g2_SetFont(&u8g2, fuente_sup);  //Fuente para el texto
		u8g2_DrawStr(&u8g2, x_sup, y_sup, textoSup);
		u8g2_SetFont(&u8g2, fuente_inf);  //Fuente para el texto
		u8g2_DrawStr(&u8g2, x_inf, y_inf, textoInf);
		u8g2_SetFont(&u8g2, fuente_ic);  //Fuente para el icono


		if(!encendido){
			u8g2_SetDrawColor(&u8g2, 0);  //Para poner el color del fondo al rectángulo y así borrar el icono
			u8g2_DrawBox(&u8g2, x_ic, y_ic-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
		}else{
			u8g2_DrawGlyph(&u8g2, x_ic, y_ic, icono);
		}
	}while (u8g2_NextPage(&u8g2));
}

/*
 * Pantalla en la que aparece un texto y varios iconos parpadeando por orden
 * (pantallas 5 y 6)
 */
void parpadeoSecuencia(uint8_t caso, uint8_t icono_mostrado){

	char  textoSup[16], textoInf[16];
	uint16_t icono1, icono2, icono3;   //Código ASCII en decimal de los iconos en la fuente
	u8g2_uint_t x_sup, x_inf, x_ic1, x_ic2, x_ic3, y_sup, y_inf, y_ic1, y_ic2, y_ic3;
	const uint8_t *fuente_sup;  //Fuente usada para la primera línea
	const uint8_t *fuente_inf;  //Fuente usada para la segunda línea
	const uint8_t *fuente_ic;  //Fuente usada para el icono


	switch(caso){
		case 5:  //Es el número de pantalla al que corresponde
			strcpy(textoSup, "Calibrando");
			strcpy(textoInf, "alcoholímetro");
			icono1 = 129;  //Rueda dentada
			icono2 = 129;  //Rueda dentada
			icono3 = 129;  //Rueda dentada
			x_sup = 23;
			x_inf = 13;
			x_ic1 = 10;
			x_ic2 = 50;
			x_ic3 = 90;
			y_sup = 14;
			y_inf = 30;
			y_ic1 = 64;
			y_ic2 = 64;
			y_ic3 = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;

		case 6:
			strcpy(textoSup, "Cargando");
			strcpy(textoInf, "");
			icono1 = 90;  //Batería vacía
			icono2 = 81;  //Flecha hacia la derecha
			icono3 = 91;  //Batería llena
			x_sup = 28;
			x_inf = 33;
			x_ic1 = 10;
			x_ic2 = 50;
			x_ic3 = 90;
			y_sup = 20;
			y_inf = 30;
			y_ic1 = 64;
			y_ic2 = 64;
			y_ic3 = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
		default:
			strcpy(textoSup, "Algo falla");
			strcpy(textoInf, " ");
			icono1 = 121;  //Cruz en círculo relleno
			icono2 = 121;  //Cruz en círculo relleno
			icono3 = 121;  //Cruz en círculo relleno
			x_sup = 28;
			x_inf = 33;
			x_ic1 = 10;
			x_ic2 = 50;
			x_ic3 = 90;
			y_sup = 20;
			y_inf = 30;
			y_ic1 = 64;
			y_ic2 = 64;
			y_ic3 = 64;
			fuente_sup = u8g2_font_calibration_gothic_nbp_tf;
			fuente_inf = u8g2_font_calibration_gothic_nbp_tf;
			fuente_ic = u8g2_font_open_iconic_all_4x_t;
			break;
	}

	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetDrawColor(&u8g2, 1);  //Para poner las letras en color (deshace la configuración de borrar el icono)
		u8g2_SetFont(&u8g2, fuente_sup);  //Fuente para el texto
		u8g2_DrawStr(&u8g2, x_sup, y_sup, textoSup);
		u8g2_SetFont(&u8g2, fuente_inf);  //Fuente para el texto
		u8g2_DrawStr(&u8g2, x_inf, y_inf, textoInf);
		u8g2_SetFont(&u8g2, fuente_ic);  //Fuente para los iconos

		if(icono_mostrado == 1){  //Dibujar icono1 y borrar el 2 y 3
			u8g2_DrawGlyph(&u8g2, x_ic1, y_ic1, icono1);

			u8g2_SetDrawColor(&u8g2, 0);  //Para poner el color del fondo al rectángulo y así borrar el icono
			u8g2_DrawBox(&u8g2, x_ic2, y_ic2-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
			u8g2_DrawBox(&u8g2, x_ic3, y_ic3-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
		}else{
			if(icono_mostrado == 2){  //Dibujar icono2 y borrar el 1 y 3
				u8g2_DrawGlyph(&u8g2, x_ic2, y_ic2, icono2);

				u8g2_SetDrawColor(&u8g2, 0);  //Para poner el color del fondo al rectángulo y así borrar el icono
				u8g2_DrawBox(&u8g2, x_ic1, y_ic1-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
				u8g2_DrawBox(&u8g2, x_ic3, y_ic3-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
			}else{
				if(icono_mostrado == 3){  //Dibujar icono3 y borrar el 1 y 2
					u8g2_DrawGlyph(&u8g2, x_ic3, y_ic3, icono3);

					u8g2_SetDrawColor(&u8g2, 0);  //Para poner el color del fondo al rectángulo y así borrar el icono
					u8g2_DrawBox(&u8g2, x_ic1, y_ic1-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
					u8g2_DrawBox(&u8g2, x_ic2, y_ic2-DIM_RECTANGULO, DIM_RECTANGULO, DIM_RECTANGULO);
				}
			}

		}

	}while (u8g2_NextPage(&u8g2));

}

/*
 * Saca por pantalla todas las medidas de los sensores
 * Pantalla en la que se muestran todas las medidas de todos los sensores
 */
//void imprimirSensores(maq_estados* maquina_est){
void imprimirMedidas(maq_estados* maquina_est){
	char  alcohol[16], tension_sis[16], tension_dia[16], pulso[16], temperatura[16], estres[16], tension[64];
	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetFont(&u8g2, u8g2_font_helvB08_tf);  //Fuente para nombres de parámetros
		u8g2_DrawStr(&u8g2, 40, 10, "MEDIDAS");
		u8g2_DrawStr(&u8g2, 0, 20, "Alcohol:");
		u8g2_DrawStr(&u8g2, 0, 31, "Tensión:");
		u8g2_DrawStr(&u8g2, 0, 42, "Pulso:");
		u8g2_DrawStr(&u8g2, 0, 54, "Tª:");
		u8g2_DrawStr(&u8g2, 0, 64, "Estrés:");

		//Pasar a char los valores recogidos por los sensores
		sprintf(alcohol, "%lu", maquina_est->medidas_sens->alcohol);  //Hacer cast de int a char
		sprintf(tension_sis, "%lu", maquina_est->medidas_sens->tension_sis);  //Hacer cast de int a char
		sprintf(tension_dia, "%lu", maquina_est->medidas_sens->tension_dia);  //Hacer cast de int a char
		sprintf(pulso, "%lu", maquina_est->medidas_sens->pulso);  //Hacer cast de int a char
		sprintf(temperatura, "%lu", maquina_est->medidas_sens->temperatura);  //Hacer cast de int a char

		//Representar las tensiones como diastólica/sistólica
		strcat(tension, tension_sis); //Para concatenar 2 strings
		strcat(tension, "/"); //Para concatenar 2 strings
		strcat(tension, tension_dia); //Para concatenar 2 strings


		u8g2_SetFont(&u8g2, u8g2_font_profont12_tf);  //Fuente para nombres de parámetros
		u8g2_DrawStr(&u8g2, 60, 20, "Valor1");
		u8g2_DrawStr(&u8g2, 60, 31, "Valor2a/Valor2b");
		u8g2_DrawStr(&u8g2, 60, 42, "Valor3");
		u8g2_DrawStr(&u8g2, 60, 54, "Valor4");
		u8g2_DrawStr(&u8g2, 60, 64, "Valor5");

		u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);  //Fuente para los iconos
		u8g2_DrawGlyph(&u8g2, 93, 14, 238);  //Electrocardiograma
		u8g2_DrawGlyph(&u8g2, 110, 64, 183);  //Corazón
	}while(u8g2_NextPage(&u8g2));

	 //TO-DO!!!

}

/*
 * Pantalla en la que se muestra una alerta por la medida de algún sensor
 * Se pasa como parámetro el número de orden dentro de la estructura de medidas que provocó la alerta
 */
/*//void imprimirAlertaSensor(medidasSensores* med_sensores, uint8_t sensor){
void imprimirAlertaSensor(medidasSensores* med_sensores){

	char  parametro[16], valor[16], unidades[16];
	u8g2_uint_t x_param, x_valor, x_uds;

	switch(med_sensores->medida_mala){
		case 1:  //Es el primer parámetro de la estructura, el alcohol
			x_param = 5;
			x_valor = 20;
			x_uds = 35;
			strcpy(parametro, "Alcohol: ");
			//strcpy(valor, (char) med_sensores->alcohol);
			sprintf(valor, "%lu", med_sensores->alcohol);  //Hacer cast de uint32_t (es un long unsigned int) a char
			strcpy(unidades, "mg/l");
			break;
		case 2:  //Tensión
			x_param = 5;
			x_valor = 20;
			x_uds = 35;

			//Representar las tensiones como diastólica/sistólica
			strcat(valor, med_sensores->tension_sis); //Para concatenar 2 strings
			strcat(valor, "/"); //Para concatenar 2 strings
			strcat(valor, med_sensores->tension_dia); //Para concatenar 2 strings

			strcpy(parametro, "Tensión: ");  //HACER CONCATENACIÓN DE STRINGS!!!!!
			//strcpy(valor, (char) med_sensores->tension);
			sprintf(valor, "%lu", tension);  //Hacer cast de uint32_t (es un long unsigned int) a char
			strcpy(unidades, "mmHg");
			break;
		case 3:  //Pulso
			x_param = 5;
			x_valor = 20;
			x_uds = 35;
			strcpy(parametro, "Pulso: ");
			//strcpy(valor, (char) med_sensores->pulso);
			sprintf(valor, "%lu", med_sensores->pulso);  //Hacer cast de uint32_t (es un long unsigned int) a char
			strcpy(unidades, "pul/min");
			break;
		case 4:  //Temperatura
			x_param = 5;
			x_valor = 20;
			x_uds = 35;
			strcpy(parametro, "Temp: ");
			//strcpy(valor, (char) med_sensores->temperatura);
			sprintf(valor, "%lu", med_sensores->temperatura);  //Hacer cast de uint32_t (es un long unsigned int) a char
			strcpy(unidades, "ºC");
			break;
		case 5:  //Estrés
			x_param = 5;
			x_valor = 20;
			x_uds = 35;
			strcpy(parametro, "Estrés: ");
			strcpy(valor, med_sensores->estres);  //No es necesario cast porque ya es char
			strcpy(unidades, "");
			break;
		default:
			x_param = 5;
			x_valor = 20;
			x_uds = 35;
			strcpy(parametro, "Error");
			strcpy(valor, "Algo fue mal");
			strcpy(unidades, "");
			break;
	}

	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetFont(&u8g2, u8g2_font_helvB08_tf);  //Fuente para el texto superior
		u8g2_DrawStr(&u8g2, 30, 14, "PRECAUCIÓN");
		u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);  //Fuente para el texto inferior
		u8g2_DrawStr(&u8g2, x_param, 64, parametro);
		u8g2_DrawStr(&u8g2, x_valor, 64, valor);
		u8g2_DrawStr(&u8g2, x_uds, 64, unidades);
		u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_4x_t);  //Fuente para el icono
		u8g2_DrawGlyph(&u8g2, 50, 20, 280);  //Señal de precaución
	}while(u8g2_NextPage(&u8g2));

}*/

//BORRAAAAAAAAAAAAR!!!!
void imprimirAlertaSensor(uint32_t med_sensores1, uint32_t med_sensores2 , uint8_t sensor){

	char  parametro[16], valor[16], unidades[16], tension_sis[16], tension_dia[16];
	u8g2_uint_t x_param, x_valor, x_uds;
	float medida_float;

	//sprintf(valor, "%lu", med_sensores);  //Hacer cast de uint32_t (es un long unsigned int) a char

	switch(sensor){
		case 1:  //Es el primer parámetro de la estructura, el alcohol
			x_param = 1;
			x_valor = 60;
			x_uds = 95;
			strcpy(parametro, "Alcohol: ");
			medida_float = (float)((float)med_sensores1*0.00390625);
			//Hacer cast de uint32_t (es un long unsigned int) a char
			//sprintf(valor, "%.2f", medida_float);  //No funciona %2f en STM32
			//sprintf(valor, "%d.%.2d", (int16_t)medida_float, abs((int16_t)((medida_float-(int16_t)medida_float)*100.0)));  //%d Muestra la parte entera y %.2d Muestra la parte decimal
			//Un solo dígito decimal
			sprintf(valor, "%d.%.1d", (int16_t)medida_float, abs((int16_t)((medida_float-(int16_t)medida_float)*10.0)));  //%d Muestra la parte entera y %.1d Muestra la parte decimal (el 1 fuerza que salga el 0 cuando es X.0)

			strcpy(unidades, "mg/l");
			break;
		case 2:  //Tensión
			x_param = 1;
			x_valor = 60;
			x_uds = 95;

			sprintf(tension_sis, "%lu", med_sensores1);  //Hacer cast de int a char
			sprintf(tension_dia, "%lu", med_sensores2);  //Hacer cast de int a char

			//Representar las tensiones como diastólica/sistólica
			strcat(valor, tension_sis); //Para concatenar 2 strings
			strcat(valor, "/"); //Para concatenar 2 strings
			strcat(valor, tension_dia); //Para concatenar 2 strings

			strcpy(parametro, "Tensión: ");
			sprintf(valor, "%lu", valor);  //Hacer cast de uint32_t (es un long unsigned int) a char
			strcpy(unidades, "mmHg");
			break;
		case 3:  //Pulso
			x_param = 1;
			x_valor = 60;
			x_uds = 95;
			strcpy(parametro, "Pulso: ");
			sprintf(valor, "%lu", med_sensores1);  //Hacer cast de uint32_t (es un long unsigned int) a char
			strcpy(unidades, "pul/min");
			break;
		case 4:  //Temperatura
			x_param = 1;
			x_valor = 60;
			x_uds = 95;
			strcpy(parametro, "Temp: ");
			strcpy(valor, (char) med_sensores1);
			strcpy(unidades, "ºC");
			break;
		case 5:  //Estrés
			x_param = 1;
			x_valor = 60;
			x_uds = 95;
			strcpy(parametro, "Estrés: ");
			strcpy(valor, med_sensores1);
			strcpy(unidades, "");
			break;
		default:
			x_param = 1;
			x_valor = 60;
			x_uds = 95;
			strcpy(parametro, "Error");
			strcpy(valor, "Algo fue mal");
			strcpy(unidades, "");
			break;
	}

	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetDrawColor(&u8g2, 1);  //Para invertir los colores
		u8g2_DrawBox(&u8g2, 0, 0, 128, 65);
		u8g2_SetDrawColor(&u8g2, 0);
		u8g2_SetFont(&u8g2, u8g2_font_helvB08_tf);  //Fuente para el texto superior
		u8g2_DrawStr(&u8g2, 30, 43, "PRECAUCIÓN");
		u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);  //Fuente para el texto inferior
		u8g2_DrawStr(&u8g2, x_param, 62, parametro);
		u8g2_DrawStr(&u8g2, x_valor, 62, valor);
		u8g2_DrawStr(&u8g2, x_uds, 62, unidades);
		u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_4x_t);  //Fuente para el icono
		u8g2_DrawGlyph(&u8g2, 50, 31, 280);  //Señal de precaución
	}while(u8g2_NextPage(&u8g2));
}




/*
 * Función que hace que un icono parpadee
 */
void imprimirParpadeo(uint8_t caso){

	uint32_t t_ref, t_diferencia, num_rep;

	t_ref = HAL_GetTick();  //Valor de tiempo de referencia
	num_rep = 0; //Para inicializar el bucle

	while(num_rep < REPETICIONES){
		t_diferencia = HAL_GetTick() - t_ref; //Para que ambas condiciones se evalúen con el mismo valor de tiempo

		if(t_diferencia < LIMITE_ON){
		  parpadeoSimple(caso, 1);
		}else{
		  parpadeoSimple(caso, 0);
		  if(t_diferencia > LIMITE_OFF){
			  t_ref = HAL_GetTick(); //Actualización del valor de tiempo de referencia
			  num_rep++;
		  }
		}
	}
	u8g2_SetDrawColor(&u8g2, 1); //Para que el último parpadeo no deje la pantalla con los colores invertidos
}

/*
 * Función que hace que tres iconos parpadeen en secuencia
 */
void imprimirSecuencia(uint8_t caso){

	uint32_t t_ref, t_diferencia, num_rep;

	t_ref = HAL_GetTick();  //Valor de tiempo de referencia
	num_rep = 0; //Para inicializar el bucle

	  while(num_rep < REPETICIONES){
		  t_diferencia = HAL_GetTick() - t_ref; //Para que ambas condiciones se evalúen con el mismo valor de tiempo

		  if(t_diferencia < LIMITE_PRIMERO){
			  parpadeoSecuencia(caso, 1);
		  }else{
			  if(t_diferencia < LIMITE_SEGUNDO){
				  parpadeoSecuencia(caso, 2);
			  }else{
				  parpadeoSecuencia(caso, 3);
				  if(t_diferencia > LIMITE_TERCERO){
					  t_ref = HAL_GetTick(); //Actualización del valor de tiempo de referencia
					  num_rep++;
				  }
			  }
		  }
	  }
	  u8g2_SetDrawColor(&u8g2, 1); //Para que el último parpadeo no deje la pantalla con los colores invertidos
}

/*
 * Imprime un mensaje por pantalla en función del aviso
 * Función que maneja las otras funciones de impresión por pantalla
 */
void imprimirAviso(maq_estados* maquina_est, int tipoAviso){
	//LLAMAR A LAS FUNCIONES DE IMPRIMIR SEGÚN EL TIPO DE AVISO
	switch(tipoAviso){
		case 0:  //Pantalla de inicio
			imprimirBasico(4);
			break;
		case 1:  //Pantalla de calibración
			imprimirSecuencia(5);
			break;
		case 2:  //Pantalla de inicio de medición de alcohol
			imprimirParpadeo(8);
			break;
		case 3:  //Pantalla de fin de medición de alcohol
			imprimirBasico(9);
			break;
		case 4:  //Pantalla de medidas de sensores
			imprimirMedidas(maquina_est);
			break;
		case 5:  //Pantalla de medida fuera de rango
//			imprimirAlertaSensor(maquina_est->medidas_sens);
			break;
		case 6:  //Pantalla de batería baja
			imprimirParpadeo(3);
			break;
		case 7:  //Pantalla de cargando
			imprimirSecuencia(6);
			break;
		case 8:  //Pantalla de alerta por cargar encendido
			imprimirBasico(7);
			break;
		default:
			break;
	}


}


/*
 *  Pantalla para las pruebas de sensores
 */
void imprimirPruebas(uint8_t sensor, float valor1, float valor2){
	//char  alcohol[16], tension_sis[16], tension_dia[16], pulso[16], temperatura[16], estres[16], tension[16];
	char  tension_sis[32], tension_dia[32], tension[64], valor[64];
	float valor_float;

	//Pasar a char los valores recogidos por los sensores
	if(sensor == 3){
		valor_float = (float)((float)valor1*0.00390625);
		sprintf(valor, "%d.%.1d", (int16_t)valor_float, abs((int16_t)((valor_float-(int16_t)valor_float)*10.0)));  //%d Muestra la parte entera y %.1d Muestra la parte decimal (el 1 fuerza que salga el 0 cuando es X.0)

	}else{
		//sprintf(valor, "%lu", valor1);  //Hacer cast de int a char
		sprintf(valor, "%d.%.3d", (int16_t)valor1, abs((int16_t)((valor1-(int16_t)valor1)*1000.0)));  //%d Muestra la parte entera y %.1d Muestra la parte decimal (el 1 fuerza que salga el 0 cuando es X.0)
	}

	//sprintf(tension_sis, "%lu", valor1);  //Hacer cast de int a char
	//sprintf(tension_dia, "%lu", valor2);  //Hacer cast de int a char
	if(sensor == 1){
		sprintf(tension_sis, "%d.%.1d", (int16_t)valor1, abs((int16_t)((valor1-(int16_t)valor1)*10.0)));  //%d Muestra la parte entera y %.1d Muestra la parte decimal (el 1 fuerza que salga el 0 cuando es X.0)
		sprintf(tension_dia, "%d.%.1d", (int16_t)valor2, abs((int16_t)((valor2-(int16_t)valor2)*10.0)));  //%d Muestra la parte entera y %.1d Muestra la parte decimal (el 1 fuerza que salga el 0 cuando es X.0)


		//Representar las tensiones como diastólica/sistólica
		strcat(tension, tension_sis); //Para concatenar 2 strings
		strcat(tension, "/"); //Para concatenar 2 strings
		strcat(tension, tension_dia); //Para concatenar 2 strings
	}

	u8g2_FirstPage(&u8g2);
	do{
		u8g2_SetFont(&u8g2, u8g2_font_helvB08_tf);  //Fuente para nombres de parámetros
		u8g2_DrawStr(&u8g2, 40, 10, "MEDIDAS");
		u8g2_DrawStr(&u8g2, 0, 20, "Alcohol:");
		u8g2_DrawStr(&u8g2, 0, 31, "Tensión:");
		u8g2_DrawStr(&u8g2, 0, 42, "Pulso:");
		u8g2_DrawStr(&u8g2, 0, 54, "Tª:");
		u8g2_DrawStr(&u8g2, 0, 64, "Estrés:");
/*
		//Pasar a char los valores recogidos por los sensores
		//sprintf(alcohol, "%lu", maquina_est->medidas_sens->alcohol);  //Hacer cast de int a char
		sprintf(tension_sis, "%lu", valor1);  //Hacer cast de int a char
		sprintf(tension_dia, "%lu", valor2);  //Hacer cast de int a char
		//sprintf(pulso, "%lu", maquina_est->medidas_sens->pulso);  //Hacer cast de int a char
		//sprintf(temperatura, "%lu", maquina_est->medidas_sens->temperatura);  //Hacer cast de int a char

		//Representar las tensiones como diastólica/sistólica
		strcat(tension, tension_sis); //Para concatenar 2 strings
		strcat(tension, "/"); //Para concatenar 2 strings
		strcat(tension, tension_dia); //Para concatenar 2 strings
		*/


		u8g2_SetFont(&u8g2, u8g2_font_profont12_tf);  //Fuente para nombres de parámetros
		switch(sensor){
			case 0:
				u8g2_DrawStr(&u8g2, 60, 20, valor);  //Alcohol
				break;
			case 1:
				u8g2_DrawStr(&u8g2, 60, 31, tension);  //Tensión
				break;
			case 2:
				u8g2_DrawStr(&u8g2, 60, 42, valor);  //Pulso
				break;
			case 3:
				u8g2_DrawStr(&u8g2, 60, 54, valor);  //Temperatura
				break;
			case 4:
				u8g2_DrawStr(&u8g2, 60, 64, valor);  //Estrés
				break;
		}
		//u8g2_DrawStr(&u8g2, 60, 54, valor1);  //Temperatura

		u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);  //Fuente para los iconos
		u8g2_DrawGlyph(&u8g2, 93, 14, 238);  //Electrocardiograma
		u8g2_DrawGlyph(&u8g2, 110, 64, 183);  //Corazón
	}while(u8g2_NextPage(&u8g2));


}

