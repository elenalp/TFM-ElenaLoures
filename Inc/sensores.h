#ifndef SENSORES_H
#define SENSORES_H

#include "maquinaEstados.h"

/*
typedef struct medidasSensores {  //Medidas tomadas por todos los sensores
	double alcohol;       //Medida de la concentraci�n de alcohol obtenida del alcohol�metro
	double tension;       //Medida de la tensi�n arterial obtenida del AFE
	double pulso;         //Medida del pulso obtenida del AFE
	double temperatura;   //Medida de la temperatura corporal obtenida del sensor
	double estres;        //Medida del nivel de estr�s obtenida de la medida de GSR

} medidasSensores;
*/

/*
 * Condiciones de cambio de estado
 */
int medida_mal(maq_estados* maquina_est);  //Devuelve 1 si alguna medida se sale del rango establecido

int medida_bien(maq_estados* maquina_est);  //Devuelve 1 si las medidas se encuentran en el rango establecido

int bat_baja(maq_estados* maquina_est);  //Devuelve 1 si el nivel de la bater�a se encuentra por debajo del umbral establecido

int conectado(maq_estados* maquina_est);  //Devuelve 1 si se ha puesto a cargar el sistema

int alerta_medir_dada(maq_estados* maquina_est);  //Devuelve 1 si dio la alerta tras una medida fuera de rango

int alerta_alcohol_dada(maq_estados* maquina_est);  //Devuelve 1 si dio la alerta tras una medida de nivel de alcohol fuera de rango

int desconectado(maq_estados* maquina_est);

int sistema_ON(maq_estados* maquina_est);

int alerta_dada(maq_estados* maquina_est);


/*
 * Acciones en estado destino
 */
void medirAlcohol(maq_estados* maquina_est);  //Activa el sensor de alcohol y almacena en la estructura el valor detectado

void medirSensores(maq_estados* maquina_est);  //Activa todos los sensores excepto el sensor de alcohol y almacena en la estructura los valores detectados

//void imprimirMedidas(maq_estados* maquina_est);   //Saca por pantalla todas las medidas de los sensores

void vibracion(void);    //Activa el motor para alertar mediante vibraci�n

void imprimirYvibrar(maq_estados* maquina_est);   //Imprime por pantalla el par�metro por el cual se realiza la alerta y activa la vibraci�n

//void imprimirAviso(maq_estados* maquina_est, int tipoAviso);  //Imprime un mensaje por pantalla en funci�n del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si bater�a baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido

void medirTemp(maq_estados* maquina_est);   //Realiza la medici�n de la temperatura corporal

void medirBateria(maq_estados* maquina_est, uint32_t valorADC1);  //Estima la bater�a que tiene

#endif
