#ifndef SENSORES_H
#define SENSORES_H

#include "maquinaEstados.h"

/*
typedef struct medidasSensores {  //Medidas tomadas por todos los sensores
	double alcohol;       //Medida de la concentración de alcohol obtenida del alcoholímetro
	double tension;       //Medida de la tensión arterial obtenida del AFE
	double pulso;         //Medida del pulso obtenida del AFE
	double temperatura;   //Medida de la temperatura corporal obtenida del sensor
	double estres;        //Medida del nivel de estrés obtenida de la medida de GSR

} medidasSensores;
*/

/*
 * Condiciones de cambio de estado
 */
uint8_t medida_mal_alcohol(maq_estados* maquina_est);  //Devuelve 1 si la medida de alcohol se sale del rango establecido

uint8_t medida_mal(maq_estados* maquina_est);  //Devuelve 1 si alguna medida se sale del rango establecido

uint8_t medida_bien(maq_estados* maquina_est);  //Devuelve 1 si las medidas se encuentran en el rango establecido

uint8_t bat_baja(maq_estados* maquina_est);  //Devuelve 1 si el nivel de la batería se encuentra por debajo del umbral establecido

uint8_t conectado(maq_estados* maquina_est);  //Devuelve 1 si se ha puesto a cargar el sistema

uint8_t alerta_medir_dada(maq_estados* maquina_est);  //Devuelve 1 si dio la alerta tras una medida fuera de rango

uint8_t alerta_alcohol_dada(maq_estados* maquina_est);  //Devuelve 1 si dio la alerta tras una medida de nivel de alcohol fuera de rango

uint8_t desconectado(maq_estados* maquina_est);

uint8_t sistema_ON(maq_estados* maquina_est);

uint8_t alerta_dada(maq_estados* maquina_est);


/*
 * Acciones en estado destino
 */
void medirAlcohol(maq_estados* maquina_est);  //Activa el sensor de alcohol y almacena en la estructura el valor detectado
//float medirAlcohol(void);  //Activa el sensor de alcohol y almacena en la estructura el valor detectado
//float medirAlcohol(float ro);
//void medirGSR(maq_estados* maquina_est);  //Mide la resistencia de la piel y almacena en la estructura el valor leído
//uint32_t medirGSR(void);  //Mide la resistencia de la piel y almacena en la estructura el valor leído
uint32_t medirGSR(maq_estados* maquina_est);
void medirSensores(maq_estados* maquina_est);  //Activa todos los sensores excepto el sensor de alcohol y almacena en la estructura los valores detectados
//float calibracionAlcohol(void);
//void imprimirMedidas(maq_estados* maquina_est);   //Saca por pantalla todas las medidas de los sensores
//uint8_t medirTensionPulso(void);
void medirTensionPulso(maq_estados* maquina_est);
//void inicializarPPG(uint8_t potenciaLED, uint8_t mediaMuestras, uint8_t modoLEDs, int tasaMuestreo, int anchoPulso, int rangoADC, uint8_t amplitudRojo, uint8_t amplitudIR, uint8_t amplitudVerde);
void vibracion(void);    //Activa el motor para alertar mediante vibración

void imprimirYvibrar(maq_estados* maquina_est);   //Imprime por pantalla el parámetro por el cual se realiza la alerta y activa la vibración

void inicializarPPG(uint8_t potenciaLED, uint8_t mediaMuestras, uint8_t modoLEDs, int tasaMuestreo, int anchoPulso, int rangoADC, uint8_t amplitudRojo, uint8_t amplitudIR, uint8_t amplitudVerde);
//void imprimirAviso(maq_estados* maquina_est, int tipoAviso);  //Imprime un mensaje por pantalla en función del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si batería baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido

//void medirTemp(maq_estados* maquina_est);   //Realiza la medición de la temperatura corporal
uint32_t medirTemp1(void);

void medirBateria(maq_estados* maquina_est, uint32_t valorADC1);  //Estima la batería que tiene

#endif
