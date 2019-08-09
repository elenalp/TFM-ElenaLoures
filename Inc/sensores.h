#ifndef SENSORES_H
#define SENSORES_H

#include "maquinaEstados.h"



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

uint32_t medirGSR(maq_estados* maquina_est);  //Mide la resistencia de la piel y almacena en la estructura el valor leído

void medirSensores(maq_estados* maquina_est);  //Activa todos los sensores excepto el sensor de alcohol y almacena en la estructura los valores detectados

void medirTensionPulso(maq_estados* maquina_est);

void vibracion(void);    //Activa el motor para alertar mediante vibración

void imprimirYvibrar(maq_estados* maquina_est);   //Imprime por pantalla el parámetro por el cual se realiza la alerta y activa la vibración

void inicializarPPG(uint8_t potenciaLED, uint8_t mediaMuestras, uint8_t modoLEDs, int tasaMuestreo, int anchoPulso, int rangoADC, uint8_t amplitudRojo, uint8_t amplitudIR, uint8_t amplitudVerde);

//void medirTemp(maq_estados* maquina_est);   //Realiza la medición de la temperatura corporal
uint32_t medirTemp1(void);   //Realiza la medición de la temperatura corporal

void medirBateria(maq_estados* maquina_est, uint32_t valorADC1);  //Estima la batería que tiene

#endif
