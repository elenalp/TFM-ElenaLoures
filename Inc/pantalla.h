#ifndef PANTALLA_H
#define PANTALLA_H

#include "u8g2.h"
#include "maquinaEstados.h"

static u8g2_t u8g2; //Pantalla

void inicializarPantalla(void);
void imprimirPantalla(uint8_t on_off);
void imprimirBasico(uint8_t caso);
void parpadeoSimple(uint8_t caso, uint8_t encendido);
void parpadeoSecuencia(uint8_t caso, uint8_t icono_mostrado);
//void imprimirSensores(maq_estados* maquina_est);
void imprimirMedidas(maq_estados* maquina_est);
//void imprimirAlertaSensor(medidasSensores* med_sensores, uint8_t sensor);
//void imprimirAlertaSensor(uint32_t med_sensores, uint8_t sensor);
//void imprimirAlertaSensor(uint32_t med_sensores1, uint32_t med_sensores2 , uint8_t sensor);
void imprimirAlertaSensor(medidasSensores* med_sensores);
void imprimirParpadeo(uint8_t caso);
void imprimirSecuencia(uint8_t caso);
void imprimirAviso(maq_estados* maquina_est, int tipoAviso);  //Imprime un mensaje por pantalla en función del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si batería baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido
void imprimirPruebas(uint8_t sensor, float valor1, float valor2);

#endif
