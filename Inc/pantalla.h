#ifndef PANTALLA_H
#define PANTALLA_H

#include "u8g2.h"
#include "maquinaEstados.h"

static u8g2_t u8g2; //Pantalla

void inicializarPantalla(void);  //Ejecutar las funciones neesarias para comunicarse por I2C con la pantalla
void imprimirPantalla(uint8_t on_off);
void imprimirBasico(uint8_t caso);  //Pantalla en la que solo aparece texto e icono de forma estática
void parpadeoSimple(uint8_t caso, uint8_t encendido);  //Pantalla en la que aparece un texto y un icono con parpadeo
void parpadeoSecuencia(uint8_t caso, uint8_t icono_mostrado);  //Pantalla en la que aparece un texto y varios iconos parpadeando por orden
void imprimirMedidas(maq_estados* maquina_est);  //Saca por pantalla todas las medidas de los sensores
void imprimirAlertaSensor(medidasSensores* med_sensores);  //Pantalla en la que se muestra una alerta por la medida de algún sensor
void imprimirParpadeo(uint8_t caso);  //Función que hace que un icono parpadee
void imprimirSecuencia(uint8_t caso);  //Función que hace que tres iconos parpadeen en secuencia
void imprimirAviso(maq_estados* maquina_est, int tipoAviso);  //Imprime un mensaje por pantalla en función del aviso tipoAviso=0 si se inicia el sistema y se va a medir el nivel de alcohol, tipoAviso=1 si batería baja, tipoAviso=2 si se ha conectado para cargar y tipoAviso=3 si se carga estando encendido
void imprimirPruebas(uint8_t sensor, float valor1, float valor2);  //Pantalla para las pruebas de sensores

#endif
