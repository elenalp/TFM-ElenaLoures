#ifndef MAQUINAESTADOS_H
#define MAQUINAESTADOS_H

#include "stm32l4xx_hal.h"

//Situación actual de la máquina
typedef struct estado_maq{
	int estado_actual;   //Estado en que se encuentra la máquina de estados
	int prox_estado;    //Próximo estado que tomará la máquina de estados
	int prev_estado;    //Estado de la máquina de estados anterior al actual

} estado_maq; //Así defino una variable de tipo "struct estado_maq" llamada "estado_maq". Si no lo pongo, no compila

//Máquina de estados con la situación actual y las condiciones de cambio de estado
typedef struct maq_estados maq_estados;  //Necesito declararla antes y luego hacer la implementación de abajo para poder usarla en la declaración de las condiciones. Defino una variable de tipo "struct maq_estados" llamada "maq_estados"

//Funciones con condiciones para cambios de estado y acciones en el estado destino (punteros a funciones con argumento "maquina_estados")
//typedef void (*func_condicion) (maquina_estados*);
//Poner dos si necesito que una sea void y la otra no o si necesito otros argumentos en alguna
typedef int (*func_cambio) (maq_estados*);  //Devuelve int para poder controlar cuándo debe entrar a un estado u otro
typedef void (*func_destino) (maq_estados*);

//Condiciones para cambios de estado
typedef struct maq_transiciones{
	uint8_t estado_origen; //Estado anterior al destino, desde el que se hace la transición
  	func_cambio condic_cambio; //Comprobación de si se cumple o no la condición para hacer la transición
  	uint8_t estado_destino; //Estado al que irá
  	func_destino accion_destino;  //Acción a realizar en el estado destino
	//func_condicion condic_entrada;
	//func_condicion condic_salida;

} maq_transiciones;

//Medidas tomadas por todos los sensores
typedef struct medidasSensores {
	uint32_t alcohol;       //Medida de la concentración de alcohol obtenida del alcoholímetro
	uint32_t tension_sis;       //Medida de la tensión arterial obtenida del AFE (sistólica)
	uint32_t tension_dia;       //Medida de la tensión arterial obtenida del AFE (diastólica)
	uint32_t pulso;         //Medida del pulso obtenida del AFE
	uint32_t temperatura;   //Medida de la temperatura corporal obtenida del sensor
	char estres[16];        //Medida del nivel de estrés obtenida de la medida de GSR
	float ref_GSR;       //Medida de la respuesta galvánica de la piel en reposo
	float refPulso;       //Medida de pulso en reposo
	uint8_t medida_mala;  //Sensor en que la medida no es la que debe

} medidasSensores;

//Implementación de la máquina de estados
struct maq_estados{
	estado_maq* situacion_maq;
	maq_transiciones* transicion_maq;
	medidasSensores* medidas_sens;
	uint16_t nivel_bateria;   //Cuánta batería tiene
};

//Crear una máquina de estados
maq_estados* crear_maq(estado_maq* situ_maquina, maq_transiciones* trans_maquina);

//Inicializar la máquina de estados
void inicializar_maq(maq_estados* maquina_estados, estado_maq* situ_maquina, maq_transiciones* trans_maquina);

//Ejecutar la máquina de estados
void ejecutar_maq(maq_estados* maquina_estados);


#endif
