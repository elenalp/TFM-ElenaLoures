#include "maquinaEstados.h"
#include <stdlib.h> //Para el malloc
#include <string.h> //Para el strcpy


//Crear una m�quina de estados
maq_estados* crear_maq(maq_transiciones* trans_maquina){

	maq_estados* maquina_estados = (maq_estados*) malloc(sizeof(maq_estados));  //Cast del malloc para asegurar que es del tipo deseado
	medidasSensores* medidas_sens = (medidasSensores*) malloc(sizeof(medidasSensores));  //Cast del malloc para asegurar que es del tipo deseado
	estado_maq* situ_maquina = (estado_maq*) malloc(sizeof(estado_maq));  //Cast del malloc para asegurar que es del tipo deseado
	inicializar_maq(maquina_estados, situ_maquina, trans_maquina, medidas_sens);
	return maquina_estados;
}

//Inicializar la m�quina de estados
void inicializar_maq(maq_estados* maquina_estados, estado_maq* situ_maquina, maq_transiciones* trans_maquina, medidasSensores* medidas_sens){

	situ_maquina->estado_actual = 0;
	situ_maquina->prev_estado = 0;
	situ_maquina->prox_estado = 0;
	maquina_estados->situacion_maq = situ_maquina;
	maquina_estados->transicion_maq = trans_maquina;
	maquina_estados->medidas_sens = medidas_sens;
	maquina_estados->situacion_maq->estado_actual = 0;
	maquina_estados->situacion_maq->prev_estado = 0;
	maquina_estados->situacion_maq->prox_estado = 0;

	/*
	 * Inicializaciones para que durante la calibraci�n no salgan valores raros
	 */
	maquina_estados->medidas_sens->pulso = 0;  //Para que no salte la alerta por estr�s si no detect� pulso
	maquina_estados->medidas_sens->temperatura = 36;
	strcpy(maquina_estados->medidas_sens->estres, "Bajo");
	
}

//Ejecutar la m�quina de estados
void ejecutar_maq(maq_estados* maquina_estados){

	maq_transiciones* trans_posibles;
	for(trans_posibles = maquina_estados->transicion_maq; trans_posibles->estado_origen >= 0; trans_posibles+=sizeof(trans_posibles)){  //La condici�n para seguir en el for es para comprobar que no est�s en el caso de error: (-1, NULL, -1, NULL)
 		
 		if((maquina_estados->situacion_maq->estado_actual == trans_posibles->estado_origen) && trans_posibles->condic_cambio(maquina_estados)){  //Adem�s de estar en la transici�n correcta (la que parte del estado en que est�), debes ver si se cumple la condici�n para ir al estado destino

			maquina_estados->situacion_maq->estado_actual = trans_posibles->estado_destino;  //Ir al estado destino

	      	if(trans_posibles->accion_destino){  //Si hay acci�n a realizar en el estado destino
	        	trans_posibles->accion_destino(maquina_estados);  //Ejecutar la acci�n del estado destino
	      	}

  		break;
 			
    	}
 	
  	}
}
