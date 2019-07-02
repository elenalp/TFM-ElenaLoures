#include "maquinaEstados.h"
#include <stdlib.h> //Para el malloc

//Crear una máquina de estados
maq_estados* crear_maq(estado_maq* situ_maquina, maq_transiciones* trans_maquina, medidasSensores* medidas_sens){

	maq_estados* maquina_estados = (maq_estados*) malloc(sizeof(maq_estados));  //Cast del malloc para asegurar que es del tipo deseado
	inicializar_maq(maquina_estados, situ_maquina, trans_maquina, medidas_sens);
	return maquina_estados;
}

//Inicializar la máquina de estados
void inicializar_maq(maq_estados* maquina_estados, estado_maq* situ_maquina, maq_transiciones* trans_maquina, medidasSensores* medidas_sens){

	situ_maquina->estado_actual = 0;
	situ_maquina->prev_estado = 0;
	situ_maquina->prox_estado = 0;
	maquina_estados->situacion_maq = situ_maquina;
	maquina_estados->transicion_maq = trans_maquina;
	maquina_estados->medidas_sens->medida_mala = 0;
//	maquina_estados->medidas_sens->medida_mala = 30;   //Para saber que aún no se comprobó el nivel de alcohol
	maquina_estados->medidas_sens->alcohol = 0;
	//strcpy(maquina_estados->medidas_sens->estres, "No medido");
	maquina_estados->medidas_sens->pulso = 0;
	maquina_estados->medidas_sens->temperatura = 0;
	maquina_estados->medidas_sens->tension_dia = 0;
	maquina_estados->medidas_sens->tension_sis = 0;
	maquina_estados->medidas_sens->refPulso = 0;
	maquina_estados->medidas_sens->ref_GSR = 0;

}

//Ejecutar la máquina de estados
void ejecutar_maq(maq_estados* maquina_estados){

	maq_transiciones* trans_posibles;
 	for(trans_posibles = maquina_estados->transicion_maq; trans_posibles->estado_origen >= 0; ++trans_posibles){  //La condición para seguir en el for es para comprobar que no estés en el caso de error: (-1, NULL, -1, NULL)
 		//PARA PRUEBAS!!!
 			//imprimirBasico(4);
 			//HAL_Delay(1000); //En ms
 		//if((maquina_estados->situacion_maq->estado_actual == trans_posibles->estado_origen) && trans_posibles->condic_cambio(maquina_estados)){  //Además de estar en la transición correcta (la que parte del estado en que está), debes ver si se cumple la condición para ir al estado destino
 			if((maquina_estados->situacion_maq->estado_actual == trans_posibles->estado_origen) & trans_posibles->condic_cambio(maquina_estados)){  //Además de estar en la transición correcta (la que parte del estado en que está), debes ver si se cumple la condición para ir al estado destino
      		maquina_estados->situacion_maq->estado_actual = trans_posibles->estado_destino;  //Ir al estado destino

	      	if(trans_posibles->accion_destino){  //Si hay acción a realizar en el estado destino
	        	trans_posibles->accion_destino(maquina_estados);  //Ejecutar la acción del estado destino
	      	}

  		break;
    	}
 		//PARA PRUEBAS!!!
 				//imprimirBasico(8);
 				//HAL_Delay(1000); //En ms
  	}
}
