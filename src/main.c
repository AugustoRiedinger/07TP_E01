/*
  * @file    main.c
  * @author  G. Garcia & A. Riedinger.
  * @version 0.1
  * @date    27-06-21
  * @brief   Mide la tensión en un PIN analogico y la muestra en un LCD. Se
  * 		 analiza el tiempo que dura esta accion.

  * SALIDAS:
  	  *	LCD		Pines Convencionales
  * ENTRADAS:
  	  * ADC		PC0
*/
/*------------------------------------------------------------------------------
LIBRERIAS:
------------------------------------------------------------------------------*/
#include "mi_libreria.h"

/*------------------------------------------------------------------------------
DEFINICIONES:
------------------------------------------------------------------------------*/
/*Configuración del TIM3 cada 250mseg para refresco del LCD:*/
#define Freq 	 4
#define TimeBase 200e3

/*Pines del ADC - PC0:*/
#define ADC_Port GPIOC
#define ADC_Pin  GPIO_Pin_0

/*Maximo voltaje en el pin del ADC:*/
#define MaxVoltADC	3

/*Maximo valor de cuentas digitales:*/
#define MaxDigCount 4095

/*Ciclos de lectura maximo:*/
#define Cycles	100

/*------------------------------------------------------------------------------
DECLARACION TAREAS:
------------------------------------------------------------------------------*/
void _ADC(void);

/*------------------------------------------------------------------------------
VARIABLES GLOBALES:
------------------------------------------------------------------------------*/
/*Definicion de los pines del LCD: */
LCD_2X16_t LCD_2X16[] = {
			// Name  , PORT ,   PIN      ,         CLOCK       ,   Init
			{ TLCD_RS, GPIOC, GPIO_Pin_10, RCC_AHB1Periph_GPIOC, Bit_RESET },
			{ TLCD_E,  GPIOC, GPIO_Pin_11, RCC_AHB1Periph_GPIOC, Bit_RESET },
			{ TLCD_D4, GPIOC, GPIO_Pin_12, RCC_AHB1Periph_GPIOC, Bit_RESET },
			{ TLCD_D5, GPIOD, GPIO_Pin_2,  RCC_AHB1Periph_GPIOD, Bit_RESET },
			{ TLCD_D6, GPIOF, GPIO_Pin_6,  RCC_AHB1Periph_GPIOF, Bit_RESET },
			{ TLCD_D7, GPIOF, GPIO_Pin_7,  RCC_AHB1Periph_GPIOF, Bit_RESET }, };

/*Variable para contar las veces que se leyo el ADC:*/
uint32_t ADC_Count = 0;

/*Variable para almacenar el valor de cuenta digital del ADC:*/
uint32_t DigCount;

/*Variable almacenar la conversion de cuenta digital a volts:*/
float 	 VoltEq;

/*Variable para almacenar el tiempo medido:*/
uint32_t MeasuredTime = 0;

/*Variable para transformar el timepo medido a segundos:*/
double Time;
int    TimeU;
double TimeD;

int main(void)
{
/*------------------------------------------------------------------------------
CONFIGURACION DEL MICRO:
------------------------------------------------------------------------------*/
	SystemInit();

	/*Inicializacion PC0 como ADC:*/
	INIT_ADC(ADC_Port, ADC_Pin);

	/*Inicializacion del DISPLAY LCD:*/
	INIT_LCD_2x16(LCD_2X16);

	/*Inicialización del TIM3 para refresco del LCD:*/
	INIT_TIM3();
	SET_TIM3(TimeBase, Freq);

/*------------------------------------------------------------------------------
BUCLE PRINCIPAL:
------------------------------------------------------------------------------*/
    while (1)
    {
    	/*Definicion de la estructura para contar ciclos:*/
    	CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    	DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    	/*Se reseta el contador:*/
    	DWT -> CYCCNT = 0;

    	/*Se lee el valor de cuenta digital en el ADC segun Cycles:*/
    	for (ADC_Count = 0; ADC_Count <= Cycles; ADC_Count++)
			DigCount = READ_ADC(ADC_Port, ADC_Pin);

    	/*Conversion a voltaje equivalente:*/
		VoltEq = (float) DigCount * MaxVoltADC / MaxDigCount;

		/*Se termina el codigo para medir tiempos y se guarda el resultado:*/
		MeasuredTime = DWT->CYCCNT;

		/*Se substrae el ciclo utilizada para transferir CYCCNT a la variable:*/
		MeasuredTime--;

		/*Se convierte el tiempo de medida digital a microsegundos:*/
		Time = (double) MeasuredTime * 1.0 / 180;
		TimeU = (int) Time;
		TimeD = (Time - (double) TimeU) * 10.0;
    }
}

/*------------------------------------------------------------------------------
INTERRUPCIONES:
------------------------------------------------------------------------------*/
/*Interrupción por agotamiento de cuenta del TIM3 cada 250mseg (4 Hz):*/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		/*Buffers para almacenamiento de datos:*/
		char BufferDigCount[BufferLength];
		char BufferVoltEq[BufferLength];
		char BufferMeasuredTime[1000];
		char BufferADC_Count[BufferLength];

		/*Refresco del LCD: */
		CLEAR_LCD_2x16(LCD_2X16);

		/*Copiado de los valores en las variables a los buffers:*/
		sprintf(BufferDigCount, "%d", DigCount);
		sprintf(BufferVoltEq, "%.2f", VoltEq);
		sprintf(BufferMeasuredTime, "%d.%.2d", TimeU,TimeD);
		sprintf(BufferADC_Count, "%d", ADC_Count);

		/*Mensaje para mostrar las cuentas digitales del ADC:*/
		PRINT_LCD_2x16(LCD_2X16, 0, 0, "Dig=");
		PRINT_LCD_2x16(LCD_2X16, 4, 0, BufferDigCount);

		/*Mensaje para mostrar el valor medido real en el ADC:*/
		PRINT_LCD_2x16(LCD_2X16, 10, 0, "V=");
		PRINT_LCD_2x16(LCD_2X16, 12, 0, BufferVoltEq);

		/*Mensaje para mostrar el tiempo de la operacion:*/
		PRINT_LCD_2x16(LCD_2X16, 0, 1, "T:");
		PRINT_LCD_2x16(LCD_2X16, 2, 1, BufferMeasuredTime);
		PRINT_LCD_2x16(LCD_2X16, 7, 1, "uS ");


		/*Mensaje para mostrar las cuentas:*/
		PRINT_LCD_2x16(LCD_2X16, 10, 1, "C:");
		PRINT_LCD_2x16(LCD_2X16, 12, 1, BufferADC_Count);

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}
