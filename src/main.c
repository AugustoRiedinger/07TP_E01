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
/*Pines del ADC - PC0:*/
#define ADC_Port GPIOC
#define ADC_Pin  GPIO_Pin_0

/*Parámetros de configuración del TIM3 para refresco del LCD:*/
#define Freq 	 4
#define TimeBase 200e3

/*Maximo voltaje en el pin del ADC:*/
#define MaxVoltADC	3

/*Maximo valor de cuentas digitales:*/
#define MaxDigCount 4095

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

/*Variable para almacenar el valor de cuenta digital del ADC:*/
uint32_t DigCount;

/*Variable almacenar la conversion de cuenta digital a volts:*/
float 	 VoltEq;

/*Variable para almacenar el tiempo medido:*/
uint32_t MeasuredTime = 0;

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
		/*Se inicializa el codigo para medir tiempos:*/
		DWT->CTRL |= 1; // enable the counter
		DWT->CYCCNT = 0; // reset the counter

		/*Se lee el valor de cuenta digital en el ADC:*/
		DigCount = READ_ADC(ADC_Port, ADC_Pin);

		/*Conversion a voltaje equivalente:*/
		VoltEq = (float) ADConv * MaxVoltADC / MaxDigCount;

		/*Se termina el codigo para medir tiempos y se guarda el resultado:*/
		MeasuredTime = DWT->CYCCNT;
		/*Se substrae el ciclo utilizada para transferir CYCCNT a la variable:*/
		MeasuredTime--;
    }
}

/*------------------------------------------------------------------------------
INTERRUPCIONES:
------------------------------------------------------------------------------*/
/*Interrupción por agotamiento de cuenta del TIM3 cada 250mseg (4 Hz):*/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		/*Buffers para almacenamiento de datos:*/
		char BufferDigCount[BufferLength];
		char BufferVoltEq[BufferLength];
		char BufferMeasuredTime[BufferLength];

		/*Refresco del LCD: */
		CLEAR_LCD_2x16(LCD_2X16);

		/*Copiado de los valores en las variables a los buffers:*/
		sprintf(BufferDigCount, "%d", DigCount);
		sprintf(BufferVoltEq, "%.2f", VoltEq);
		sprintf(BufferMeasuredTime, "%d", MeasuredTime);

		/*Mensaje para mostrar las cuentas digitales del ADC:*/
		PRINT_LCD_2x16(LCD_2X16, 0, 0, "Dig=");
		PRINT_LCD_2x16(LCD_2X16, 4, 0, BufferDigCount);

		/*Mensaje para mostrar el valor medido real en el ADC:*/
		PRINT_LCD_2x16(LCD_2X16, 6, 0, "Volt=");
		PRINT_LCD_2x16(LCD_2X16, 11, 0, BufferVoltEq);

		/*Mensaje para mostrar el tiempo de la operacion:*/
		PRINT_LCD_2x16(LCD_2X16, 0, 1, "T-Dig:");
		PRINT_LCD_2x16(LCD_2X16, 6, 0, BufferMeasuredTime);
	}
}

