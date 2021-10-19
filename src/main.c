/*
  * @file    main.c
  * @author  G. Garcia & A. Riedinger.
  * @version 0.1
  * @date    27-06-21
  * @brief   Mide la tensión en un PIN analogico y la muestra en un LCD.

  * SALIDAS:

  	  *	LCD:
  	  	  *RS -> PC10
  	  	  *E  -> PC11
  	  	  *D4 -> PC12
  	  	  *D5 -> PD2
  	  	  *D6 -> PF6
  	  	  *D7 -> PF7

  * ENTRADAS:

  	  * ADC -> PC0
*/
/*------------------------------------------------------------------------------
LIBRERIAS:
------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stdio.h"						//Uso de funciones basicas de c como sprintf
#include "adc.h"						//Uso de conversores analogico-digital
#include "stm32_ub_lcd_2x16.h"			//Uso del Display LCD 2x16

/*------------------------------------------------------------------------------
DEFINICIONES:
------------------------------------------------------------------------------*/
#define InitDelay 100e5

/*------------------------------------------------------------------------------
DECLARACION DE FUNCIONES:
------------------------------------------------------------------------------*/
void DELAY(volatile uint32_t n);

int main(void)
{
/*------------------------------------------------------------------------------
DECLARACION DE VARIABLES:
------------------------------------------------------------------------------*/
	char 	BufferADC[100];
	char 	BufferVoltEq[100];

	float  	VoltEq;

	int 	ADConv;
	int  	i;

/*------------------------------------------------------------------------------
CONFIGURACION DEL MICRO:
------------------------------------------------------------------------------*/
//Inicializacion de los puertos analogicos:
	ADC_INIT();

//Inicializacion del DISPLAY LCD:
	SystemInit();
	UB_LCD_2x16_Init();

/*------------------------------------------------------------------------------
BUCLE PRINCIPAL:
------------------------------------------------------------------------------*/
    while (1)
    {
    	//Refresco del DISPLAY:
    	DELAY(InitDelay);
    	UB_LCD_2x16_Clear();

    	//Lectura del pin analogico y conversion a int:
    	ADConv = READ_ADC1();

    	//Conversion de digital a voltaje. 4095 es la cuenta al maximo.
    	//Si 4095 cuentas digitales equivalen a 3V, entonces
    	//el valor actual de cuentas digitales equivale a VoltEq:
    	VoltEq = (float) ADConv * 3 / 4095;

    	//Conversion de los valores enteros a string y almacenado en buffers:
    	sprintf(BufferADC, "CTS:%d", ADConv);
    	sprintf(BufferVoltEq, "%.2f",  VoltEq);

    	//Muestra de los valores almacenados en el DISPLAY:
    	UB_LCD_2x16_String(0 , 0, BufferADC);
    	UB_LCD_2x16_String(9 , 0, "|");
    	UB_LCD_2x16_String(11, 0, BufferVoltEq);
    	UB_LCD_2x16_String(15, 0, "V");

    	//Creacion de la barrita en la segunda fila.
    	//Si 3V equivalen a 16 barritas, el voltaje actual equivale a i.
    	//Luego, se disminuye i en cada pasada hasta llegar al principio del LCD:
    	for (i = VoltEq * 16 / 3 ; i >=0 ; i--)
    	    UB_LCD_2x16_String(i,1,".");
    }
}

/*------------------------------------------------------------------------------
FUNCIONES:
------------------------------------------------------------------------------*/
void DELAY(volatile uint32_t n)
{
  while(n--) {};
}
