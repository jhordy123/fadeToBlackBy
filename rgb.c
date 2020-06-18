#include <avr/io.h> //    ATMEGA328P --- atmel studio 7. 
#define  F_CPU 16000000UL
#include "util/delay.h"
#include "avr/interrupt.h"
#include "avr/pgmspace.h"
//#include "lcd.h"
#include "avr/eeprom.h"
/*
#include <stdint.h>
#include <stdint-gcc.h>
#include <avr/sfr_defs.h>
*/
#define LED_PORT PORTD
#define LED_DDR  DDRD
#define LED_PIN  0
//#define   SCALE8_C 1 
//#define  FASTLED_SCALE8_FIXED 1



typedef struct rgb_color{
	
	uint8_t NeoRed, NeoGreen, NeoBlue ;
	
} rgb_color;


void  __attribute__((noinline)) led_strip_write(rgb_color * colors, uint16_t count){
	
	LED_PORT &= ~(1<< LED_PIN);   // salida en 0 logico
	LED_DDR |=(1<<LED_PIN);       //salida en PORTC.0
	
	cli(); // desactivamos las interrupciones
	
	while(count--){
		
		asm volatile (     
		"ld __tmp_reg__ , %a0+\n"       // Avanzar el puntero de rojo a verde.
		"ld __tmp_reg__ , %a0\n"        // Lea el componente verde y deje el puntero apuntando a verde.
		
		"rcall send_led_strip_byte%=\n"  // Enviar componente verde // 1°
		"ld __tmp_reg__ , -%a0\n"        // REGRESA Y Lee el componente rojo y deja el puntero en rojo.
		
		"rcall send_led_strip_byte%=\n"  //// Enviar componente ROJO. 2°
		"ld __tmp_reg__ , %a0+\n"        // Puntero avanzado de rojo a verde.
		"ld __tmp_reg__ , %a0+\n"        // Avanzar el puntero de verde a azul
		"ld __tmp_reg__ , %a0+\n"        // Lee el componente azul y deja el puntero en el rojo del siguiente color.
		
		"rcall send_led_strip_byte%=\n\t" ///// Enviar componente azul. 3°
		"rjmp led_strip_asm_end%=\n"      //// Salta las subrutinas de ensamblaje.
		
		// subrutina send_led_strip_byte: envía un byte a la tira de LED.
		"send_led_strip_byte%=:\n\t"
		
		"rcall send_led_strip_bit%=\n" // Enviar el bit más significativo (bit 7).
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"
		"rcall send_led_strip_bit%=\n"  // Enviar el bit menos significativo (bit 0).
		"ret\n"
		
		
		"send_led_strip_bit%=:\n"
		
		#if F_CPU == 8000000
		"rol __tmp_reg__\n"  //// Girar a la izquierda a través de carry.
		#endif
		"sbi %2, %3\n"     // // Sube la línea
		
		#if F_CPU != 8000000
		"rol __tmp_reg__\n"  // // Girar a la izquierda a través de carry.
		#endif
		
		#if F_CPU == 16000000
		"nop\n"  "nop\n"
		#elif F_CPU == 20000000
		"nop\n" "nop\n" "nop\n" "nop\n"
		
		#elif F_CPU != 8000000
		#error "Unsupported F_CPU"
		
		#endif
		
		"brcs .+2\n"        // Si el bit a enviar es 0, ahora baja la línea.
		"cbi %2, %3\n"
		
		
		#if F_CPU == 8000000
		"nop\n" "nop\n"
		#elif F_CPU == 16000000
		"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
		#elif F_CPU == 20000000
		"nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
		"nop\n""nop\n"
		#endif
		
		"brcc .+2\n" "cbi %2, %3\n"       // Si el bit a enviar es 1, ahora baja la línea.
		
		"ret\n"
		"led_strip_asm_end%=:\n"
		
		: "=b" (colors)
		: "0" (colors),                        // % a0 puntos al siguiente color para mostrar
		"M" ( _SFR_IO_ADDR(LED_PORT)),   //// % 2 es el registro de puerto (por ejemplo, PORTC)
		"M" (LED_PIN)                    // % 3 es el número de pin (0-8)

		);
		
		// Descomenta la línea de abajo para habilitar temporalmente las interrupciones entre cada color.
		//sei (); asm volatile ("nop\n"); cli ();
		
		
	}
	
	sei();            // Vuelva a habilitar las interrupciones ahora que hemos terminado.
	
	//asm volatile ("nop\n");
	_delay_us(80);    // Enviar la señal de reinicio.
	
	
}


//**************FUNCIONES QUE INVOLUCRAN EL DESVNECIMIENTO DE LOS LEDS ELEGIDOS *****************************************************************************

void fadeLEDs(unsigned char);
void fadeToBlackBy( rgb_color * colors2 , unsigned char ,unsigned char);
void MEMORY(rgb_color*colors,rgb_color*colors2);

//***************FUNCIONES INVOLUCRADAS EN EL ENVIO Y CONTROL DE LA TIRA DE LEDS******************************************************************************

void showLED();  //FUNCION DE ENVIO DE LEDS 
rgb_color HsvToRgb(unsigned char,unsigned char,unsigned char);                
void clearTiraled(rgb_color*colors);                                        
void cylonWithHueControl();                                                

//************************FUNCION DE LECTUTA DEL ADC******************************************************************************
void LeeADC();
volatile unsigned char ADCvalor;

//**************************VARIABLES Y ARREGLOS UTILIZADOS EN EL ******************************************************************


#define LED_COUNT 64                                         

rgb_color colors[LED_COUNT+1];                                 
rgb_color colors2[LED_COUNT+1];

volatile unsigned char LEDPosicion = 0;                           

//************************* FUNCION PRENCIPAL Y WHILE ***********************************************************************

int main(void)
{
	 
	 //DDRD &=(PORTD2);          //  PORTD.2 -->INT0
	   
	 //DDRD &=(PORTD3);         // PORTD.3--> INT1
	 
	 
	 DDRC &=(0<<PORTC2) ;  
	 
	 DDRB  &= (0<<PORTB0)  ; 
	 PORTB |= (1<<PORTB0) ;
	 
	 DDRB  &=(0<<PORTB1) ;    /
	 PORTB |=(1<<PORTB1) ;

         ADMUX = 0b00100010; // AN2 -> ON , ADLAR = 1 ,  
	 ADCSRA =0b11000110; //  f= 250 khz / 13 = 19khz --> adc
	 
	
   while (1) {                 // LAZO INFINITO DEL PROGRAMA 
	   
	   // arrglar la funcion e HSVTORGB por que tiene poblemas con el cero joder
			
		 LeeADC();
		 
		 if (!(PINB & 0x02)){
			 _delay_ms(100);
			 
			 if((!(PINB & 0x02))){
				 LEDPosicion --;
				 if(LEDPosicion==255){
					 LEDPosicion =64;
				 }
				 
			 }
			 
		 }
		 
		 if (!(PINB & 0x01)){
			 _delay_ms(100);
			 
			 if ((!(PINB & 0x01))){
				 LEDPosicion++;
				 if (LEDPosicion ==65){
					 LEDPosicion = 0 ;
				 }
			 }
			
		 }
		 
		
		cylonWithHueControl();
		fadeLEDs(16);
		showLED(); 
		
		_delay_ms(50); 
		
    
  }
  
}



//=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=//
//=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=////=//=//=//=//

//****************************************LECRURA DEL ADC ****************************************************************************************************
void LeeADC(){
	
	while(!(ADCSRA & 1<<ADIF));
	ADCSRA |=(1<<ADIF);
	
	
	ADCvalor = ADCH; 
	
	ADCSRA |=(1<<ADSC);
	asm("nop");                            

}

//*****************FUNCION QUE CONTROLA LOS LEDS A PRENDER POR MEDIO DE UN POTENCIOMETRO Y ADC

void cylonWithHueControl(){
	 
	
	//unsigned char POTVal = 85; 
	unsigned char intensidad = 255; 
	colors[LEDPosicion] = HsvToRgb(ADCvalor,255,intensidad);       
	
		
}
//*****************FUNCION QUE ENVIA LOS DATOS A LA TIRA LED *******************************************************************

void showLED(){  //unsigned char  LEDPos , unsigned char H, unsigned char S, unsigned char V
	 	
	led_strip_write(colors,LED_COUNT);       
	clearTiraled(colors);                 
	MEMORY(colors,colors2);               
	
}

//*******************************FUNCION DE DESVANECIMIENTO DE LOS LEDS INDICADOS **************************************++

void fadeLEDs(unsigned char fadeVal){
	
	 colors2[LEDPosicion] = colors[LEDPosicion];
	   
	 fadeToBlackBy (colors2,LED_COUNT,255-fadeVal);
	

}
//************************************************************************************************************************************************************

void fadeToBlackBy (rgb_color * colors2, unsigned char led_count , unsigned char fadeVal){
	
	   unsigned int scale_fixed = fadeVal + 1;
	   for (unsigned char i=0; i<=LED_COUNT;i++){
	      colors2[i].NeoRed  =(((unsigned int)colors2[i].NeoRed)   * scale_fixed)   >> 8;
	      colors2[i].NeoGreen=(((unsigned int)colors2[i].NeoGreen) * scale_fixed)   >> 8;
	      colors2[i].NeoBlue =(((unsigned int)colors2[i].NeoBlue)  * scale_fixed)   >> 8;
		  
	   }
}


//************************FUNCION QUE COVIERTE HSV-RGB**************************************************************************

rgb_color HsvToRgb( unsigned char H , unsigned char S, unsigned char V){ //Funcion que retorna una estructura del tipo RgbColor

   rgb_color rgb; 
   unsigned char region; 
   unsigned char remainder;
   unsigned char p ;
   unsigned char q;
   unsigned char t;
   
   if (S == 0) {    
      rgb.NeoRed = V; 
      rgb.NeoGreen = V; 
      rgb.NeoBlue = V; 
      return rgb;
   }
   
   region = H / 43; 
   remainder = (H - (region * 43)) * 6;   
    
   p = (V * (255 - S)) >> 8; 
   q = (V * (255 - ((S * remainder) >> 8))) >> 8; 
   t = ((V * (255 - ((S * (255 - remainder)) >> 8)))>>8);

   switch (region)
   { 
           case 0:
                rgb.NeoRed = V; 
				rgb.NeoGreen = t;  
				rgb.NeoBlue = p; 
                break; 
           case 1:
                 rgb.NeoRed = q; rgb.NeoGreen= V; rgb.NeoBlue = p;
                 break; 
           case 2: 
                 rgb.NeoRed = p; rgb.NeoGreen = V; rgb.NeoBlue = t; 
                 break; 
           case 3:
                rgb.NeoRed = p; rgb.NeoGreen = q; rgb.NeoBlue = V; 
                break; 
           case 4: 
                rgb.NeoRed = t; rgb.NeoGreen = p; rgb.NeoBlue = V; 
                break; 
           default: 
                rgb.NeoRed = V; rgb.NeoGreen = p; rgb.NeoBlue = q; 
                break; 
    } 
 
    return rgb; 
}


//***FUNCION QUE RECETEA EL ARREGLO DE ESTRUCTURAS DESPUES DE UN ENVIO DE DATOS , PARA NO ACARREAR DATOS ANTERIOS A TIRA LED **********

void clearTiraled(rgb_color*colors){
	
	for (unsigned char i = 0; i<= LED_COUNT; i++){
		colors[i]=(rgb_color){0,0,0};
	}
}

void MEMORY(rgb_color *colors,rgb_color*colors2){
	  for (unsigned char i=0;i<=LED_COUNT;i++){
		  
		  colors[i]= colors2[i];
	  }
	
}
