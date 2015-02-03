// ***************************gtvbh njm,********************************
// Project: Master Thesis Project
//
// Author: Janusz Tomaszewski
//
// Module description: PI type regulator of current
// ***********************************************************
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include "lcd.h"

//************************************************************
// 		Zmienne zwiazane z obs³ug¹ klawiatury oraz LCD
//************************************************************

/*Definicje preprocesora*/


#define SETBIT(x,y) (x |=(y))			// definicja operacji ustawiania bitu
#define CLEARBIT(x,y) (x &= (~y))	    // definicja operacji zerowania bitu
#define CHECKBIT(x,y) (x & (y))		    // definicja opercji sprawdzania wartosci bitu

#define KLAWISZ_F 0x01                  // definicja bitu 1 bajtu znaczników klawiatury
#define KLAWISZ_N 0x02					// definicja bitu 2 bajtu znaczników klawiatury
#define KLAWISZ 0x04					// definicja bitu 3 bajtu znaczników klawiatury

/*Definicje zmiennych*/

unsigned char uc_l_obiegow;			  // Licznik obiegów pêtli
unsigned char uc_kolumna;             // Numer kolumny
unsigned char uc_l_spowalniajacy;	  // Wartosc licznika spowalniajacego
unsigned char uc_nr_przycisku;		  // Numer naciœniêtego przycisku
unsigned  int ui_liczba_w;    	      // Liczba wprowadzana
unsigned char uc_klawisze=0xFF;   	  // Bajt znaczników logiki klawiatury
	 	   char c_buffer_n[5];    	  // Bufor do przechowywania wyniku konwersji z "unsigned int" na ASCII
		    int i_war_wy, i_war_cof;    // Zmienne waruku wyjœcia i warunku cofniêcia
unsigned  int ui_Ir_ogr;			  // Ograniczenie pr¹du rozruchu
unsigned  int ui_Ih_ogr;              // Ograniczenie pr¹du hamowania
        float f_I_pom;

			 int blad;
			 int war_przejscia;
			 int reset;
			
/*Zmienne zwiazane z regulatorem PI */

		  float f_Ie;
  		  float f_I_zad;
		  float f_S;
		  float f_Ie_1;
		  float f_I_1;
 		  float f_Innt;
		  float f_Ii;
		  float f_Ip;
		  float f_I_wy;

/*Nastawy dla ROZRUCHU */

		 float f_kp_r = 0.9;
		 float f_ki_r = 0.01;

/*Nastawy dla HAMOWANIA*/

		 float f_kp_h = 1.9;  
		 float f_ki_h = 1.5;

/* Zmienne kontroli PWM	*/		

volatile unsigned int ui_PWM;
volatile float f_I_pwm;


		  int i, j, k, licz, r_jazdy, czas, czas_2;
		  float suma;

//============================================================
//                 	  PROTOTYPY FUNKCJI
//============================================================
	
	int Wybieg(void);
	int Rozruch(void);
	int Hamowanie(void);
	int Klawiatura(void);
   int Wychylenie_Nastawnika(void);
		
	int Przetwornik_ADC(int Kanal_ADC);
	
  void Inicjalizacja_CPU(void);
  void Pobierz_Wartosci(void);

  void wyswietl_float(float liczba);

 float Prad_Pomierzony(void);

 float Prad_Zadany(int Iogr);
	
 float Regulator_PI(int I_ogr, float I_pom, float kp, float ki);

//------------------------------------------------------------


int (*wskf_int[5])(void);
int (*wskf_P_ADC)(int);
void (*wskf_void[2])(void);
void (*wskf_w_float)(float);
float (*wskf_P_Pom)(void);
float (*wskf_P_Zad)(int);
float (*wskf_R_PI)(int, float, float, float);


//============================================================
//                 		PROGRAM G£ÓWNY
//============================================================

int main(void)
{

	wskf_int[0] = &Wybieg;
	wskf_int[1] = &Rozruch;
	wskf_int[2] = &Hamowanie;
	wskf_int[3] = &Klawiatura;
	wskf_int[4] = &Wychylenie_Nastawnika;
	
	wskf_P_ADC = &Przetwornik_ADC;
	
	wskf_void[0] = &Inicjalizacja_CPU;
	wskf_void[1] = &Pobierz_Wartosci;
	
	wskf_w_float = &wyswietl_float;
	
	wskf_P_Pom = &Prad_Pomierzony;
	
	wskf_P_Zad = &Prad_Zadany;
	
	wskf_R_PI = &Regulator_PI;

	while(1)
	{	
	
//		Inicjalizacja_CPU();
		(*wskf_void[0])();
		
		if(!reset)
		{
			lcd_clrscr();

			lcd_puts("   Regulator\n");
			lcd_puts("     pradu  ");
			_delay_ms(2500);
		}

	   sei(); 								//wlaczenie obslugi przerwan
 	   reset = 0;
 	   r_jazdy = 0;


//		Pobierz_Wartosci();
		(*wskf_void[1])();

		war_przejscia = 0;
		
		r_jazdy = 1;
	
	   blad = 0;
	

   	while(!blad)
   	{
     	      	
	      lcd_clrscr();
      	lcd_puts("     WYBIEG");
      	blad = Wybieg();
      	
      	war_przejscia = 0;


      	if(bit_is_set(PIND,0) && !bit_is_set(PIND,1) && war_przejscia != 2)
      	{
//     	   	blad = Rozruch();
         	blad = (*wskf_int[1])();
         	
      		war_przejscia = 1;
      	}

		 	if(bit_is_set(PIND,1) && !bit_is_set(PIND,0) && war_przejscia != 1)
      	{
//      	 blad = Hamowanie();
    	     blad = (*wskf_int[2])();
      	
      		war_przejscia = 2;
      	}
   	}
	
		OCR1A = 0;
	
		if(!reset)
	   {
			lcd_clrscr();
			lcd_puts("Za duzy prad!!!");
			_delay_ms(2500);
	   }
	
	}

	return 0;
}

//============================================================
//                 	DEFINICJE FUNKCJI
//============================================================


void Inicjalizacja_CPU(void)
{

//************************************************************
//						Konfiguracja portów I/O
//************************************************************

// DDRx 1 - wyjscie wowczas PORTx ustawiony jako 1,
// 0 - wejscie PORTx ustawiony jako 0,

/*ADC PA0, PA1*/
	DDRA = 0x00;					// bity 0 - 7 jako wejœæia,
//	PORTA = 0x00;

/*Klawiatura*/
	DDRB = 0x0F;					// bity 0 - 3 jako wyjœcia,
										// 4 - 7 jako wejœcia
	PORTB = 0xFF;					// wejœcia z podci¹ganiem

/*LCD*/
//	DDRC = 0xFF;					// bity 0 - 7 jako wyjœcia

/*Wyjœcie PWM, T1, T2, Sygna³ R, Sygna³ H, Reset zewnêtrzny*/
	DDRD = 0x70;					// bity 0 - 3 oraz bit 7 jako wejœcia
	PORTD = 0x07; //0x07;					// bity 4 - 6 jako wyjœcia	
										// Podciagniecie pibow 0, 1 i 2	

//************************************************************
//							Konfiguracja ADC
//************************************************************

// Wewnetrzne 2.56V Trzeba wylaczyc aby bylo zewnetrzne zrodlo	                           	
//	ADMUX |= _BV(REFS0);			// Wybór Ÿród³a referencyjnego
//	ADMUX |= _BV(REFS1); 		// AREF, Vref wylaczone
										
										// ADLAR = 0 (tryb 10 bitowy)
	
// Je¿eli ADCSRA nie dziala nale¿y u¿yæ nzwy ADCSR.
	
	ADCSRA |= _BV(ADPS1);		// Czêstotliwoœæ taktowania przetwornika	
	ADCSRA |= _BV(ADPS2);		// (1/64 czêstotliwoœci zegara kontrolera)
	
	ADCSRA |= _BV(ADEN);			// Odblokowanie przetwornika (zezwolenie
										// na konwersje).
	
// W programie nale¿y dokonywaæ wyboru pinu, który bêdzie pod³¹czony
// przez multiplekser do przetwornika ADC, i tak:
// ADMUX = 0x00 - Aby wybrac kana³ (PA0) ADC 0 - Pomiar pr¹du
//																z przetwornika LEM
// ADMUX = 0x01 lub ADMUX |= _BV(MUX0); - Aby wybrac kana³ (PA1) ADC 1
//													- Pomiar wychylenia nastawnika
//													  jazdy.

// W programie nale¿y dokonywac w³¹czania bitu rozpoczêcia konwersji
// w trybie pojedynczego wyzwalania, i tak:
// ADCSRA |= _BV(ADSC); - Ustawienie bitu rozpoczynaj¹cego konwersje

//************************************************************
//							Konfiguracja PWM
//************************************************************

// Rozdzielczosc PWM: Rpcpwm = log(TOP + 1)/log(2) = 10 bit,
// Czêstotliwoœæ PWM: focnxpwm = fclk/(2 * N * TOP) = 2kHz.

	TCCR1A |= _BV(COM1A1);		// Clear OC1A/OC1B on compare match,
										// set OC1A/OC1B at BOTTOM.
	
	TCCR1B |= _BV(WGM13);		// Mode 10: PWM, Phase Correct,
	TCCR1A |= _BV(WGM11);      // TOP = ICR1.
	
	TCCR1B |= _BV(CS10);			// clk / 1 - bez preskalera.
	
	ICR1 = 0x7D0;					// ICR1=2000, aby uzyskac 2kHz i 10 bit.
	
//************************************************************
//							Konfiguracja LCD
//************************************************************	

	lcd_init(LCD_DISP_ON);		// Inicjalizacja wyswietlacza
	
//************************************************************
//		Konfiguracja Przerwania INT0 - Reset zewnetrzny PD2
//************************************************************	

	GICR = _BV(INT0);
	
	MCUCR = _BV(ISC01);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Pobierz_Wartosci(void)
{
	
	i_war_wy = 0;
//	i_war_cof = 0;


	while(!i_war_wy)
	{	
		lcd_clrscr();
	
		lcd_puts("I rozruchu:\n");
		lcd_puts("Ir = ");
		
		lcd_command(LCD_DISP_ON_CURSOR);

		while(!i_war_cof)	
   	{
    			
//    		i_war_cof = Klawiatura();   	// Obs³uga klawiszy
    		i_war_cof = (*wskf_int[3])();
    			
    			if( i_war_cof == 2)
    			{
    				i_war_wy = 1;
    			}
    		
		}
		
		i_war_cof = 0;
		
		if(ui_liczba_w > 20 && i_war_wy == 1)
		{
			lcd_clrscr();
			lcd_puts(" Za duzy prad:\n");
			lcd_puts("  (Max 20 A)");
			
			_delay_ms(2500);
			
			i_war_wy = 0;
		}
		ui_Ir_ogr = ui_liczba_w;
		ui_liczba_w = 0;
		
	}
	
		i_war_wy=0;
		while(!i_war_wy)
	{	
		lcd_clrscr();
	
		lcd_puts("I hamowania:\n");
		lcd_puts("Ih = ");
		lcd_command(LCD_DISP_ON_CURSOR);
	
		while(!i_war_cof)
		{
//				i_war_cof = Klawiatura();
				i_war_cof = (*wskf_int[3])();
				
				if( i_war_cof == 2)
    			{
    				i_war_wy = 1;
    			}
		}
		i_war_cof = 0;
		
		if(ui_liczba_w > 17 && i_war_wy == 1)
		{
			lcd_clrscr();
			lcd_puts(" Za duzy prad:\n");
			lcd_puts("  (Max 17 A)");
			
			_delay_ms(2500);
						
			i_war_wy = 0;
		}
		lcd_clrscr();
		ui_Ih_ogr = ui_liczba_w;
		ui_liczba_w = 0;
   }

	lcd_command(LCD_DISP_ON);
	lcd_clrscr();
	
	utoa(ui_Ir_ogr, c_buffer_n, 10);
	lcd_puts("Ir = ");
	lcd_puts(c_buffer_n);
	lcd_puts(" A\n");
	
	utoa(ui_Ih_ogr, c_buffer_n, 10);
	lcd_puts("Ih = ");
	lcd_puts(c_buffer_n);
	lcd_puts(" A");
	
	_delay_ms(2500);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int Klawiatura(void)
{
  	uc_l_obiegow++;              						// Licznik obiegów pêtli

	if(uc_l_obiegow == 20)
    {
		uc_kolumna++;            						// Licznik kolumny
		uc_l_obiegow = 0;
		if(uc_kolumna == 4)            				// Przepe³nienie licznika obiegów pêtli
	    {
	      uc_kolumna = 0;
	      uc_l_spowalniajacy++;
	
			if(uc_l_spowalniajacy == 5) 				// Licznik spowalniaj¹cy
	        {
	          SETBIT(uc_klawisze, KLAWISZ);		// Flaga naciœniêcia klawisza
	          SETBIT(uc_klawisze, KLAWISZ_N);		// Flaga naciœniêcia klawisza numerycznego
	          SETBIT(uc_klawisze, KLAWISZ_F);		// Flaga naciœniêcia klawisza funkcyjnego
	          uc_nr_przycisku = 0;
	          uc_l_spowalniajacy = 0;
	        }
	    }
	
		PORTB = 0xFF;
		PORTB &= ~_BV(uc_kolumna);
    }
	
	if ((PINB >> 4) != 15)								// Wykrycie wciœniêcia klawisza
    {		
		CLEARBIT(uc_klawisze, KLAWISZ);				// Ustawienie flagi naciœniêcia klawisza
		uc_l_spowalniajacy = 0;
    }
	if((uc_kolumna == 0))								// Wykrycie klawisza 1-szej kolumny
    {
      if((PINB >> 4) == 14)
        {
				uc_nr_przycisku = 1;				     	// Wciœniêto przycisk "1"
        }
      if((PINB >> 4) == 13)	
        {
				uc_nr_przycisku = 4;						// Wciœniêto przycisk "4"
        }
      if((PINB >> 4) == 11)
        {
				uc_nr_przycisku = 7;						// Wciœniêto przycisk "7"
        }
      if((PINB >> 4) == 7)
        {
			uc_nr_przycisku = 15;						// Wciœniêto przycisk "15" (*)
        }
    }
  if((uc_kolumna == 1))									// Wykrycie klawisza 2-giej kolumny
    {
      if((PINB >> 4) == 14)
        {
				uc_nr_przycisku = 2;						// Wciœniêto przycisk "2"
        }
      if((PINB >> 4) == 13)
        {
				uc_nr_przycisku = 5;						// Wciœniêto przycisk "5"
        }
      if((PINB >> 4) == 11)
        {
				uc_nr_przycisku = 8;						// Wciœniêto przycisk "8"
		  }
      if((PINB >> 4) == 7)
        {
				uc_nr_przycisku = 0;						// Wciœniêto przycisk "0"
        }
    }
  if((uc_kolumna == 2))									// Wykrycie klawisza 3-ciej kolumny
    {
      if((PINB >> 4) == 14)
        {
				uc_nr_przycisku = 3;						// Wciœniêto przycisk "3"
        }
      if((PINB >> 4) == 13)
        {
				uc_nr_przycisku = 6;						// Wciœniêto przycisk "6"
        }
      if((PINB >> 4) == 11)
        {
				uc_nr_przycisku = 9; 			     	// Wciœniêto przycisk "9"
        }
      if((PINB >> 4) == 7)
        {
				uc_nr_przycisku = 16;					// Wciœniêto przycisk "16" (#)
        }
    }	
  if((uc_kolumna == 3))									// Wykrycie klawisza 4-tej kolumny
    {
      if((PINB >> 4) == 14)
        {
				uc_nr_przycisku = 11;					// Wciœniêto przycisk "11" (A)
        }
      if((PINB >> 4) == 13)
        {
				uc_nr_przycisku = 12;					// Wciœniêto przycisk "12" (B)
        }
      if((PINB >> 4) == 11)
        {
				uc_nr_przycisku = 13;					// Wciœniêto przycisk "13" (C)
        }
      if((PINB >> 4) == 7)
        {
				uc_nr_przycisku = 14;					// Wciœniêto przycisk "14" (D)
        }
    }
  		
  if((uc_nr_przycisku == 11) && (CHECKBIT(uc_klawisze, KLAWISZ_F)))		// Zatwierdzenie wpisanej wrtoœci - Przycisk (A)
  {
		CLEARBIT(uc_klawisze, KLAWISZ_F);       									// Ustawienie flagi wybrania klawisza funkcyjnego
		return 2;
  }
  if((uc_nr_przycisku == 13) && (CHECKBIT(uc_klawisze, KLAWISZ_F))) 		// Powtórzenie wpisu - Przycisk (C)
  {
		CLEARBIT(uc_klawisze, KLAWISZ_F);											// Ustawienie flagi wybrania klawisza funkcyjnego
		return 1;
  }
  if(!(CHECKBIT(uc_klawisze, KLAWISZ)) && (CHECKBIT(uc_klawisze, KLAWISZ_N)) && (uc_nr_przycisku < 10))
  {
		utoa(uc_nr_przycisku, c_buffer_n, 10);
		lcd_puts(c_buffer_n);
		ui_liczba_w = ui_liczba_w * 10 + uc_nr_przycisku;						// Obliczanie wprowadzanej liczby
		CLEARBIT(uc_klawisze, KLAWISZ_N);											// Ustawienie flagi zadzia³ania klawisza numerycznego
  }

	return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int Przetwornik_ADC(int Kanal_ADC)
{

	ADMUX &= ~_BV(MUX0) &~_BV(MUX1) &~_BV(MUX2);
		
	switch(Kanal_ADC) 									// Wybór kana³u
	{
		
		case 0:
					break;
					
		case 1:  ADMUX |= _BV(MUX0);
	            break;
	
		case 2:	ADMUX |= _BV(MUX1);
					break;
				
		case 3:	ADMUX |= _BV(MUX0) | _BV(MUX1);
					break;
					
		case 4:  ADMUX |= _BV(MUX2);
					break;
					
		case 5:	ADMUX |= _BV(MUX0) | _BV(MUX2);
					break;
		
		case 6:	ADMUX |= _BV(MUX1) | _BV(MUX2);
					break;
					
		case 7:	ADMUX |= _BV(MUX0) | _BV(MUX1) | _BV(MUX2);
					break;
	}
	
	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADSC);         						// Rozpoczêcie przetwarzania
	while(bit_is_set(ADCSRA,ADSC))					// Oczekiwanie na zakoñczenie przetwarzania
	{
	}

	return (ADCW);		
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int Wychylenie_Nastawnika(void)
{
	int wychylenie;
	
//	wychylenie = Przetwornik_ADC(1);
	wychylenie = (*wskf_P_ADC)(1);
		
	if( wychylenie < 120)
	{
		wychylenie = 120;
	}
	
	if( wychylenie > 620)
	{
		wychylenie = 620;
	}
	
	return (wychylenie - 120);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

float Prad_Zadany(int Iogr)
{
//	return (Wychylenie_Nastawnika() * Iogr * 0.002);
    return ((*wskf_int[4])() * Iogr * 0.002);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

float Prad_Pomierzony(void)
{
	int V_pom;
	float Vref, R, P_LEM;
	
	Vref = 5;
  	R = 300;                 			// Rezystor na wyjsciu LEMa  R = 300 Ohm
	P_LEM = 0.0005;          			// Przek³adnia LEM 1:2000;
	
//	V_pom = Przetwornik_ADC(0);
	V_pom = (*wskf_P_ADC)(0);
	
	return ((V_pom * Vref / 1024) / (P_LEM * R));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int Wybieg(void)
{
	ui_PWM = 0;
	suma = 0;
	licz = 0;
	
	OCR1A = ui_PWM;
	
//	f_I_pom = Prad_Pomierzony();
	f_I_pom = (*wskf_P_Pom)();
	
	if(f_I_pom >= 23)
	{
		return 1;
	}
	
	while((bit_is_set(PIND,1) && bit_is_set(PIND,0)) || (!bit_is_set(PIND,1) && !bit_is_set(PIND,0)))
	{
		if(reset)
		{
			return 1;
		}
	}
	
	return 0;	
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int Rozruch(void)
{
   f_S = 0;
	f_I_1 = 0;
	f_Ie_1 = 0;
	
	lcd_clrscr();
	lcd_puts("    ROZRUCH");
	
	PORTD &= ~_BV(PORTD6);								// Wylaczenie T1
	_delay_ms(200);										// Opoznienie aby nie przelaczac jednoczesnie tranzystorow
			
	do
	{
	}
	while((*wskf_P_Pom)());
// 	while( Prad_Pomierzony() );


	PORTD |= _BV(PORTD4);				// Wlaczenie T2
	
	while(bit_is_set(PIND,0) && !bit_is_set(PIND,1))
	{
//		f_I_pom = Prad_Pomierzony();
		f_I_pom = (*wskf_P_Pom)();
		
		if(f_I_pom >= 23)
		{
			return 1;
		}
		
		suma += f_I_pom;
		
		if(licz == 500)
		{

			lcd_clrscr();
         lcd_puts("Izad = ");

//		wyswietl_float(Prad_Zadany(ui_Ir_ogr));
		(*wskf_w_float)((*wskf_P_Zad)(ui_Ir_ogr));

			
			lcd_puts(" A\n");
			
			suma = suma/500;

			lcd_puts("Ipom = ");
			
//			wyswietl_float(suma);
			(*wskf_w_float)(suma);

			lcd_puts(" A\n");
			
			licz = 0;
			suma = 0;
			
		}
		
//		f_I_pwm = Regulator_PI(ui_Ir_ogr, f_I_pom, f_kp_r, f_ki_r);
		f_I_pwm = (*wskf_R_PI)(ui_Ir_ogr, f_I_pom, f_kp_r, f_ki_r);
		
		if(f_I_pwm < 0)
		{
			f_I_pwm = 0;
		}
		
		f_I_pwm *= 100;
		f_I_pwm += 0.5;
		
		ui_PWM = (unsigned int)f_I_pwm;
		
		OCR1A = ui_PWM;
	
		licz++;
			
	}
	
	return 0;
	
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int Hamowanie(void)
{
	f_S = 0;
	f_I_1 = 0;
	f_Ie_1 = 0;
	
	lcd_clrscr();
	lcd_puts("    HAMOWANIE");
	
	PORTD &= ~_BV(PORTD4);				// Wylaczenie T2

	_delay_ms(200);						// Opoznienie aby nie przelaczac jednoczesnie tranzystorow
	
	do
	{
	}
	while((*wskf_P_Pom)());


	
	PORTD |= _BV(PORTD6);				// Wlaczenie T1
	
	while(bit_is_set(PIND,1) && !bit_is_set(PIND,0))
	{
//		f_I_pom = Prad_Pomierzony();
		f_I_pom = (*wskf_P_Pom)();
		
		if(f_I_pom >= 23)
		{
				return 1;
		}
		
		suma += f_I_pom;
		
		if(licz == 500)
		{
         lcd_clrscr();
         lcd_puts("Izad = ");
//		wyswietl_float(Prad_Zadany(ui_Ih_ogr));
		(*wskf_w_float)((*wskf_P_Zad)(ui_Ih_ogr));

		lcd_puts(" A\n");
			
		suma = suma/500;

		lcd_puts("Ipom = ");
			
//		wyswietl_float(suma);
		(*wskf_w_float)(suma);

		lcd_puts(" A");
			
		licz = 0;
		suma = 0;
			
		}
	
//		f_I_pwm = Regulator_PI(ui_Ih_ogr, f_I_pom, f_kp_h, f_ki_h);
		f_I_pwm = (*wskf_R_PI)(ui_Ih_ogr, f_I_pom, f_kp_h, f_ki_h);

		if(f_I_pwm < 0)
		{
			f_I_pwm = 0;
		}
		
		f_I_pwm *= 100;
		f_I_pwm += 0.5;
		
		ui_PWM = (unsigned int)f_I_pwm;

		OCR1A = ui_PWM;
		
		licz++;
	}
	
	return 0;
	
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIGNAL (SIG_INTERRUPT0)
{
	OCR1A = 0;
	
	if(r_jazdy)
	{	
		lcd_clrscr();
		lcd_puts("      RESET \n");
		lcd_puts("   ZEWNETRZNY");

	
		while((bit_is_set(PIND,1) && !bit_is_set(PIND,0)) || (!bit_is_set(PIND,1) && bit_is_set(PIND,0)));
		reset = 1;
	}
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void wyswietl_float(float liczba)
{
	int zmie;

	zmie = liczba;
	utoa(liczba, c_buffer_n, 10);

	lcd_puts(c_buffer_n);
	lcd_puts(".");

	zmie = (liczba - zmie)*100;
	if(zmie < 10)
	{
		lcd_puts("0");
	}

	utoa(zmie, c_buffer_n, 10);
	lcd_puts(c_buffer_n);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


float Regulator_PI(int I_ogr, float I_pom, float kp, float ki)
{
	f_Ie = 0;
//	f_I_zad = Prad_Zadany(I_ogr);
	f_I_zad = (*wskf_P_Zad)(I_ogr);
		
	f_Ie = f_I_zad - I_pom;

	f_Ip = f_Ie * kp;
	
	if(f_I_wy < 20)
	{
		f_S = f_S + f_Ie_1;
		f_Ii = (f_Ie + f_S)/2 * ki; //+ f_I_1;
	}
	else
	{
		czas++;
		
		if(czas == 400)
		{
			czas_2++;
			if(czas_2 == 1000)
			{
				czas_2 = 0;
				f_Ii = 0;
			}
			czas = 0;
		}
			
	}
	
	f_I_wy = f_Ip + f_Ii;
	
	if(f_I_wy > 20)
	{
		f_I_wy = 20;
	}
	
	if(f_I_wy < 0)
	{
		f_I_wy = 0;
	}
	
	f_I_1 = (f_Ie + f_S)/2 * ki;
	
	f_Ie_1 = f_Ie;
	
	return (f_I_wy);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



