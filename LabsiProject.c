/* LABSI 2015
 * 
 * Autonomous Guided Vehicle (AGV)
 *
 * 1120484 - João Brandão
 * 1120519 - José Pedro Silva
 * Turma: 3DD
 *
 * RESUMO:
 * Trabalho desenvolvido no âmbito da unidade curricular de Laboratório de Sistemas.
 * Este trabalho consiste num veículo que, automatizado, percorre um determinado percurso seguindo 
 * uma linha preta numa superfície branca.
 */ 



/************************************************************************/
/*			BIBLIOTECAS NECESSÁRIAS AO FUNCIONAMENTO DO PROGRAMA		*/
/************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>


/************************************************************************/
/*						CONFIGURAÇÕES GLOBAIS							*/
/************************************************************************/
#define BRANCO 0								//Define branco como 0 para comparar com flag de leitura
#define PRETO 1									//Define preto como 1 para comparar com flag de leitura

//Definição dos pinos dos leds
#define LED_ESQUERDA PD5
#define LED_MEIO PD6
#define LED_DIREITA PD7

//Definição dos pinos do motor esquerdo
#define MTR_ESQ_FRT PD0							//LM298 | INPUT 1
#define MTR_ESQ_TRS PD1							//LM298 | INPUT 2

//Definição dos pinos do motor direito
#define MTR_DIR_FRT PD2							//LM298 | INPUT 3
#define MTR_DIR_TRS PD4							//LM298 | INPUT 4

//Velocidades (valores a colocar nos OCRxy)
#define RAPIDO 80
#define RAPIDO_ESQ 50							//É necessário uma vez que o motor da esquerda é mais lento
#define MEDIO 90
#define LENTO 100


volatile unsigned char counter_50 = 50;			//Contador para decrementar de 10 em 10 ms (50*10ms=500ms)
volatile unsigned char leituraAd = 0;			//Variável para guardar leitura efetuado pelo AD

//Estrutura de Flags
struct{
	unsigned char blackHole;					//Flag que indica quando os sensores lêm tudo preto
	//Led a piscar
	volatile unsigned char atraso;				//Flag para verificar se passaram 500 ms
	unsigned char led_io;						//Flag para verificar se o led está ON/OFF
	//Sensores
	volatile unsigned char esquerda;
	volatile unsigned char meio;
	volatile unsigned char direita;
	//Movimento
	unsigned char frente;
	unsigned char esq;
	unsigned char dir;
	unsigned char esq90;
	unsigned char dir90;
}flag;



/************************************************************************/
/*						Função de Configuração							*/
/************************************************************************/
void config()
{

	DDRC = 0b00000000;							//Define Porto C como entradas
	DDRB = 0b00001001;							//Define pinos 0 e 3 como saidas para LED a piscar e Motor da Esquerda respetivamente
	
	//Configuração do Timer/Counter0
	TCCR0A = 0b00000010;						//Normal Port Operation, OC0A e OC0B disconnected, CTC MODE
	TCCR0B = 0b00000011;						//Prescaler 64
	OCR0A = 155;								//Valor para obter 10 ms
	sei();										//Ativa interrupções globais
	TIMSK0 = 0b00000010;						//Ativa OCIE0A: ativa Timer/Counter0 Compare Match A Interrupt
	
	//Configuração LED a piscar (PC5)
	DDRD = 0b11101000;							//Define pino 3 como saida para Motor da Direita e pinos 5, 6 e 7 para LED de indicação das
												//leituras do AD
	
	//Configuração do Timer/Counter2
	TCCR2A = 0b11110001;						//Normal Port Operation OC2A, Set OC2B on compare match when up-counting
												//Clear OC2B on compare match when down-counting, PWM Phase Correct TOP=OCRA e update do OCRA no TOP
	TCCR2B = 0b00000011;						//Prescaler 32, Freq.=(1 MHz)/(32*510)=61,3 Hz, T=16,3 ms	
	
	//Configuração ADC
	ADMUX = 0b01100000;							//AVCC with external capacitor at AREF pin, ADLAR result is left adjusted, ADC0 enable
	ADCSRA = 0b11101011;						//Enable ADC, Fator de Divisão de 8 para obter 125 kHz de frequência de clock [50, 200] kHz
	ADCSRB = 0b00000011;						//Timer/Counter0 compare match A
}

//Interrupção 0
ISR(TIMER0_COMPA_vect)
{
	if(counter_50!=0)							//Se o contador ainda não tiver chegado a 0
		counter_50--;							//Decrementa-o
	else
	{											//Se o contador chegar a 0
		flag.atraso = 1;						//Ativa a flag_atraso: Passaram 500ms
		counter_50 = 50;						//Reinicia o valor do contador
	}
}

//Leitura dos Sensores CNY70
ISR(ADC_vect){
	leituraAd = ADCH;							//Guarda em "leituraAd" os 8 bits mais significativos da leitura do AD
	
	switch(ADMUX)								//Verifica valor do ADMUX
	{
		case 0b01100000:
			if(leituraAd<50){
				flag.esquerda = 1;				//Sensor da Esquerda leu preto
			}
			else{
				flag.esquerda = 0;				//Sensor da Esquerda leu branco
			}
			ADMUX = 0b01100001;					//Para em seguida testar a leitura do sensor do meio
			break;
		case 0b01100001:
			if(leituraAd<50){
				flag.meio = 1;					//Sensor do Meio leu preto
			}
			else{
				flag.meio = 0;					//Sensor do Meio leu branco
			}
			ADMUX = 0b01100010;					//Para em seguida testar a leitura do sensor da direita
			break;
		case 0b01100010:
			if(leituraAd<50){
				flag.direita = 1;				//Sensor da Direita  leu preto
			}
			else{
				flag.direita = 0;				//Sensor da Direita leu branco
			}
			ADMUX = 0b01100000;					//Para em seguida testar a leitura do sensor da esquerda
			break;
	}
	ADCSRA |= 1<<ADSC;							//Terminando a conversão volta a colocar-se o pino de "Start Conversion" a 1 para voltar
												//a fazer conversões do AD
}




/*-----------------------------------------------------------------------------------------------*/
//Função que liga o LED
void liga_led()
{
	PORTB |= (1<<PB0);							//Liga o led que está no pino PB0
	flag.led_io = 1;							//Ativa a flag_led: led ligado
	flag.atraso = 0;							//Desativa a flag_atraso: Para não alterar estado do led
}
//Função que desliga o LED
void desliga_led()
{						
	PORTB &= ~(1<<PB0);							//Desliga o led que está no pino PB0
	flag.led_io = 0;							//Desativa a flag_led: led desligado
	flag.atraso = 0;							//Desativa a flag_atraso: Para não alterar estado do led
}
/*-----------------------------------------------------------------------------------------------*/



//FRENTE
void forward()
{
	//Ativa movimento dos motores: ambos para a frente
	PORTD &= ~((1<<MTR_ESQ_TRS) | (1<<MTR_DIR_TRS));
	PORTD |= (1<<MTR_ESQ_FRT) | (1<<MTR_DIR_FRT);
	
	//Velocidades dos motores
	OCR2B = RAPIDO;
	OCR2A = RAPIDO_ESQ;
	
	flag.frente = 1;							//Ativa a flag frente
	flag.esq = 0;
	flag.dir = 0;
	flag.esq90 = 0;
	flag.dir90 = 0;
}
//ESQUERDA
void turnLeft()
{
	//Ativa movimento dos motores: Esquerdo para trás, Direito para a frente
	PORTD &= ~((1<<MTR_ESQ_FRT) | (1<<MTR_DIR_TRS));
	PORTD |= (1<<MTR_ESQ_TRS) | (1<<MTR_DIR_FRT);
	
	//Velocidades dos motores
	OCR2B = MEDIO;
	OCR2A = MEDIO;
	
	flag.frente = 0;
	flag.esq = 1;								//Ativa a flag esquerda
	flag.dir = 0;
	flag.esq90 = 0;
	flag.dir90 = 0;
}
//DIREITA
void turnRight()
{
	//Ativa movimento dos motores: Esquerdo para frente, Direito para a trás
	PORTD &= ~((1<<MTR_ESQ_TRS) | (1<<MTR_DIR_FRT));
	PORTD |= (1<<MTR_ESQ_FRT) | (1<<MTR_DIR_TRS);
	
	//Velocidades dos motores
	OCR2B = MEDIO;
	OCR2A = MEDIO;
	
	flag.frente = 0;
	flag.esq = 0;
	flag.dir = 1;								//Ativa a flag direita
	flag.esq90 = 0;
	flag.dir90 = 0;
}
//ESQUERDA 90º
void turnLeft90()
{
	//Ativa movimento dos motores: Esquerdo para trás, Direito para a frente
	PORTD &= ~((1<<MTR_ESQ_FRT) | (1<<MTR_DIR_TRS));
	PORTD |= (1<<MTR_ESQ_TRS) | (1<<MTR_DIR_FRT);
	
	//Velocidades dos motores
	OCR2B = LENTO;
	OCR2A = LENTO;
	
	flag.frente = 0;
	flag.esq = 0;
	flag.dir = 0;
	flag.esq90 = 1;								//Ativa a flag esquerda 90º
	flag.dir90 = 0;
}
//DIREITA 90º
void turnRight90()
{
	//Ativa movimento dos motores: Esquerdo para frente, Direito para a trás
	PORTD &= ~((1<<MTR_ESQ_TRS) | (1<<MTR_DIR_FRT));
	PORTD |= (1<<MTR_ESQ_FRT) | (1<<MTR_DIR_TRS);
	
	//Velocidades dos motores
	OCR2B = LENTO;
	OCR2A = LENTO;
	
	flag.frente = 0;
	flag.esq = 0;
	flag.dir = 0;
	flag.esq90 = 0;
	flag.dir90 = 1;								//Ativa a flag direita 90º
}
//STOP
void stop()
{	//Desativa movimento dos motores
	PORTD &= ~((1<<MTR_ESQ_FRT) | (1<<MTR_DIR_TRS) | (1<<MTR_ESQ_TRS) | (1<<MTR_DIR_FRT));
	
	//Velocidades dos motores
	OCR2B = 0;
	OCR2A = 0;
												//Desativa todas as flags
	flag.frente = 0;
	flag.esq = 0;
	flag.dir = 0;
	flag.esq90 = 0;
	flag.dir90 = 0;
}


/************************************************************************/
/*								MAIN									*/
/************************************************************************/
int main(void)
{
	config();									//Faz as configurações do Portos a usar, Timer/Counters e Interrupções
	
    while(1)
    {	
		// 0 | 0 | 0
		if(flag.esquerda==BRANCO && flag.meio==BRANCO && flag.direita==BRANCO)
		{
			PORTD &= ~((1<<LED_ESQUERDA)|(1<<LED_MEIO)|(1<<LED_DIREITA));	//Clear bits
			if(flag.frente == 1){
				if(flag.blackHole == 1){
					turnLeft90();				//Quando lê tudo branco depois de tudo preto (fim de pista em T) dá a volta pra esquerda
					flag.blackHole = 0;
				}
				else
					PORTD &= ~((MTR_ESQ_FRT) | (MTR_ESQ_TRS) | (MTR_DIR_FRT) | (MTR_DIR_TRS));			//Quando lê tudo branco sem antes tudo preto pára (fim de pista)
			}
			if(flag.esq == 1)
				turnLeft();
			if(flag.dir == 1)
				turnRight();
			if(flag.esq90 == 1)
				turnLeft90();
			if(flag.dir90 == 1)
				turnRight90();
		}
		
		// 0 | 0 | 1
		if(flag.esquerda==BRANCO && flag.meio==BRANCO && flag.direita==PRETO)
		{
			turnRight();
			PORTD &= ~((1<<LED_ESQUERDA)|(1<<LED_MEIO));					//Clear Bits
			PORTD |= (1<<LED_DIREITA);										//Set Bits
			flag.blackHole = 0;
		}
		
		// 0 | 1 | 0
		if(flag.esquerda==BRANCO && flag.meio==PRETO && flag.direita==BRANCO)
		{
			forward();
			PORTD &= ~((1<<LED_ESQUERDA)|(1<<LED_DIREITA));					//Clear bits
			PORTD |= (1<<LED_MEIO);											//Set bits
			flag.blackHole = 0;
		}
		
		// 0 | 1 | 1
		if(flag.esquerda==BRANCO && flag.meio==PRETO && flag.direita==PRETO)
		{
			turnRight90();
			PORTD &= ~((1<<LED_ESQUERDA));									//Clear bits
			PORTD |= (1<<LED_MEIO)|(1<<LED_DIREITA);						//Set bits
			flag.blackHole = 0;
		}
		
		// 1 | 0 | 0
		if(flag.esquerda==PRETO && flag.meio==BRANCO && flag.direita==BRANCO)
		{
			turnLeft();
			PORTD &= ~((1<<LED_MEIO)|(1<<LED_DIREITA));						//Clear bits
			PORTD |= (1<<LED_ESQUERDA);										//Set bits
			flag.blackHole = 0;
		}
		
		// 1 | 0 | 1
		if(flag.esquerda==PRETO && flag.meio==BRANCO && flag.direita==PRETO)
		{
			forward();
			PORTD &= ~((1<<LED_MEIO));										//Clear bits
			PORTD |= (1<<LED_ESQUERDA)|(1<<LED_DIREITA);					//Set bits
			flag.blackHole = 0;
		}
		
		// 1 | 1 | 0
		if(flag.esquerda==PRETO && flag.meio==PRETO && flag.direita==BRANCO)
		{
			turnLeft90();
			PORTD &= ~((1<<LED_DIREITA));									//Clear bits
			PORTD |= (1<<LED_ESQUERDA)|(1<<LED_MEIO);						//Set bits
			flag.blackHole = 0;
		}
		
		// 1 | 1 | 1
		if(flag.esquerda==PRETO && flag.meio==PRETO && flag.direita==PRETO)
		{
			forward();
			PORTD |= (1<<LED_ESQUERDA)|(1<<LED_MEIO)|(1<<LED_DIREITA);		//Set bits
			flag.blackHole = 1;
		}
		


		if(flag.atraso == 1)				//Se já tiverem passado 500 ms
		{
			if(flag.led_io == 0)
				liga_led();					//Se o led estiver desligado ao fim de 500 ms, liga-o
			else
				desliga_led();				//Se o led estiver ligado ao fim de 500 ms, desliga-o
		}
    }
	return 0;
}