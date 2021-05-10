#define F_CPU 3333333
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU / (16 * (float)BAUD_RATE)) * 64)

//CONTROL REGISTER B
#define RXEN 7
#define TXEN 6
#define SFDEN 4
#define ODME 3
#define RXMODE1 2
#define RXMODE0 1
#define MPCM 0
//CONTROL REGISTER C
#define CMODE1 7
#define CMODE0 6
#define PMODE1 5
#define PMODE0 4
#define SBMODE 3
#define CHSIZE2 2
#define CHSIZE1 1
#define CHSIZE0 0
//PIN CONTROL ON PINCTRL
#define INVEN 7
#define PULLUPEN 3
#define ISC2 2
#define ISC1 1
#define ISC0 0
//INTFLAGS
#define INT7 7
#define INT6 6
#define INT5 5
#define INT4 4
#define INT3 3
#define INT2 2
#define INT1 1
#define INT0 0
#define maxperiod 60000
#define period 15000
#include "includes/oled.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/sleep.h>

bool rad=0, sleep=0,wokeUp=0,DataReady = 0, clicks=1, clicks2, stanPrzycisk, PB5_STATE=0, PC5_STATE=0, DisplayReady=0, changeMode = 0;
uint8_t bounceFlag, bounceFlag2, bounceFlag3, flag=0, a, b, c, br=0x00, brw=0, i=0, j=1, Mode = 0, refresh=0;
uint16_t buf[32], buf2[6], g = 0, ActualTime = 0, now = 0, PastTime = 0, mnoz = 0,cpm=0;
unsigned char nap0[]="Power On. Wait", nap1[]="Power Off.", nap3[]="Press SW0 to start...";
unsigned char napisjasn[]="DIM:", jasn[]="", spacja[]=" ";
unsigned char CLRline[]="              ";
unsigned char PM10[]="PM10 :", PM1[]="PM0.1:", PM25[]="PM2.5:", czasteczki[]="Particles ug/m^3";
unsigned char um03[]="0.3um:", um05[]="0.5um:", um10[] = "10 um:", wdech[]="Particles in 1 breath";
unsigned char release[]="Release to reset";
unsigned char RelToMode[]="Release - change mode";
unsigned char sivert[] = "uS/min: ";
unsigned char radiation[] = "Radiation";

unsigned char tab[5]={0,0,0,0,0};
unsigned char tab2[6]={0,0,0,0,0,0};
unsigned char geiger[4]={0,0,0,0};

void initAll(void){
	
	VPORTB.OUT &= ~PIN0_bm; //RESET PB0 - init low level
	VPORTB.DIR |= PIN0_bm; //RESET PB0 - init low level
	
	VPORTB.OUT &= ~(1 << 1); //DC PB1 - init low level
	VPORTB.DIR |= (1 << 1); //DC PB1 - init low level
	
	VPORTB.OUT &= ~(1 << 4);//CS PB4 - init low lvl
	VPORTB.DIR |= (1 << 4);//CS PB4 - init low lvl
	
	VPORTB.DIR &= ~PIN5_bm; //SW0
	PORTB.PIN5CTRL =  PORT_PULLUPEN_bm;
	
	VPORTC.DIR &= ~PIN5_bm; //SW1
	PORTC.PIN5CTRL =  PORT_PULLUPEN_bm;

	VPORTB.OUT |= PIN7_bm; //Sensor_SET
	VPORTB.DIR |= PIN7_bm;	
	
	PORTA.PIN6CTRL = PORT_PULLUPEN_bm;
	VPORTA.DIR &= ~PIN6_bm;
	
	PORTA.PIN7CTRL = PORT_ISC_BOTHEDGES_gc;
	PORTA.DIR &= ~PIN7_bm;
}
void USART0_init(void){
	VPORTB.DIR &= ~PIN3_bm; //USTAW JAKO INPUT
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);
	
	USART0.CTRLA = USART_RXCIF_bm;

	USART0.CTRLB = (1<<RXEN) | (0<<TXEN) | (1<<SFDEN) | (0<<ODME) | (0<<MPCM)
	| (0<<RXMODE1) | (0<<RXMODE0);
	
	USART0.CTRLC = (0<<CMODE1) | (0<<CMODE0) | (0<<PMODE1) | (0<<PMODE0)
	| (0<<SBMODE) | (0<<CHSIZE2) | (1<<CHSIZE1) | (1<<CHSIZE0);
	
	
}
void SPI0_init(void){
	
	PORTC.DIR &= ~(1 << 1);
	PORTC.PIN1CTRL &= ~PORT_PULLUPEN_bm;
	PORTMUX.CTRLB |= PORTMUX_SPI0_bm;
	
	PORTC.OUT &= ~(1 << 2);
	PORTC.DIR |= (1 << 2);
	PORTMUX.CTRLB |= PORTMUX_SPI0_bm;
	
	PORTC.OUT &= ~(1 << 0);
	PORTC.DIR |= (1 << 0);
	PORTMUX.CTRLB |= PORTMUX_SPI0_bm;
	
	SPI0.CTRLA = 0 << SPI_CLK2X_bp | 0 << SPI_DORD_bp | 1 << SPI_ENABLE_bp 
				| 1 << SPI_MASTER_bp | SPI_PRESC_DIV4_gc;
}
void timerISRinit(){

	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; //enable overflow interrupt
	// set Normal mode
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
	// disable event counting 
	TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
	// set the period ~*
	TCA0.SINGLE.PER = 52; //dla ~5ms
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc	| TCA_SINGLE_ENABLE_bm; //start timer

}
void Oled_BrLVL(){ //OLED BRIGHTNESS LEVEL CONTROL
	j++;
	if(j==4) j=1;
	switch(j){
		case 1:{
			br=0x00;
			brw=1;
			break;
		}
		case 2:{
			br=0x50;
			brw=2;
			break;
		}
		case 3:{
			br=0xFF;
			brw=3;
			break;
		}
	}
	OLED_CMD(0x81); //rejestr kontrastu
	OLED_CMD(br); //jasnosc 0-255
	jasn[1]=0;
	jasn[0]=0x30|brw%10;
	OLED_6X8_String(95,1,napisjasn);
	OLED_6X8_String(120,1,jasn);
	_delay_ms(500);
	OLED_6X8_String(95,1,CLRline);
	DisplayReady=1;
}
void OLED_OffOn(){
	clicks = (!clicks);
	if(clicks){
		VPORTB.OUT &= ~PIN7_bm;	//WYLACZENIE PM1003
		OLED_FILL(0x00);
		OLED_6X8_String(20,2,nap1);
		_delay_ms(2500);
		OLED_CMD(0xae); //WYLACZENIE EKRANU
		clicks2=0;
		sleep=1;
		PORTB.PIN5CTRL |= PORT_ISC_LEVEL_gc; //TURN ON PA6 INTERRUPT TO WAKE UP FROM SLEEP_MODE
		PORTA.PIN7CTRL &= ~PORT_ISC_BOTHEDGES_gc;
		set_sleep_mode(SLEEP_MODE_PWR_DOWN); //SLEEP PROCEDURE *START
		cli();
		sleep_enable();
		sei();
		sleep_cpu();//SLEEP PROCEDURE *END
		
		}else{
		VPORTB.OUT |= PIN7_bm; //WLACZENIE PM1003
		PORTB.PIN5CTRL &= ~PORT_ISC_LEVEL_gc; //TURN OFF PA6 INTERRUPT FOR RESET BUTTON TO BE AVAILABLE
		PORTA.PIN7CTRL |= PORT_ISC_BOTHEDGES_gc;
		OLED_FILL(0x00);
		OLED_CMD(0xaf); //WLACZENIE EKRANU
		OLED_6X8_String(20,2,nap0);
		_delay_ms(4500);
		clicks2=1;
		refresh=1;
	}
	OLED_FILL(0x00);
	
	
}
void showData(uint8_t tryb){
	switch(tryb){
		case 0:{
			OLED_6X8_String(2,0,czasteczki);
			tab[4]=0;
			tab[3]=0x30|buf2[2]%10;
			tab[2]=0x30|(buf2[2]/10)%10;
			tab[1]=0x30|(buf2[2]/100)%10;
			tab[0]=0x30|(buf2[2]/1000)%10;
			OLED_6X8_String(5,2,PM10);
			OLED_6X8_String(45,2,tab);
			tab[4]=0;
			tab[3]=0x30|buf2[1]%10;
			tab[2]=0x30|(buf2[1]/10)%10;
			tab[1]=0x30|(buf2[1]/100)%10;
			tab[0]=0x30|(buf2[1]/1000)%10;
			OLED_6X8_String(5,4,PM25);
			OLED_6X8_String(45,4,tab);
			tab[4]=0;
			tab[3]=0x30|buf2[0]%10;
			tab[2]=0x30|(buf2[0]/10)%10;
			tab[1]=0x30|(buf2[0]/100)%10;
			tab[0]=0x30|(buf2[0]/1000)%10;
			OLED_6X8_String(5,6,PM1);
			OLED_6X8_String(45,6,tab);
			break;
		}
		case 1:{
			OLED_6X8_String(2,0,wdech);
			tab2[6]=0;
			tab2[5]=0x30|buf2[3]%10;
			tab2[4]=0x30|(buf2[3]/10)%10;
			tab2[3]=0x30|(buf2[3]/100)%10;
			tab2[2]=0x30|(buf2[3]/1000)%10;
			tab2[1]=0x30|(buf2[3]/10000)%10;
			tab2[0]=0x30|(buf2[3]/100000)%10;
			OLED_6X8_String(45,2,tab2);
			tab2[6]=0;
			tab2[5]=0x30|buf2[4]%10;
			tab2[4]=0x30|(buf2[4]/10)%10;
			tab2[3]=0x30|(buf2[4]/100)%10;
			tab2[2]=0x30|(buf2[4]/1000)%10;
			tab2[1]=0x30|(buf2[4]/10000)%10;
			tab2[0]=0x30|(buf2[4]/100000)%10;
			OLED_6X8_String(45,4,tab2);
			tab2[6]=0;
			tab2[5]=0x30|buf2[5]%10;
			tab2[4]=0x30|(buf2[5]/10)%10;
			tab2[3]=0x30|(buf2[5]/100)%10;
			tab2[2]=0x30|(buf2[5]/1000)%10;
			tab2[1]=0x30|(buf2[5]/10000)%10;
			tab2[0]=0x30|(buf2[5]/100000)%10;
			OLED_6X8_String(45,6,tab2);
			OLED_6X8_String(5,2,um03);
			OLED_6X8_String(5,4,um05);
			OLED_6X8_String(5,6,um10);
			break;
		}
		case 2:{
			mnoz = maxperiod / period; //obliczenie mnoznika wzgledem maksymalnego czasu i czasu pokazania próbki
			
			if((ActualTime - PastTime) > period){
				PastTime = ActualTime;
				cpm = g * mnoz;
				geiger[4]=0;
				geiger[3]=0x30|(cpm)%10;
				geiger[2]=0x30|(cpm/10)%10;
				geiger[1]=0x30|(cpm/100)%10;
				geiger[0]=0x30|(cpm/1000)%10;
				g = 0;
			}
			OLED_FILL(0x00);
			OLED_6X8_String(2,0,radiation);
			OLED_6X8_String(2,3,sivert);
			OLED_6X8_String(50,3,geiger);
			break;
		}
	}			
}

int main(void)
{
	SPI0_init();
	_delay_ms(10);
	initAll();
	OLED_Init();
	LCD_shuzi(128,0,' ');
	LCD_shuzi(128,1,' ');
	LCD_shuzi(128,2,' ');
	LCD_shuzi(128,3,' ');
	LCD_shuzi(128,4,' ');
	LCD_shuzi(128,5,' ');
	LCD_shuzi(128,6,' ');
	LCD_shuzi(128,7,' ');
	LCD_shuzi(128,8,' ');	
	OLED_FILL(0x00);
	timerISRinit();
	USART0_init();
	
	clicks=0;
	clicks2=1;
	DisplayReady=1;
	bounceFlag=0;
	bounceFlag2=0;
	bounceFlag3=0;
	changeMode=0;
	sei();
	
	
	
	while (1) {

		if(wokeUp){
			PORTB.PIN5CTRL &= ~PORT_ISC_LEVEL_gc;
			wokeUp=0;
			PB5_STATE=1;
		}
		while(PB5_STATE){
			OLED_OffOn();
			PB5_STATE = 0;	
		}
		while(PC5_STATE){
			Oled_BrLVL();
			PC5_STATE = 0;
		}
		if(refresh){
			OLED_CLEAR();
			geiger[4]=0;
			geiger[3]=0;
			geiger[2]=0;
			geiger[1]=0;
			geiger[0]=0;
		}
		if(refresh || (clicks2 && DisplayReady && ((bounceFlag | bounceFlag2 | bounceFlag3 | changeMode) == 0))){
				showData(Mode);
				DisplayReady = 0;
				refresh=0;
		}else{
		}
		if(Mode==2){
			if((ActualTime - PastTime) > period){
				DisplayReady=1;
			}
		}else{
			ActualTime=period + 1;
			PastTime=0;
		}
	}
}
ISR(PORTA_PORT_vect){
		if(Mode==2){
			g++;
			
		}
	PORTA.INTFLAGS = PORT_INT7_bm;
}
ISR(PORTB_PORT_vect){ //PA6 BUTTON - WAKE UP
	if(sleep){
		wokeUp=1;
		sleep=0;
	}
	PORTB.INTFLAGS = PORT_INT5_bm;
}

ISR(USART0_RXC_vect){ //READ USART DATA EACH TIME IT ARRIVES
	if(USART0.RXDATAL == 0x42){
		DataReady = 1;
	}
	if(DataReady){
		buf[i] = USART0.RXDATAL;
		i++;
	}
	if(i == 32){
		DataReady = 0;
		i = 0;
		if((buf2[0] != buf[6] || buf2[1] != buf[8] || buf2[2] != buf[10]) && Mode == 0){
			DisplayReady=1;
		}
		if((buf2[3] != 5*buf[18] || buf2[4] != 5*buf[20] || buf2[5] != 5*buf[22]) && Mode == 1){
			DisplayReady=1;
		}
		
		buf2[0] = buf[6];
		buf2[1] = buf[8];
		buf2[2] = buf[10];
		
		buf2[3] = 5*buf[18]; //0.3um PARTICLES COUNT in 0.1L of AIR
		buf2[4] = 5*buf[20]; //0.5um PARTICLES COUNT in 0.1L of AIR
		buf2[5] = 5*buf[22]; //1.0um PARTICLES COUNT in 0.1L of AIR
	}
}
ISR(TCA0_OVF_vect) //CHECK BUTTONS STATE EACH 5ms (PER)
{
	
	ActualTime++;
	if(!(VPORTB.IN & (PIN5_bm)))//PB5 BUTTON - POWER OFF/ON
	{
		b++;
		if(b>25){
			b=0;
			bounceFlag2=1;
		}
	}
	if((VPORTB.IN & (PIN5_bm)) && bounceFlag2==1){
		b=0;
		PB5_STATE=1;
		bounceFlag2=0;
	}	
	if(!(VPORTC.IN & (PIN5_bm)))//PC5 BUTTON - CHANGE BRIGHTNESS
	{
		a++;
		_delay_ms(10);
		if(a>5 && a<100){
			bounceFlag=1;
		}
		if(a>100){
			changeMode = 1;		
			a = 0;
			bounceFlag = 0;	
			OLED_FILL(0x00);
			OLED_6X8_String(2,2,RelToMode);
		}
	}
	if((VPORTC.IN & (PIN5_bm)) && bounceFlag==1){
		a=0;
		if(changeMode==0){
			PC5_STATE=1;
		}
		bounceFlag=0;
	}
	if((VPORTC.IN & (PIN5_bm)) && changeMode==1){
		
		Mode++;
		if(Mode==3) Mode=0;
		
		OLED_FILL(0x00);
		a=0;
		changeMode=0;
		DisplayReady = 1;
		
	}
	
	
	if(!(VPORTA.IN & (PIN6_bm)))//PA6 - RESET BUTTON - HOLD 1sec
	{
		c++;
		_delay_ms(10);
		if(c>100){
			c=0;
			OLED_FILL(0x00);
			OLED_6X8_String(5,2,release);
			bounceFlag3=1;
		}
	}
	if((VPORTA.IN & (PIN6_bm)) && bounceFlag3==0 && c<100){
		c=0;
	}
	if((VPORTA.IN & (PIN6_bm)) && bounceFlag3==1){
		c=0;
		CPU_CCP = 0xd8;
		RSTCTRL.SWRR = RSTCTRL_SWRE_bm;
		bounceFlag3=0;
	}
	
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}