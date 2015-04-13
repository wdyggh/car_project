
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// #define CPU_CLOCK_HZ 7372800UL
#define CPU_CLOCK_HZ 16000000UL

#define ABS(x)	(x)>=0 ? (x) : -(x)		// step_position ���밪
#define STX 0x02	// start of text
#define ETX 0x03	// end of text
#define ACK '8'
#define NACK '9'

#define TIMER0_0N()		TCCR0 |= (7<<CS00)	// ���ֺ� 1024, Ÿ�̸� ON
#define TIMER0_0FF()	TCCR0 &= ~(7<<CS00)	// ǥ�� ���, Ÿ�̸� ����
#define TIMSK_SET()		TIMSK = (1<<TOIE0)	// Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���
#define TIMSK_RESET()	TIMSK = 0x00		// Ÿ�̸� ���ͷ�Ʈ ����ũ �ʱ�ȭ

#define DEVICE_ID	'1' 
#define AD_CHANNEL	2

#define FORWARD	'F'	
#define STOP	'S'

#define ONE_PHASE 1
#define TWO_PHASE 2
#define ONETWO_PHASE 3 

volatile char *step_pulse;
volatile char STEP_TBL_1_2[]={0x98,0x1C,0x34,0x26,0x62,0x43,0xc1,0x89};		// 1-2 �� ���ܸ��� ��ȸ�� ������
volatile char STEP_TBL_1[]={0x08,0x04,0x02,0x01,0x08,0x04,0x02,0x01};		// 1�� ���� ���� ��ȸ�� ������
volatile char STEP_TBL_2[]={0x0C,0x06,0x03,0x09,0x0C,0x06,0x03,0x09};		// 2�� ���� ���� ��ȸ�� ������

volatile unsigned int step_position=0, position=0; 	// ���� ���� ������
volatile int idx=0;	// �ε��� ����

volatile char Step_flag = FORWARD;				// ���� ���� ��,��ȸ�� flag
volatile int Step_speed = 230;	// ���� ���� �ӵ�
volatile unsigned long step_count = 0;

volatile int step_count_flag = 0;

// UART0 INT state
enum states { STX_STATE, DATA_STATE, ACK_NAC_STATE, ETX_STATE, parsing_state};
volatile enum states state;

volatile unsigned char rx_string[100];				// data from server, end with '\0'
volatile unsigned char save_rx_string[100];			// data from server, end with '\0'
volatile unsigned char rx_eflg = 0;					// USART0 ��� ���� ���ͷ�Ʈ ���� ������/�÷���
volatile int rx_usi = 0, rx_udi = 0, rx_str_len=0;	// USART0 ���� �� ���� ������
volatile int save_rx_str_len = 0, colon_cnt = 0;

unsigned char ID;					// id 1~16, '\0'
unsigned char COMMAND = '0';
unsigned char Past_COMMAND = '0';
unsigned char LENGTH[10];
unsigned char SPEED;		// data from server, end with '\0'
unsigned char DIR;

int server_parsing(unsigned char *pData);
void init_serial(unsigned long baud);
void init_debug_serial(unsigned long baud);

int CRC(unsigned char buf[],int max_cnt);
void adc_init();
void STEP_INIT(unsigned char type);
void check_step_count();
void device_init();
void interrupt_init(void);
void adc_convert(unsigned int *pData);
void TIM0_OVF_INIT(void);

// serial tx, rx function start ===
void debug_data (unsigned char data);
void debug_string(unsigned char *data);

void tx_data (unsigned char data);
char rx_getchar_0(void);
char rx_getchar_1(void);
void serial_string(unsigned char *data);
// serial tx, rx function end ===

void send_protocol(char command, char ack_nack);



ISR(USART0_RX_vect)		// USART0 ���� �Ϸ� ���ͷ�Ʈ ��ƾ( Zigbee Data RX )	
{
	char status, data;
	
	status = UCSR0A;
	data = UDR0; 
	
	// debug_data(data);
 
	switch( state ){
		case STX_STATE : 
					
					  if(data == STX) {
						rx_usi = 0;
						rx_string[rx_usi++] = data;
						state = DATA_STATE;
						
			     	  }	
					  else {
					  	state = STX_STATE;
	                  }
					  break;
		
		case DATA_STATE :
					
					if(rx_usi >  3500){
						 rx_usi = 0;
						 state = STX_STATE;
						 
					}					
					else{	 	
				
						if(data == ETX) {
							rx_string[rx_usi++] = data;
	             			state = STX_STATE;
	                    
	        				rx_str_len = (rx_usi-1);
							save_rx_str_len	= rx_str_len;		
							rx_eflg = 1;				
							
		            	}
		            	else {
	        				rx_string[rx_usi++] = data;
                			state = DATA_STATE;
					    }
	
					}
					break;
		default: break;

	}		

}

ISR(TIMER0_OVF_vect)	// TIMER0 OVF
{   
	
	PORTA=step_pulse[idx]; 
	TCNT0 = Step_speed;	// 5 ms

		
	// FORWARD : ��ȸ��, REVERSE : ��ȸ��
	if ( Step_flag == FORWARD ) {	

		if (++idx > 7) idx=0;
		
		step_position = step_position+1;		// ��ȸ�� ���� ������ ī��Ʈ
    }
	else {	

		if (--idx < 0) idx=7;
		
		step_position = ABS((step_position-1));	// ��ȸ�� ���� ������ ī��Ʈ
    }
	
	step_count++;

	/*
	// if ( step_position > 300 ) Step_speed = MS_3_5;

	// 1-2 �� ���� ������
	if ( step_position > 400 ) {
		step_position = 1;	// ��ȸ�� 1���� ���� ���� ������ �ʱ�ȭ
	}	
	else if ( step_position == 0 ) {
		step_position = 400;	// ��ȸ�� 1���� ���� ���� �ʱ�ȭ
	}
	*/

	// check_step_count();
	
}
ISR(INT0_vect)	
{
	
	for(int j=0;j<6;j++)
	{
		switch( j ){
		case 0 : 
				if(step_count<10000) step_count=0;
				break;
		case 1 : 
				if(40000<step_count<60000) step_count=50000;
				break;
		
		case 2 :
				if(90000<step_count<110000) step_count=100000;
				break;
		case 3 :
				if(140000<step_count<160000) step_count=150000;
				break;
		case 4 :
				if(190000<step_count<210000) step_count=200000;
				break;
		case 5 :
				j=0;
				break;
		default: break;
		}
	}
	//if(j==5) j=0;
}
int main() {

	
	/*
	*	STX, DEVICE_ID, COMMAND, DATA, x, x, ETX
	*/
	// unsigned int adc_result[AD_CHANNEL];
	int parse_result=0, crc = 0;
	char buf[4], ACK_NACK;
	
	state = STX_STATE;
		
	device_init();
	// serial_string((unsigned char*)"\rtest avr data\r");
	debug_string((unsigned char*)"\ravr init ok\r");
	

	while(1) {
		/*
		serial_string("test data test test");
		_delay_ms(100);
		*/
		/*
		if( step_count_flag == 1 ) {
			step_count_flag = 0;
			printf("\rstep count is 2000 => init 0\n");
		}
		*/
		
		if ( rx_eflg == 1 ) {
			
			parse_result = server_parsing((unsigned char*)rx_string);
			
			// DEVICE_ID Check
			if( ID == DEVICE_ID ) {
		
				crc = CRC((unsigned char*)rx_string, rx_str_len-3 );
				sprintf(buf,"%02X", crc&0xff);
						 
				// CRC Value Check
				if( (buf[0] == rx_string[rx_str_len-3]) && (buf[1] == rx_string[rx_str_len-2]) ) {
					//serial_string((unsigned char * )"\rID and CRC Value Agreement\n");
									
					switch(COMMAND) {
				
						case '1': 	
									if( DIR == FORWARD ) {
										TIMER0_0N();
									} else if ( DIR == STOP ) {
										TIMER0_0FF();
									}

									if ( SPEED == 'U' ) {
										if( Step_speed < 230 ) Step_speed += 10;
										else Step_speed = 230;
									} else if( SPEED == 'D' ){
										if( Step_speed > 150 ) Step_speed -= 10;
										else Step_speed = 150;	
									} else if( SPEED == 'I' ){
										Step_speed = 150;	
									}
									break;
				
						default: 	break;
					}

					send_protocol('1', ACK);							
					// debug_string((unsigned char * )rx_string);

				}

			}
					
			rx_eflg = 0;
			
		}
		
	}
}

void port_init(void)
{
	DDRA = 0xff;		// Stepping Motor 
	DDRF = 0x00;
}

void interrupt_init(void)
{
	EIMSK=0x01;		//INT0 
	EICRA=0x02;		//falling

}
// uart 0 setting : debug
void init_serial(unsigned long baud)
{
	unsigned short ubrr;

	ubrr=(unsigned short)(CPU_CLOCK_HZ/(16*baud))-1;
	UBRR0H=(unsigned char)(ubrr>>8);
	UBRR0L=(unsigned char)(ubrr & 0xff);
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);		// RX, TX, ���ſϷ� ���ͷ�Ʈ ���
	UCSR0C = (3<<UCSZ00);	// �񵿱���, 1 ���� ��Ʈ 8 ������ ��Ʈ
}

// uart 1 setting : zigbee
void init_debug_serial(unsigned long baud)
{
	unsigned short ubrr;
	ubrr = (unsigned short) (CPU_CLOCK_HZ/(16*baud)-1);
	UBRR1H = (unsigned char)(ubrr >> 8);
	UBRR1L = (unsigned char)(ubrr & 0xff);
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);
	UCSR1C = (3<<UCSZ00);	// �񵿱���, 1 ���� ��Ʈ 8 ������ ��Ʈ
}

void adc_init()			// ADC ����	
{
	// ���������Ϸ� �� 128, ADC ����ϵ��� ����
	ADCSRA = (1<<ADEN) | (7<<ADPS0);
}

void TIM0_OVF_INIT(void)
{
	// MODE:Normal, TOP:0xFF, 1024����
	TCCR0= 0x00;
	
	TCNT0 = Step_speed;
	TIMSK_SET();
	TIMER0_0FF();
	
}

void STEP_INIT(unsigned char type){
	switch(type) {
		case ONE_PHASE: step_pulse = STEP_TBL_1;
						break;
		case TWO_PHASE:	step_pulse = STEP_TBL_2;
						break;				
		
		case ONETWO_PHASE:	step_pulse = STEP_TBL_1_2;
							break;				
		default: 	step_pulse= STEP_TBL_2;
					break;
	}
	
}

void check_step_count() {
	
	if( step_count >= 2000 ) {
		// printf("\rstep count is %d => init 0\n", step_count);
		step_count = 0;
		step_count_flag = 1;
	}
}


void device_init() {
	
	port_init();
	// adc_init();
	interrupt_init();
	TIM0_OVF_INIT();
	init_serial(9600);			// uart 1 init
	init_debug_serial(9600);	// uart 0 init : debug print
	STEP_INIT(TWO_PHASE);
	
	// fdevopen( (int(*)(char, FILE *))debug_data, (int(*)(char, FILE *))rx_getchar_1);	// //printf ����� ���� ��
	// fdevopen( debug_data, rx_getchar_1);	// //printf ����� ���� ��

	sei();
} 


int server_parsing( unsigned char *pData ) {

	int i=0;
	colon_cnt = 0;
	
	//printf("\rserver parsing function start \r");
		
	for( i=0; i<rx_str_len; i++) {

		if( pData[i] == ':' ) {
			colon_cnt++;
		}
		else {
			
			if( colon_cnt == 1 ) {
				ID = rx_string[i];
			}	
			else if( colon_cnt == 2 ) {		// COMMAND
				COMMAND = rx_string[i];
			}
			else if( colon_cnt == 3 ) {		// SPPED 
				SPEED = rx_string[i];
			}
			else if( colon_cnt == 4 ) {		// Direction
				DIR = rx_string[i];
			}
		}
	}
	
	return 1;

	//printf("\rbuf[0]: %c, buf[1]: %c\r", buf[0], buf[1]);
	//printf("\rserver parsing function end \r");
		
}

// ���� �����Ͽ� ��ȯ
char rx_getchar_0(void)
{	
	char ch;

	// ���� ���ۿ� ���ڰ� �� ������ ��ٸ�
	// RXC �÷��� ��Ʈ�� ���� ���� �����Ͱ� ���� �� ��Ʈ, ������� ��������.
	while(!(UCSR0A & (1<<RXC)));

	ch = UDR0;

	return ch;
}

// ���� �����Ͽ� ��ȯ
char rx_getchar_1(void)
{	
	char ch;

	// ���� ���ۿ� ���ڰ� �� ������ ��ٸ�
	// RXC �÷��� ��Ʈ�� ���� ���� �����Ͱ� ���� �� ��Ʈ, ������� ��������.
	while(!(UCSR1A & (1<<RXC)));

	ch = UDR1;

	return ch;
}

void tx_data (unsigned char data) 
{
	while((UCSR0A & (1<<UDRE0)) == 0x00); 
    UDR0 = data;
}

void debug_data (unsigned char data) 
{
	while((UCSR1A & (1<<UDRE1)) == 0x00); 
    UDR1 = data;
}

void adc_convert(unsigned int *pData) {
	
	int i;

	for(i=0; i<AD_CHANNEL; i++) {
		// Mux ����
		// ����� �������Ϳ� ������ ����
		// �̱ۿ��� �Է����� i-��° ADC ä�� ����
		// ���� ���� ���� 2.56v
		ADMUX = (3<<REFS0) | (i<<MUX0);
		_delay_us(100);	
				 
		ADCSRA |= (1<<ADSC);	// AD ��ȯ ����
	
		while(! (ADCSRA & (1<<ADIF)));	// ��ȯ ���Ḧ ��ٸ�
		// ��ȯ ����� �Ϸ�Ǹ� ADIF ��Ʈ�� ��Ʈ�ȴ�.
		// �ش� ���ͷ�Ʈ ���񽺷�ƾ�� ����� �� �ڵ������� ����
		// 1�� �ᵵ ADIF ��Ʈ�� ���µȴ�.
			// iä���� ��ȯ ����� �д´�.
		pData[i] = ADC;
	
		ADCSRA |= (1<<ADIF);	// ADIF �÷��� ����
	
	
	}
	pData[1] = pData[1]>>2;	// �µ�
}

void serial_string(unsigned char *data)		
{
	while(*data!='\0') {		
		tx_data(*data++);
	}						
}

void debug_string(unsigned char *data)		
{
	while(*data!='\0') {		
		debug_data(*data++);
	}						
}

// =========================== CRC ���� ==========================
// [�μ�] char buf[],int max_cnt
// Item1: ���� �ڷ� ����
// Item2: �ڷ� ����
// [����] ������
// ---------------------------------------------------------------
int CRC(unsigned char buf[], int max_cnt)
{
	int i,accum = 0;
	
	// //printf("max_cnt: %d\r\n", max_cnt);
	
	for(i=0; i<max_cnt; i++)
	{
		accum^= buf[i];		// XOR ����
		// //printf("CRC buf[%d]: %c\r\n", i, buf[i]);
	}
	
	// //printf("accum: %x\r\n", accum&0xff);

	return (accum & 0xff);
}


void send_protocol(char command, char ack_nack){

	unsigned char tx_string[30];	// data from server, end with '\0'	
	int i = 0, crc = 0;
	char buf[10];
	// serial_string("test avr tx data");
	
	tx_string[i++] = STX;
	tx_string[i++] = ':';	
	tx_string[i++] = DEVICE_ID;
	tx_string[i++] = ':';	
	tx_string[i++] = command;
	tx_string[i++] = ':';	
	
	// step count
	sprintf(buf, "%08lX", step_count);
	
	debug_data('\r');
	debug_string((unsigned char*) buf);
	debug_data('\r');

	tx_string[i++] = buf[0];
	tx_string[i++] = buf[1];
	tx_string[i++] = buf[2];
	tx_string[i++] = buf[3];
	tx_string[i++] = buf[4];
	tx_string[i++] = buf[5];
	tx_string[i++] = buf[6];
	tx_string[i++] = buf[7];
	
	tx_string[i++] = ':';
	tx_string[i++] = ack_nack; 
	tx_string[i++] = ':';
	
	// crc value
	crc = CRC((unsigned char*)tx_string, i );
	sprintf(buf,"%02X", crc&0xff);
	
	tx_string[i++] = buf[0];
	tx_string[i++] = buf[1];
	tx_string[i++] = ':';
	tx_string[i++] = ETX;
	tx_string[i++] = ':';
	tx_string[i] = '\0';

	// debug_string((unsigned char*) tx_string);
	serial_string((unsigned char*) tx_string);
	
}
