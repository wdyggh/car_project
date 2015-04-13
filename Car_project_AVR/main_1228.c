
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// #define CPU_CLOCK_HZ 7372800UL
#define CPU_CLOCK_HZ 16000000UL

#define ABS(x)	(x)>=0 ? (x) : -(x)		// step_position ���밪
#define STX 0x02	// start of text
#define ETX 0x03	// end of text

#define TIMER0_0N()		TCCR0 |= (7<<CS00)	// ���ֺ� 1024, Ÿ�̸� ON
#define TIMER0_0FF()	TCCR0 &= ~(7<<CS00)	// ǥ�� ���, Ÿ�̸� ����
#define TIMSK_SET()		TIMSK = (1<<TOIE0)	// Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���
#define TIMSK_RESET()	TIMSK = 0x00		// Ÿ�̸� ���ͷ�Ʈ ����ũ �ʱ�ȭ

#define DEVICE_ID	'1' 
#define AD_CHANNEL	2

#define FOWARD	1	
#define REVERSE	2

volatile char STEP_TBL_1_2[]={0x98,0x1C,0x34,0x26,0x62,0x43,0xc1,0x89};		// 1-2 �� ���ܸ��� ��ȸ�� ������

// volatile char STEP_TBL_2[]={0x0C,0x06,0x03,0x09,0x0C,0x06,0x03,0x09};		// 2�� ���� ���� ��ȸ�� ������

volatile unsigned int step_position=0, position=0; 	// ���� ���� ������
volatile int idx=0;	// �ε��� ����

int Step_flag = 0;				// ���� ���� ��,��ȸ�� flag
volatile int Step_speed = 130;	// ���� ���� �ӵ�

int server_parsing(unsigned char *pData);
void init_serial(unsigned long baud);
void init_debug_serial(unsigned long baud);
void debug_data (unsigned char data);
void debug_string(unsigned char *data);

int CRC(unsigned char buf[],int max_cnt);
void adc_init();
void device_init();
void adc_convert(unsigned int *pData);
void send_protocol(unsigned char *pData);
void TIM0_OVF_INIT(void);

// serial tx, rx function start ===
void tx_data (unsigned char data);
char rx_getchar_0(void);
char rx_getchar_1(void);
// serial tx, rx function end ===

void save_buffer(char *pBuf);
void send_buffer();


// UART1 INT state
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
	PORTA=STEP_TBL_1_2[idx]; // 1-2 ��
	//PORTA=STEP_TBL_2[idx]; // 2 ��
	
	TCNT0 = Step_speed;	// 5 ms
		
	// FOWARD : ��ȸ��, REVERSE : ��ȸ��
	if ( Step_flag == FOWARD ) {	

		if (++idx > 7) idx=0;
		
		step_position = step_position+1;		// ��ȸ�� ���� ������ ī��Ʈ
    }
	else {	

		if (--idx < 0) idx=7;
		
		step_position = ABS((step_position-1));	// ��ȸ�� ���� ������ ī��Ʈ
    }

	// if ( step_position > 300 ) Step_speed = MS_3_5;

	// 1-2 �� ���� ������
	if ( step_position > 400 ) {
		step_position = 1;	// ��ȸ�� 1���� ���� ���� ������ �ʱ�ȭ
	}	
	else if ( step_position == 0 ) {
		step_position = 400;	// ��ȸ�� 1���� ���� ���� �ʱ�ȭ
	}	
	
}

int main() {
	
	/*
	*	STX, DEVICE_ID, COMMAND, DATA, x, x, ETX
	*/
	// unsigned int adc_result[AD_CHANNEL];
	int parse_result=0;
	state = STX_STATE;

	device_init();

	debug_string((unsigned char*)"\ravr init ok\r");
	
	while(1) {
		/*
		debug_string((unsigned char*)"\rhi hello\r");
		_delay_ms(100);
		*/
		if ( rx_eflg == 1 ) {
			
			parse_result = server_parsing((unsigned char*)rx_string);
			
			/*		
			// test protocol 6 : HTML -> AVR
			// test protocol 7 : HTML <- AVR
			if(COMMAND == '6') {   

				save_buffer((char*)rx_string);
				
				PORTA= ~0x01;
				_delay_ms(30);
				PORTA= ~0x00;							
				
			}
			else if(COMMAND == '7') {   //NACK
				//serial_string("test");
				
				send_buffer();
				
				PORTA= ~0x02;
				_delay_ms(30);
				PORTA= ~0x00;
				
			}
			*/	
			rx_eflg = 0;
		}	
	}
}

void port_init(void)
{
	DDRA = 0xff;		// Stepping Motor 
	PORTA = 0xff;		// LED OFF
	DDRF = 0x00;
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
}

void device_init() {
	
	port_init();
	//adc_init();

	init_serial(9600);			// uart 1 init
	init_debug_serial(9600);	// uart 0 init : debug print
	
	// fdevopen( (int(*)(char, FILE *))debug_data, (int(*)(char, FILE *))rx_getchar_1);	// //printf ����� ���� ��
	// fdevopen( debug_data, rx_getchar_1);	// //printf ����� ���� ��

	sei();
} 


int server_parsing( unsigned char *pData ) {

	int i=0, crc = 0;
	char buf[4];
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
	
	// DEVICE_ID Check
	if( ID == DEVICE_ID ) {
		
		crc = CRC((unsigned char*)rx_string, rx_str_len-3 );
		sprintf(buf,"%02X", crc&0xff);
		// debug_string(buf);
		
		// CRC Value Check
		if( (buf[0] == rx_string[rx_str_len-3]) && (buf[1] == rx_string[rx_str_len-2]) ) {

			// action!!
			debug_string("\rID and CRC Value Agreement\r");
			
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
	while((UCSR0A & (1<<UDRE1)) == 0x00); 
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
// ----------------------------------------------------------------
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

void save_buffer(char *pBuf) {
	
	int len = save_rx_str_len;
	int i, crc=0;
	char buf[4];

	//printf("\rsave_buffer function start \r");

	for( i= 0; i<len-5; i++) {
	 	save_rx_string[i] = *(pBuf++);
	}
	
	crc = CRC((unsigned char*)rx_string, rx_str_len-5 );
	sprintf(buf,"%02X", crc&0xff);

	save_rx_string[i++] = buf[0];
	save_rx_string[i++] = buf[1];
	save_rx_string[i++] = ':';
	
	save_rx_string[i++] = ETX;
	save_rx_string[i++] = ':';

	save_rx_string[i] ='\0';

	//printf("\rbuf[0]: %c, buf[1]: %c\r", buf[0], buf[1]);
	//printf("\rrsave_buffer function end \r");

		
	//printf("\rsave_rx_string : %s \r", save_rx_string);
	//printf("\rsave_buffer function end \r");
}

void send_buffer() {
		
	//printf("\send_buffer function start \r");

	// serial_string((unsigned char*)save_rx_string);
	serial_string((unsigned char*)save_rx_string);
	_delay_ms(1);
	debug_data ('\r');
	debug_string((unsigned char*)save_rx_string);
	//debug_data ('\n'); 

	//serial_string("testavrdata string" );

	//printf("\rsave_rx_string : %s \r", save_rx_string);
	//printf("\send_buffer function end \r");
	
}
