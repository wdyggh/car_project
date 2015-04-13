
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

#define ABS(x)	(x)>=0 ? (x) : -(x)		// step_position ABS value
#define STX 0x02	// start of text
#define ETX 0x03	// end of text
#define ACK '8'
#define NACK '9'

#define TIMER0_0N()		TCCR0 |= (7<<CS00)	// ���ֺ� 1024, Ÿ�̸� ON
#define TIMER0_0FF()	TCCR0 &= ~(7<<CS00)	// ǥ�� ����, Ÿ�̸� ����
#define TIMSK_SET()		TIMSK = (1<<TOIE0)	// Ÿ�̸�0 �����÷� ���ͷ�Ʈ ����
#define TIMSK_RESET()	TIMSK = 0x00		// Ÿ�̸� ���ͷ�Ʈ ����ũ �ʱ�ȭ

#define DEVICE_ID	'1' 
//#define DEVICE_ID	'2' 
unsigned char forward_DEVICE_ID;
#define AD_CHANNEL	2

#define FORWARD	'F'	
#define STOP	'S'

#define ONE_PHASE 1
#define TWO_PHASE 2
#define ONETWO_PHASE 3 
#define sw_remote_control 0x01
//****************   speed control *****************
#define Step_speed_H 6
#define Step_speed_M 10
#define Step_speed_L 14
#define distance_H 1000
#define distance_M 600
#define distance_L 300
//****************   step count check **************
#define check_point_1 0  //0
#define check_point_2 500
#define check_point_3 1000
#define check_point_4 1500
#define check_point_5 2000
#define end_step_cnt 2400
#define check_step_deviation 200

//*************************************************
volatile char *step_pulse;
/*
volatile char STEP_TBL_1_2[]={0x98,0x1C,0x34,0x26,0x62,0x43,0xc1,0x89};		// 1-2 �� ���ܸ��� ��ȸ�� ������
volatile char STEP_TBL_1[]={0x08,0x04,0x02,0x01,0x08,0x04,0x02,0x01};		// 1�� ���� ���� ��ȸ�� ������
volatile char STEP_TBL_2[]={0x0C,0x06,0x03,0x09,0x0C,0x06,0x03,0x09};		// 2�� ���� ���� ��ȸ�� ������
*/

volatile char STEP_TBL_1_2[] = { 0x89, 0xc1, 0x43, 0x62, 0x26, 0x34, 0x1C, 0x98 };		// 1-2 �� ���ܸ��� ��ȸ�� ������
volatile char STEP_TBL_1[] = { 0x01, 0x02, 0x04, 0x08, 0x01, 0x02, 0x04, 0x08 };		// 1�� ���� ���� ��ȸ�� ������
volatile char STEP_TBL_2[] = { 0x09, 0x03, 0x06, 0x0C, 0x09, 0x03, 0x06, 0x0C };		// 2�� ���� ���� ��ȸ�� ������

volatile unsigned int step_position=0, position=0; 	// ���� ���� ������
volatile int idx=0;	// �ε��� ����

volatile char Step_flag = FORWARD;				// ���� ���� ��,��ȸ�� flag
volatile int Step_speed = 0;	// ���� ���� �ӵ�
volatile unsigned long step_count = 0;
volatile long distance=0;
volatile int step_count_flag = 0;

// UART0 INT state
enum states { STX_STATE, DATA_STATE, ACK_NAC_STATE, ETX_STATE, parsing_state};
volatile enum states state;

volatile unsigned char rx_string[200];				// data from server, end with '\0'
volatile unsigned char save_rx_string[200];			// data from server, end with '\0'
volatile unsigned char rx_eflg = 0;					// USART0 ���� ���� ���ͷ�Ʈ ���� ������/�÷���
volatile int rx_usi = 0, rx_udi = 0, rx_str_len=0;	// USART0 ���� �� ���� ������
volatile int save_rx_str_len = 0, colon_cnt = 0;
volatile char step_check_flag = 0;
volatile char interrupt_count = 0;

unsigned char ID;					// id 1~16, '\0'
unsigned char COMMAND = '1';
unsigned char Past_COMMAND = '0';
unsigned char LENGTH[10];
unsigned char SPEED;		// data from server, end with '\0'
unsigned char DIR;
unsigned check_erea ='0';

// Id Infomation Structure : id, step count
typedef struct _ID_INFO {
	unsigned char id;
	unsigned long step_cnt;
}CarInfo;

CarInfo car_info[4];

//unsigned long step_cnt_info[4];

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

// serial tx, rx function start ===
void debug_data (unsigned char data);
void debug_string(unsigned char *data);

void tx_data (unsigned char data);
char rx_getchar_0(void);
char rx_getchar_1(void);
void serial_string(unsigned char *data);
// serial tx, rx function end ===

void send_protocol(char command, char ack_nack);
void step_count_check(void);


ISR(USART0_RX_vect)		// USART0 ���� �Ϸ� ���ͷ�Ʈ ��ƾ( Zigbee Data RX )	
{
	char status, data;
	
	status = UCSR0A;
	data = UDR0; 
	
	debug_data(data);
 
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
					
					if(rx_usi >  200){
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

void sw_step_motor(int Step_speed )	// TIMER0 OVF
{   

	PORTA = STEP_TBL_2[idx];
	
	// FORWARD : ��ȸ��, REVERSE : ��ȸ��
	if( DIR == FORWARD ){
		idx++; 
		if(idx > 7) idx = 0;
		step_count++;
    }
	else if ( DIR == STOP ) {	
		
    }
    
    if(( step_count >= 210000) &&( step_count >= 210000)) {
		check_erea = '1';
    }else{
    	check_erea = '0';
    }	
	
	_delay_ms( Step_speed );
}

ISR(INT0_vect)	
{
	
	//send_protocol('1', ACK);
	 step_count_check();
	//step_check_flag = 1;
	//interrupt_count++;
	/*
	// ���ٿ���( ä�͸� ���� )
	_delay_ms(10);	
	while(~PIND & 0x01);
	_delay_ms(10);	
	*/
	//_delay_ms(20);	
	//EIFR = 0x01;	// EIFR = (1<<INTF0); �÷��� ���� (�ٽ� INT0���� �����ϴ°� ���ϱ� ����)

}
ISR(INT1_vect)	
{	
	DIR = STOP;
	//PORTA = ~0x00;
	//PORTA = STEP_TBL_1[idx];
}
void port_init(void)
{
	DDRA = 0xff;		// Stepping Motor 
	DDRF = 0x00;
	//DDRD |= ~0x01;
	DDRC |= 0xfc;    // PC0 PC1 test_sw
}

void interrupt_init(void)
{
	EIMSK=0x01|0x02;		//INT0 INT1
	EICRA=0x02|0x00;		//falling low

}
// uart 0 setting : debug
void init_serial(unsigned long baud)
{
	unsigned short ubrr;

	ubrr=(unsigned short)(CPU_CLOCK_HZ/(16*baud))-1;
	UBRR0H=(unsigned char)(ubrr>>8);
	UBRR0L=(unsigned char)(ubrr & 0xff);
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);		// RX, TX, ���ſϷ� ���ͷ�Ʈ ����
	UCSR0C = (3<<UCSZ00);	// �񵿱�����, 1 ���� ��Ʈ 8 ������ ��Ʈ
}

// uart 1 setting : zigbee
void init_debug_serial(unsigned long baud)
{
	unsigned short ubrr;
	ubrr = (unsigned short) (CPU_CLOCK_HZ/(16*baud)-1);
	UBRR1H = (unsigned char)(ubrr >> 8);
	UBRR1L = (unsigned char)(ubrr & 0xff);
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);
	UCSR1C = (3<<UCSZ00);	// �񵿱�����, 1 ���� ��Ʈ 8 ������ ��Ʈ
}

void adc_init()			// ADC ����	
{
	// ���������Ϸ� ���� 128, ADC �����ϵ��� ����
	ADCSRA = (1<<ADEN) | (7<<ADPS0);
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
    	
	init_serial(9600);			// uart 1 init
	init_debug_serial(9600);	// uart 0 init : debug print
	//STEP_INIT(TWO_PHASE);
	
	// fdevopen( (int(*)(char, FILE *))debug_data, (int(*)(char, FILE *))rx_getchar_1);	// //printf ������ ���� ��
	// fdevopen( debug_data, rx_getchar_1);	// //printf ������ ���� ��

	sei();
} 

/*
int server_parsing_old( unsigned char *pData ) {

	int i=0,j=0;
	colon_cnt = 0;
	unsigned char step_cnt_str[4][10];
		
	COMMAND = pData[2];
	
	switch( COMMAND ) {
	
		case '1': 	ID = pData[4];
					DIR = pData[6];
					break;
					
		case '2': 	for( i=3; i<rx_str_len; i++) {
						
						if( pData[i] == ':' ) {
							colon_cnt++;
							j = 0;
						} else {

							if( colon_cnt == 1 ) {
								// ID1 Step Count
								step_cnt_str[0][j++] = pData[i];	
							}	
							else if( colon_cnt == 2 ) {
								// ID2 Step Count
								step_cnt_str[1][j++] = pData[i];
							}
							else if( colon_cnt == 3 ) {
								// ID3 Step Count
								step_cnt_str[2][j++] = pData[i];
							}
							else if( colon_cnt == 4 ) {
								// ID4 Step Count
								step_cnt_str[3][j++] = pData[i];
							}
						}
					}
					
					step_cnt_info[0] = atoi((char*)(&(step_cnt_str[0])));//car1 step_count
					step_cnt_info[1] = atoi((char*)(&(step_cnt_str[1])));//car2 step_count
					step_cnt_info[2] = atoi((char*)(&(step_cnt_str[2])));//car3 step_count
					step_cnt_info[3] = atoi((char*)(&(step_cnt_str[3])));//car4 step_count
					
					break;
		
		default: 	break;
	}
	
	
	
	return 1;

	//printf("\rbuf[0]: %c, buf[1]: %c\r", buf[0], buf[1]);
	//printf("\rserver parsing function end \r");
		
}
*/

int server_parsing( unsigned char *pData ) {

	int i=0,j=0;
	colon_cnt = 0;
	unsigned char step_cnt_str[4][10];
		
	COMMAND = pData[2];
	
	switch( COMMAND ) {
	
		case '1': 	ID = pData[4];
					DIR = pData[6];
					break;
					
		case '2': 	// i=3 : index calc
					for( i=3; i<rx_str_len; i++) {
						
						if( pData[i] == ':' ) {
							colon_cnt++;
							j = 0;
						} else {

							if( colon_cnt == 1 ) {
								// ID1
								car_info[0].id = pData[i];
							}	
							else if( colon_cnt == 2 ) {
								// ID1 Step Count
								step_cnt_str[0][j++] = pData[i];	
							}
							else if( colon_cnt == 3 ) {
								// ID2
								car_info[1].id = pData[i];
							}
							else if( colon_cnt == 4 ) {
								// ID2 Step Count
								step_cnt_str[1][j++] = pData[i];
																
							} else if( colon_cnt == 5 ) {
								// ID3
								car_info[2].id = pData[i];
							}
							else if( colon_cnt == 6 ) {
								// ID3 Step Count
								step_cnt_str[2][j++] = pData[i];
																
							}else if( colon_cnt == 7 ) {
								// ID4
								car_info[3].id = pData[i];
							}
							else if( colon_cnt == 8 ) {
								// ID4 Step Count
								step_cnt_str[3][j++] = pData[i];
							}
						}
					}
					
					// step count ascii to intger
					car_info[0].step_cnt = (unsigned long)atoi((char*)(&(step_cnt_str[0])));
					car_info[1].step_cnt = (unsigned long)atoi((char*)(&(step_cnt_str[1])));
					car_info[2].step_cnt = (unsigned long)atoi((char*)(&(step_cnt_str[2])));
					car_info[3].step_cnt = (unsigned long)atoi((char*)(&(step_cnt_str[3])));
									
					break;
		
		default: 	break;
	}
	
	
	
	return 1;

	//printf("\rbuf[0]: %c, buf[1]: %c\r", buf[0], buf[1]);
	//printf("\rserver parsing function end \r");
		
}

// ���� �����Ͽ� ��ȯ
char rx_getchar_0(void)
{	
	char ch;

	// ���� ���ۿ� ���ڰ� �� ������ ���ٸ�
	// RXC �÷��� ��Ʈ�� ���� ���� �����Ͱ� ���� �� ��Ʈ, �������� ��������.
	while(!(UCSR0A & (1<<RXC)));

	ch = UDR0;

	return ch;
}

// ���� �����Ͽ� ��ȯ
char rx_getchar_1(void)
{	
	char ch;

	// ���� ���ۿ� ���ڰ� �� ������ ���ٸ�
	// RXC �÷��� ��Ʈ�� ���� ���� �����Ͱ� ���� �� ��Ʈ, �������� ��������.
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
		// ������ �������Ϳ� ������ ����
		// �̱ۿ��� �Է����� i-��° ADC ä�� ����
		// ���� ���� ���� 2.56v
		ADMUX = (3<<REFS0) | (i<<MUX0);
		_delay_us(100);	
				 
		ADCSRA |= (1<<ADSC);	// AD ��ȯ ����
	
		while(! (ADCSRA & (1<<ADIF)));	// ��ȯ ���Ḧ ���ٸ�
		// ��ȯ ������ �Ϸ��Ǹ� ADIF ��Ʈ�� ��Ʈ�ȴ�.
		// �ش� ���ͷ�Ʈ ���񽺷�ƾ�� ������ �� �ڵ������� ����
		// 1�� �ᵵ ADIF ��Ʈ�� ���µȴ�.
			// iä���� ��ȯ ������ �д´�.
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
// [����] ��������
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

	volatile unsigned char tx_string[30];	// data from server, end with '\0'	
	volatile int i = 0, crc = 0;
	volatile char buf[10];
	// serial_string("test avr tx data");
	
	tx_string[i++] = STX;
	tx_string[i++] = ':';	
	tx_string[i++] = command;
	tx_string[i++] = ':';	
	tx_string[i++] = DEVICE_ID;
	tx_string[i++] = ':';	
	
	// step count
	sprintf(buf, "%08lX", step_count);
	
	debug_data('\r');
	debug_string((unsigned char*) buf);
	debug_data('\r');
	
	if( ack_nack == ACK ) {
		if( COMMAND == '1' ) {
			
			tx_string[i++] = buf[0];		//send step_count by hexcode
			tx_string[i++] = buf[1];
			tx_string[i++] = buf[2];
			tx_string[i++] = buf[3];
			tx_string[i++] = buf[4];
			tx_string[i++] = buf[5];
			tx_string[i++] = buf[6];
			tx_string[i++] = buf[7];
			
			tx_string[i++] = ':';
			
		} else if( COMMAND == '2' ) {
			
		}
		
	}
	
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

void step_count_check() {
	if(step_count<check_point_1 + check_step_deviation) {
		step_count=check_point_1;
		debug_string((unsigned char * )"\rstep_count init 0\n");
	}

	else if( ((check_point_2 - check_step_deviation) < step_count) && (step_count < (check_point_2 + check_step_deviation))) {
		step_count=check_point_2;
		debug_string((unsigned char * )"\rstep_count init 500\n");
	}
	

	else if(( (check_point_3 - check_step_deviation)<step_count) && (step_count < (check_point_3 + check_step_deviation)) ) {
		step_count=check_point_3;
		debug_string((unsigned char * )"\rstep_count init 1000\n");
	}
	
	else if( ((check_point_4 - check_step_deviation) < step_count)  && (step_count<(check_point_4 + check_step_deviation)) ) {
		step_count=check_point_4;
		debug_string((unsigned char * )"\rstep_count init 1500\n");
	}
	
	else if( ((check_point_5 - check_step_deviation)<step_count) && (step_count <  (check_point_5 - check_step_deviation)) ) {
		step_count=check_point_5;
		debug_string((unsigned char * )"\rstep_count init 2000\n");
		
	}
	
		
}

void main() {

	
	/*
	*	STX, DEVICE_ID, COMMAND, DATA, x, x, ETX
	*/
	// unsigned int adc_result[AD_CHANNEL];
	int parse_result=0, crc = 0;
	char buf[4];
	
	state = STX_STATE;
		
	device_init();
	// serial_string((unsigned char*)"\rtest avr data\r");
	debug_string((unsigned char*)"\ravr init ok\r");

	idx=0;
	DIR = STOP;
    Step_speed = 20;

	while(1) {
		
		/*if( step_check_flag == 1 ) {
			step_count_check();
			step_check_flag = 0;
		}
		*/
		
		if((~PINC & 0x02) == 0x02)
		{
			Step_speed = Step_speed-2;
			if(Step_speed < 2 ) Step_speed=20;
			_delay_ms(500);
		}
		
		
		if((~PINC & sw_remote_control) == sw_remote_control)
		{
			DIR = FORWARD;
			sw_step_motor(Step_speed);
		}
		else
		{
			//DIR = FORWARD;
			sw_step_motor(Step_speed);
			
		}
		//sw_step_motor(Step_speed);
		

		// debug_string((unsigned char * )"\rwhile syntax\n");
		
		
		if ( rx_eflg == 1 ) {
			// debug_string((unsigned char * )"\rx eflg == 1\n");
			parse_result = server_parsing((unsigned char*)rx_string);
			// debug_string((unsigned char * )"\rserver_parsing ok\n");

			// DEVICE_ID Check
			if( ID == DEVICE_ID ) {
				
				// debug_string((unsigned char * )"\rID Value Agreement\n");
				crc = CRC((unsigned char*)rx_string, rx_str_len-3 );
				sprintf(buf,"%02X", crc&0xff);
						 
				// CRC Value Check
				if( (buf[0] == rx_string[rx_str_len-3]) && (buf[1] == rx_string[rx_str_len-2]) ) {
					// debug_string((unsigned char * )"\rID and CRC Value Agreement\n");
									
					switch(COMMAND) {
				
						case '1': 	
									if( DIR == 'F' ) {
										DIR = FORWARD;
									} else if ( DIR == 'S' ) {
										DIR = STOP;
									}
									
									send_protocol('1', ACK);
									break;
						case '2': 	
								
								if (check_erea =='1'){
									
									Step_speed = Step_speed_L;	
								}
								else{
									
									if( DEVICE_ID == '1') {
										forward_DEVICE_ID ='4';											
									}else{
										forward_DEVICE_ID =DEVICE_ID- '1';	
									}
									
									distance = car_info[forward_DEVICE_ID-'0'].step_cnt - car_info[DEVICE_ID-'0'].step_cnt;
							        if(distance < 0) {
							        	distance = (end_step_cnt - (car_info[DEVICE_ID-'0'].step_cnt) + car_info[forward_DEVICE_ID-'0'].step_cnt);
							        }
							        
							        if((distance_M < distance) && (distance < distance_H)){
										Step_speed = Step_speed_H;
									}else if((distance_L < distance) && (distance < distance_M)){
										Step_speed = Step_speed_M;
									}else if(distance < distance_L){
										Step_speed = Step_speed_L;
									}	
							        
							
								}
									// DEVICE_ID-49, DEVICE_ID-48 : DEVICE_ID is Char
							/*		distance = car_info[DEVICE_ID-'1'].step_cnt - car_info[DEVICE_ID-'0'].step_cnt;
								if((distance < 0) || (DEVICE_ID == '1')){	// (front car >< back car)||( NO.1car)
									Step_speed = Step_speed_M;
								}else{
									//distance = step_cnt_info[DEVICE_ID-1] - step_cnt_info[DEVICE_ID];
									if((distance_M < distance) && (distance < distance_H)){
									Step_speed = Step_speed_H;
									}else if((distance_L < distance) && (distance < distance_M)){
									Step_speed = Step_speed_M;
									}else if((0 < distance) && (distance < distance_L)){
									Step_speed = Step_speed_L;
									}	
								}*/
								/*
								distance = step_cnt_info[DEVICE_ID-1] - step_cnt_info[DEVICE_ID];
								if((distance < 0) || (DEVICE_ID == '1')){	// (front car >< back car)||( NO.1car)
									Step_speed = Step_speed_M;
								}else{
									//distance = step_cnt_info[DEVICE_ID-1] - step_cnt_info[DEVICE_ID];
									if((distance_M < distance) && (distance < distance_H)){
									Step_speed = Step_speed_H;
									}else if((distance_L < distance) && (distance < distance_M)){
									Step_speed = Step_speed_M;
									}else if((0 < distance) && (distance < distance_L)){
									Step_speed = Step_speed_L;
									}	
								}
								*/
									break;
						default: 	break;
					}
				/*	
				//===============================
				location[4]={car1,car2,car3,car4};
				#define car_ID 1;
				#define Step_speed_H 5;
				#define Step_speed_M 14;
				#define Step_speed_L 20;
				#define distance_H 1000;
				#define distance_M 600;
				#define distance_L 300;
				
				//distance = location[4]-location[1];
				distance = location[car_ID-1]-location[car_ID];
				if((distance_M<distance)&&(distance<distance_H))	Step_speed=Step_speed_H;
				else if((distance_L<distance)&&(distance<distance_M))	Step_speed=Step_speed_M;
				else if((0<distance)&&(distance<distance_L))	Step_speed=Step_speed_L;
								
				//============================================
				*/	
					// debug_string((unsigned char * )rx_string);

				} 
				/*
				else {
					if( COMMAND == '1' ) {
						send_protocol('1', NACK);
						debug_string((unsigned char * )"\rCommand 1 NACK\n");	
					} else if( COMMAND == '2' ) {
						send_protocol('2', NACK);
						debug_string((unsigned char * )"\rCommand 2 NACK\n");
					} else {
						
					}
				}
				*/
			}
					
			rx_eflg = 0;
			
		}
		
	}
}

