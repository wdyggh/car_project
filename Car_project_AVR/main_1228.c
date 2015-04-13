
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// #define CPU_CLOCK_HZ 7372800UL
#define CPU_CLOCK_HZ 16000000UL

#define ABS(x)	(x)>=0 ? (x) : -(x)		// step_position 절대값
#define STX 0x02	// start of text
#define ETX 0x03	// end of text

#define TIMER0_0N()		TCCR0 |= (7<<CS00)	// 분주비 1024, 타이머 ON
#define TIMER0_0FF()	TCCR0 &= ~(7<<CS00)	// 표준 모드, 타이머 정지
#define TIMSK_SET()		TIMSK = (1<<TOIE0)	// 타이머0 오버플로 인터럽트 허용
#define TIMSK_RESET()	TIMSK = 0x00		// 타이머 인터럽트 마스크 초기화

#define DEVICE_ID	'1' 
#define AD_CHANNEL	2

#define FOWARD	1	
#define REVERSE	2

volatile char STEP_TBL_1_2[]={0x98,0x1C,0x34,0x26,0x62,0x43,0xc1,0x89};		// 1-2 상 스텝모터 정회전 시퀀스

// volatile char STEP_TBL_2[]={0x0C,0x06,0x03,0x09,0x0C,0x06,0x03,0x09};		// 2상 스텝 모터 정회전 시퀀스

volatile unsigned int step_position=0, position=0; 	// 현재 스텝 포지션
volatile int idx=0;	// 인덱스 변수

int Step_flag = 0;				// 스텝 모터 정,역회전 flag
volatile int Step_speed = 130;	// 스텝 모터 속도

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
volatile unsigned char rx_eflg = 0;					// USART0 통신 수신 인터럽트 관련 데이터/플레그
volatile int rx_usi = 0, rx_udi = 0, rx_str_len=0;	// USART0 저장 및 리딩 포인터
volatile int save_rx_str_len = 0, colon_cnt = 0;

unsigned char ID;					// id 1~16, '\0'
unsigned char COMMAND = '0';
unsigned char Past_COMMAND = '0';
unsigned char LENGTH[10];
unsigned char SPEED;		// data from server, end with '\0'
unsigned char DIR;

ISR(USART0_RX_vect)		// USART0 수신 완료 인터럽트 루틴( Zigbee Data RX )	
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
	PORTA=STEP_TBL_1_2[idx]; // 1-2 상
	//PORTA=STEP_TBL_2[idx]; // 2 상
	
	TCNT0 = Step_speed;	// 5 ms
		
	// FOWARD : 정회전, REVERSE : 역회전
	if ( Step_flag == FOWARD ) {	

		if (++idx > 7) idx=0;
		
		step_position = step_position+1;		// 정회전 스텝 포지션 카운트
    }
	else {	

		if (--idx < 0) idx=7;
		
		step_position = ABS((step_position-1));	// 역회전 스텝 포지션 카운트
    }

	// if ( step_position > 300 ) Step_speed = MS_3_5;

	// 1-2 상 스텝 포지션
	if ( step_position > 400 ) {
		step_position = 1;	// 정회전 1바퀴 돌면 스텝 포지션 초기화
	}	
	else if ( step_position == 0 ) {
		step_position = 400;	// 역회전 1바퀴 돌면 스텝 초기화
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
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);		// RX, TX, 수신완료 인터럽트 허용
	UCSR0C = (3<<UCSZ00);	// 비동기모드, 1 정지 비트 8 데이터 비트
}

// uart 1 setting : zigbee
void init_debug_serial(unsigned long baud)
{
	unsigned short ubrr;
	ubrr = (unsigned short) (CPU_CLOCK_HZ/(16*baud)-1);
	UBRR1H = (unsigned char)(ubrr >> 8);
	UBRR1L = (unsigned char)(ubrr & 0xff);
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);
	UCSR1C = (3<<UCSZ00);	// 비동기모드, 1 정지 비트 8 데이터 비트
}

void adc_init()			// ADC 설정	
{
	// 프리스케일러 비를 128, ADC 사용하도록 설정
	ADCSRA = (1<<ADEN) | (7<<ADPS0);
}

void TIM0_OVF_INIT(void)
{
	// MODE:Normal, TOP:0xFF, 1024분주
	TCCR0= 0x00;
	
	TCNT0 = Step_speed;
	
	TIMSK_SET();
}

void device_init() {
	
	port_init();
	//adc_init();

	init_serial(9600);			// uart 1 init
	init_debug_serial(9600);	// uart 0 init : debug print
	
	// fdevopen( (int(*)(char, FILE *))debug_data, (int(*)(char, FILE *))rx_getchar_1);	// //printf 사용을 위한 것
	// fdevopen( debug_data, rx_getchar_1);	// //printf 사용을 위한 것

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

// 문자 수신하여 반환
char rx_getchar_0(void)
{	
	char ch;

	// 수신 버퍼에 문자가 찰 때까지 기다림
	// RXC 플래그 비트는 읽지 않은 데이터가 있을 때 세트, 비워지면 지워진다.
	while(!(UCSR0A & (1<<RXC)));

	ch = UDR0;

	return ch;
}

// 문자 수신하여 반환
char rx_getchar_1(void)
{	
	char ch;

	// 수신 버퍼에 문자가 찰 때까지 기다림
	// RXC 플래그 비트는 읽지 않은 데이터가 있을 때 세트, 비워지면 지워진다.
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
		// Mux 설정
		// 결과를 레지스터에 오른쪽 정렬
		// 싱글엔드 입력으로 i-번째 ADC 채널 설정
		// 내부 기준 전압 2.56v
		ADMUX = (3<<REFS0) | (i<<MUX0);
		_delay_us(100);	
				
		ADCSRA |= (1<<ADSC);	// AD 변환 시작
	
		while(! (ADCSRA & (1<<ADIF)));	// 변환 종료를 기다림
		// 변환 결과가 완료되면 ADIF 비트가 세트된다.
		// 해당 인터럽트 서비스루틴이 수행될 때 자동적으로 리셋
		// 1울 써도 ADIF 비트는 리셋된다.
			// i채널의 변환 결과를 읽는다.
		pData[i] = ADC;
	
		ADCSRA |= (1<<ADIF);	// ADIF 플래그 지움
	
	
	}
	pData[1] = pData[1]>>2;	// 온도
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

// =========================== CRC 연산 ==========================
// [인수] char buf[],int max_cnt
// Item1: 연산 자료 버퍼
// Item2: 자료 갯수
// [리턴] 연산결과
// ----------------------------------------------------------------
int CRC(unsigned char buf[], int max_cnt)
{
	int i,accum = 0;
	
	// //printf("max_cnt: %d\r\n", max_cnt);
	
	for(i=0; i<max_cnt; i++)
	{
		accum^= buf[i];		// XOR 연산
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
