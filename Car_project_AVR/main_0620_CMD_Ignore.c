   
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define CPU_CLOCK_HZ 16000000UL
#define CHAR2INT(x) x-48
#define ABS(x)	(x)>=0 ? (x) : -(x)		// step_position abs
#define STX 0x02	// start of text 
#define ETX 0x03	// end of text

#define TIMER0_0N()		TCCR0 |= (7<<CS00)	// Prescaler 1024, Timer ON
#define TIMER0_0FF()	TCCR0 &= ~(7<<CS00)	// 표준 모드, 타이머 정지
#define TIMSK_SET()		TIMSK = (1<<TOIE0)	// 타이머0 오버플로 인터럽트 허용
#define TIMSK_RESET()	TIMSK = 0x00		// 타이머 인터럽트 마스크 초기화

//<<<<<<< HEAD
//#define CAR_ID			'1'		// Car ID
unsigned char CAR_ID;
#define CAR_INIT_POS	4		// Car Init position
#define FRONT_POS	5			// Car Init position
#define REAR_POS	7			// Car Init position
//=======
//#define CAR_ID			'1'		// Car ID     1~4    'char'
//#define CAR_INIT_POS	5-(CAR_ID-'0')		// Car Init position   'int'
#define FRONT_POS	5			// Car Init position  front door
#define REAR_POS	7			// Car Init position  back door
//>>>>>>> origin/init_test

#define DRIVE		'D'	
#define STOP		'S'
#define KEEP		'K'

#define FIND_POS 		'F'
#define NOT_INIT 		'Z'
#define FINISH_INIT 	'I'
#define POS_RESET 		'#'

#define ONE_PHASE 1
#define TWO_PHASE 2
#define ONETWO_PHASE 3 
#define sw_remote_control 0x01

//****************   update step count  *****************
#define distance_H 1000
#define distance_M 600
#define distance_L 300

#define circle_count 2300

#define position_Door 1800 		//??
#define position_A	1700
#define position_B 	1400
#define position_C 	1000
#define position_D 	600
#define position_E 	200
#define position_F 	0

#define interrupt_delay 50
			
unsigned int init_position[5] = { 1700, 1400, 1000, 600, 200 };
//********************************************
volatile char *step_pulse;

volatile char STEP_TBL_1_2[] = { 0x89, 0xc1, 0x43, 0x62, 0x26, 0x34, 0x1C, 0x98 };		// 1-2 상 스텝모터 정회전 시퀀스
volatile char STEP_TBL_1[] = { 0x01, 0x02, 0x04, 0x08, 0x01, 0x02, 0x04, 0x08 };		// 1상 스텝 모터 정회전 시퀀스
volatile char STEP_TBL_2[] = { 0x09, 0x03, 0x06, 0x0C, 0x09, 0x03, 0x06, 0x0C };		// 2상 스텝 모터 정회전 시퀀스

volatile unsigned int step_position=0, position=0; 	// 현재 스텝 포지션
volatile int step_idx=0;	// 인덱스 변수


volatile char Step_flag = DRIVE;				// 스텝 모터 정,역회전 flag

int step_speed[5] = {0, 20, 15, 10, 5};			// motor speed array
volatile char current_position = 'A';	
volatile char direction_state = 'Z';
volatile char specific_position[8] = {
								'#', '1', '2', '3',
								'4', '5', '6', '7'
							};	// Specific Position array
volatile char specific_pos[8] = {
								'0', '1', '2', '3',
								'4', '5', '6', '7'
							};	// Specific Position array
volatile int interrupt_count = 0;	// pos_info count 
volatile int all_interrupt_count = 0;	// pos_info count 
/* parsing variable datas start */
unsigned char id_receive;
//unsigned char id_receive_ex;
unsigned char dir_receive;
unsigned char drive_state;
unsigned char speed_receive='1';
unsigned char position_receive='1';
unsigned char ack_nack = '1';
/* parsing variable datas end */

volatile int Step_speed = 0;	// 스텝 모터 속도
volatile unsigned long step_count = 0;

volatile long distance=0;
volatile int step_count_flag = 0;

// UART0 INT state
enum states { STX_STATE, DATA_STATE };
volatile enum states state;

volatile unsigned char rx_string[100];				// data from server, end with '\0'
volatile unsigned char save_rx_string[100];			// data from server, end with '\0'
volatile unsigned char rx_eflg = 0;					// USART0 통신 수신 인터럽트 관련 데이터/플레그
volatile int rx_usi = 0, rx_udi = 0, rx_str_len=0;	// USART0 저장 및 리딩 포인터
volatile int save_rx_str_len = 0, colon_cnt = 0;
volatile char step_check_flag = 0;

unsigned char COMMAND = 'I'; //del
enum CMD_STATES {INIT, MANUAL, AUTO, NORMAL, REQUEST ,LED_on,LED_off, Elock_on, Elock_off};
enum CMD_STATES COMMAND_STATE = NORMAL;

unsigned char Past_COMMAND = '0';
unsigned char LENGTH[10];

unsigned char cmd_I_Flag=0;
unsigned char id_sw_char;				//confirm id  by  switch		Character
unsigned int id_sw_int;					//confirm id  by  switch		int
unsigned int id_operand_arr[8] = {1,3,5,7,9,11,13,15};	// find index 

int server_parsing(unsigned char *pData);
void init_serial(unsigned long baud);
void init_debug_serial(unsigned long baud);

//int CRC(unsigned char buf[],int max_cnt);
void adc_init();
void STEP_INIT(unsigned char type);
void check_step_count();
void device_init();
void id_confirm(void);
void interrupt_init(void);

// serial tx, rx function start ===
void debug_data (unsigned char data);
void debug_string(unsigned char *data);

void tx_data (unsigned char data);
char rx_getchar_0(void);
char rx_getchar_1(void);
void serial_string(unsigned char *data);
// serial tx, rx function end ===

void send_protocol();
void reed_sw1_step_count_init();		// init step count
void reed_sw0_step_count_check(void);
void position_check(void);


ISR(USART0_RX_vect)		// USART0 수신 완료 인터럽트 루틴( Zigbee Data RX )	
{
	char status, data;
	
	status = UCSR0A;
	data = UDR0; 
	
	//debug_data(data);
 
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

void sw_step_motor(int Step_speed)	// TIMER0 OVF
{   

	PORTA = STEP_TBL_2[step_idx];
	
	// DRIVE : 정회전, REVERSE : 역회전
	if( drive_state == DRIVE ){
		step_idx++; 
		if(step_idx > 7) step_idx = 0;
		step_count++;
    }
	else if ( drive_state == STOP ) {	
		
    }
	_delay_ms( Step_speed );
}

ISR(INT0_vect)		// update step_count with reed_sw
{
	interrupt_count++;
	all_interrupt_count++;
	current_position = 	specific_position[all_interrupt_count];
	
	debug_string((unsigned char*)"\rINT0-");
	debug_data (interrupt_count+'0');
	debug_data ('\n');
	
	step_count = 0;
	position_check();
	interrupt_count = 0;  		//*********
	
	// step_count_check();	//*********************
	//step_check_flag = 1;
	//interrupt_count++;

	// 디바운싱( 채터링 방지 )
	_delay_ms(interrupt_delay);	
	while(~PIND & 0x01);
	_delay_ms(interrupt_delay);	
	EIFR = 0x01;	// EIFR = (1<<INTF0); 플래그 리셋 (다시 INT0으로 진입하는걸 피하기 위해)

}

// pos_reset Interrupt
ISR(INT1_vect)
{	
	step_count = 0;
	if(direction_state == 'I'){
		drive_state = STOP;	
		direction_state = 'A';
		send_protocol();
		debug_data ('Z');
	}
	interrupt_count = 0;			// interrupt count init
	current_position = 	specific_position[all_interrupt_count];	
	
	debug_string((unsigned char*)"\rINT1-");
	debug_data (all_interrupt_count+'0');
	debug_data ('\n');
	// direction_state = 'I';
	
	// drive_state = STOP;
	// PORTA = ~0x00;
	// PORTA = STEP_TBL_1[step_idx];
	// 디바운싱( 채터링 방지 )
	_delay_ms(interrupt_delay);	
	while(~PIND & 0x02);
	_delay_ms(interrupt_delay);	
	EIFR = 0x02;	// EIFR = (1<<INTF0); 플래그 리셋 (다시 INT0으로 진입하는걸 피하기 위해)
}

void port_init(void)
{
	DDRA = 0xff;		// PA 0~3 Stepping Motor 
	//DDRB |= 0x20;		//tractor  transform(servo moto)
	DDRC |= 0xfc;    // PC0(control mode toogle sw) PC1 (speed control sw)
	//DDRD |= ~0x01;//PD 0,1,2,3  INT0(reed sw) INT1(scan sw) debug(Rx Tx)
	//PE 0,1 Rx Tx
	DDRF |= 0x07;	//PF0,1,2 LED
	DDRG |= ~0x07;	// add dip 4sw  to define car ID	
	//**********************************
}

void id_confirm(void)		//id 0~3 truck	4~7 tractor
{	
	unsigned char id_data;
	id_data |= PING;		// PG 0,1,2
		
	switch(id_data){
		case 0x00: 	CAR_ID = '1';
						break;
		case 0x01: 	CAR_ID = '2';
						break;
		case 0x02: 	CAR_ID = '3';
						break;
		case 0x03: 	CAR_ID = '4';
						break;
		// case '0x04': 	id_sw_char = '4';
						// break;
		// case '0x05': 	id_sw_char = '5';
						// break;
		// case '0x06': 	id_sw_char = '6';
						// break;
		// case '0x07': 	id_sw_char = '7';
						// break;
		default: 	debug_string((unsigned char *)"\n id_confirm error");
						break;
	}
	
}

void interrupt_init(void)
{
	EIMSK=0x01|0x02;		//INT0 INT1
	EICRA=0x02|0x08;		//falling falling

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
	id_confirm();
	interrupt_init();
    	
	init_serial(9600);			// uart 1 init
	init_debug_serial(9600);	// uart 0 init : debug print
	//STEP_INIT(TWO_PHASE);
	
	// fdevopen( (int(*)(char, FILE *))debug_data, (int(*)(char, FILE *))rx_getchar_1);	// //printf 사용을 위한 것
	// fdevopen( debug_data, rx_getchar_1);	// //printf 사용을 위한 것

	sei();
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
	while((UCSR0A & (1<<UDRE0)) == 0x00); 
    UDR0 = data;
}

void debug_data (unsigned char data) 
{
	while((UCSR1A & (1<<UDRE1)) == 0x00); 
    UDR1 = data;
}

void serial_string(unsigned char *data)		
{
	while(*data!='\0'){		
		tx_data(*data++);
	}						
}

void debug_string(unsigned char *data)		
{
	while(*data!='\0') {		
		debug_data(*data++);
	}						
}

void position_check() {
	
	// if ( (interrupt_count == FRONT_POS) || (interrupt_count == REAR_POS) ) {
		// drive_state = STOP;		// Waiting front and read door 
	// } else if ( interrupt_count == CAR_INIT_POS ) {
		// drive_state = STOP;		// init pos
	// } else {
		// drive_state = DRIVE;
	// }
	/*debug_string((unsigned char *)"\n position_check ");
	if (direction_state == 'K'){
		//  ************ char or int ************
		//if (interrupt_count == CAR_INIT_POS){
		if (interrupt_count == (position_receive-'0')){
			drive_state = STOP;	
			direction_state = 'M';
			send_protocol();
			debug_data(direction_state);
			debug_string((unsigned char *)" STOP \n");
		}
	}//else{
	else if (direction_state == 'D'){
		//  ************ char or int ************
		//if (interrupt_count == CAR_INIT_POS){
		if (interrupt_count == (position_receive-'0')){
			drive_state = STOP;	
			//direction_state = 'M';
			//send_protocol();
			//debug_data(direction_state);
			debug_data('S');
			debug_string((unsigned char *)" STOP \n");
		}
	}
	debug_data(direction_state);*/
	//--------------------------------------------------------
	debug_string((unsigned char *)"\n pos_check ");
	debug_data(direction_state);
	debug_data(position_receive);
	if (direction_state == 'K'){    // go ahead until waitting position  >>I,1,K,3,3
		//  ************ char or int ************
		//if (interrupt_count == CAR_INIT_POS){
		if (all_interrupt_count == (position_receive-'0')){
			drive_state = STOP;	
			direction_state = 'M';
			send_protocol();
			all_interrupt_count = 0;
			debug_data(direction_state);
			debug_string((unsigned char *)" STOP \n");
		}
	}else if (direction_state == 'D'){		//      >>M,1,D,1,3
		//  ************ char or int ************
		//if (interrupt_count == CAR_INIT_POS){
		if (interrupt_count == (position_receive-'0')){
			drive_state = STOP;	
			direction_state = 'S';
			//send_protocol();
			//debug_data(direction_state);
			debug_data("S");
			debug_string((unsigned char *)" STOP \n");
		}
	}
}

int server_parsing( unsigned char *pData ) {

	//unsigned int i=0,j=0;
	//unsigned int delimiter_count = 0, delimiter_max_count=4;
	unsigned int id_operand=8;
	unsigned int id_index = id_sw_int*id_operand;
	
	// Command store	
	switch( pData[2] ) {
		case 'I': 	COMMAND_STATE = INIT;	
					break;
		case 'M': 	COMMAND_STATE = MANUAL;	
					break;
		case 'A': 	COMMAND_STATE = AUTO;	
					break;		
		case 'R': 	COMMAND_STATE = REQUEST;	
					break;	
		//??
		case 'O': 	COMMAND_STATE = LED_on;	
					break;	
		case 'X': 	COMMAND_STATE = LED_off;	
					break;		
		case 'Y': 	COMMAND_STATE = Elock_on;	
					break;	
		case 'Z': 	COMMAND_STATE = Elock_off;	
					break;	
	}
	
	// id_receive
	// direction
	// position
	// speed
	// id_receive = pData[4+id_index];			//receive id   pData[4 12 20 28...]
	// dir_receive = pData[6+id_index];		//direction
	// position_receive = pData[8+id_index];	//position
	// speed_receive = pData[10+id_index];		//speed
	
	id_receive = pData[4];			//receive id   pData[4 12 20 28...]
	dir_receive = pData[6];		//direction
	
	// Command 'R' ignore
	if( COMMAND_STATE != REQUEST ) {
		position_receive = pData[8];	//position
		speed_receive = pData[10];		//speed
	}
	
	debug_data ('\r');	
	debug_data ('\n');	
	debug_data ('>');
	debug_data (pData[2]);
	debug_data (',');	
	debug_data (id_receive);	
	debug_data (',');	
	debug_data (dir_receive);	
	debug_data (',');	
	debug_data (position_receive);	
	debug_data (',');	
	debug_data (speed_receive);	
	debug_data ('\r');	
	debug_data ('\n');
	
	return 1;
	
	
	//printf("\rbuf[0]: %c, buf[1]: %c\r", buf[0], buf[1]);
	//printf("\rserver parsing function end \r");
		
}

void send_protocol(){
/*
	volatile unsigned char tx_string[30];	// data from server, end with '\0'	
	volatile int i = 0;
	volatile char buf[10];
*/		
	
	unsigned char tx_string[30];	// data from server, end with '\0'	
	int i = 0;
	char buf[10];
	tx_string[i++] = STX;
	tx_string[i++] = ',';	
	tx_string[i++] = CAR_ID;
	tx_string[i++] = ',';	
	
	// step count  > 16hex
	sprintf(buf, "%08lX", step_count);
	
	/*
	debug_data('\r');
	debug_string((unsigned char*) buf);
	debug_data('\r');
	*/
	
	tx_string[i++] = buf[0];	//send step_count by hexcode
	tx_string[i++] = buf[1];
	tx_string[i++] = buf[2];
	tx_string[i++] = buf[3];
	tx_string[i++] = buf[4];
	tx_string[i++] = buf[5];
	tx_string[i++] = buf[6];
	tx_string[i++] = buf[7];
	
	tx_string[i++] = ',';
	//??
	//direction  state 	Z,I,J,K,A,M
	tx_string[i++] = direction_state;
	tx_string[i++] = ',';
	//position
	tx_string[i++] = current_position; 	//??
	tx_string[i++] = ',';
	//ack_nack
	tx_string[i++] = ack_nack; 
	tx_string[i++] = ',';
	
	tx_string[i++] = ETX;
	tx_string[i++] = ',';
	tx_string[i] = '\0';
	
	debug_data ('<');
	debug_string((unsigned char*) tx_string);

	serial_string((unsigned char*) tx_string);
	//debug_string((unsigned char *)" suc  ");
}

void action_func() {
		
	switch(COMMAND_STATE) {
		
		case NORMAL:	
						break;
		
		case INIT:		// direction action    Z,I,J,K,A,M,N,S,D
						switch ( dir_receive ) {
							
							/*case FIND_POS : position_check();	// CMD 'F'
											break;
							
							case DRIVE : 	position_check();	// CMD 'D'	
											break;
											
							case STOP : 	drive_state = STOP;		// CMD 'S'
											break;
												
							case FINISH_INIT : 	drive_state = STOP;	// CMD 'I'
												COMMAND_STATE = NORMAL;	
												break;*/
							case 'N':				// holding
											break;
							case 'S': 		drive_state = STOP;		// stop
											break;
							case 'D': 		direction_state = 'D';
											drive_state = DRIVE;		// arrive my position
											debug_data ('D');		// move
											break;
							case 'I': 		direction_state = 'I';	// i'm finding reed sw
											debug_data ('I');
											drive_state = DRIVE;
											break;
							case 'J': 		direction_state = 'J';		// the frount car is finding reed sw
											drive_state = DRIVE;
											break;
							case 'K': 		direction_state = 'K';		// after reed sw
											drive_state = DRIVE;
											//position_check();
											break;
							case 'M': 		
											break;
							default: 
										break;
							
						}
						sw_step_motor(step_speed[CHAR2INT(speed_receive)]);
						
						send_protocol();
						debug_data ('T');
						COMMAND_STATE = NORMAL;	//??
						
						
						break;
						
		case MANUAL:	switch ( dir_receive ) {

							case 'N':				// holding
											break;
							case 'S': 		drive_state = STOP;		// stop
											break;
							case 'D': 		direction_state = 'D';
											drive_state = DRIVE;		// arrive my position
											
											debug_data ('D');		// move
											break;
							// case 'I': 		direction_state = 'I';	// i'm finding reed sw
											// debug_data ('I');
											// drive_state = DRIVE;
											// break;
							// case 'J': 		direction_state = 'J';		// the frount car is finding reed sw
											// drive_state = DRIVE;
											// break;
							// case 'K': 		direction_state = 'K';		// after reed sw
											// drive_state = DRIVE;
											// break;
							default: 
										break;
							
						}
						sw_step_motor(step_speed[CHAR2INT(speed_receive)]);
						interrupt_count = 0;
						send_protocol();
						debug_data ('M');
						COMMAND_STATE = NORMAL;	//??
						
						break;
						
		case AUTO:
						break;
		case LED_on:	PORTF = ~0x01;
						direction_state = 'N';	//   led
						send_protocol();
						debug_data (direction_state);
						COMMAND_STATE = NORMAL;	//??
						break;
		case LED_off:	PORTF = 0x01;
						direction_state = 'N';	//   led
						send_protocol();
						debug_data (direction_state);
						COMMAND_STATE = NORMAL;	//??
						break;
				
		case REQUEST:	
						switch(dir_receive){
							
							case 'N':				// holding
											break;
							case 'S': 		drive_state = STOP;		// stop
											break;
							case 'D': 		direction_state = 'D';
											drive_state = DRIVE;		// arrive my position
											debug_data ('D');		// move
											break;
							
						}
						sw_step_motor(step_speed[CHAR2INT(speed_receive)]);
						
						send_protocol();
						debug_data ('R');
						COMMAND_STATE = NORMAL;	//??
						
						
						break;
						
	}	

}

int main(){	
	/*
	*	STX, CAR_ID, COMMAND, DATA, x, x, ETX
	*/
	int parse_result=0;
	state = STX_STATE;
	
	device_init();
	
	step_idx=0;
	drive_state = STOP;
    Step_speed = 20;
	
	
	debug_string((unsigned char *)"avr init ");
	debug_string((unsigned char *)" ID-");
	debug_data(CAR_ID);
	debug_string((unsigned char *)" POS-");
	debug_data(CAR_INIT_POS+'0');
	debug_string((unsigned char*)" INT0-");
	debug_data (interrupt_count+'0');
	debug_string((unsigned char*)" INT1-");
	debug_data (all_interrupt_count+'0');
	debug_data ('\r');
	
	
	while(1) {
				
		// position_check();
		
		if((~PINC & 0x02) == 0x02)
		{
			Step_speed = Step_speed-2;
			if(Step_speed < 2 ) Step_speed=20;
			_delay_ms(500);
		}
		
		if((~PINC & sw_remote_control) == sw_remote_control)
		{
			drive_state = DRIVE;
			sw_step_motor(Step_speed);		//car go straight on
		}
		else
		{
			//drive_state = DRIVE;
			sw_step_motor(Step_speed);		//car go straight on	
		}

		if ( rx_eflg == 1 ) {
			// debug_string((unsigned char*)rx_string);
			
			parse_result = server_parsing((unsigned char*)rx_string);
					
			rx_eflg = 0;
		}
		if( id_receive == CAR_ID ) {
			//id_receive_ex = id_receive;
			action_func();	
		}
	}
}

