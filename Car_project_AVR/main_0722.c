   
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define CPU_CLOCK_HZ 7372800UL
// #define CPU_CLOCK_HZ 16000000UL

#define CHAR2INT(x) x-48
#define ABS(x)	(x)>=0 ? (x) : -(x)		// step_position abs
#define STX 0x02	// start of text 
#define ETX 0x03	// end of text

#define TIMER0_0N()		TCCR0 |= (7<<CS00)	// Prescaler 1024, Timer ON
#define TIMER0_0FF()	TCCR0 &= ~(7<<CS00)	// 표준 모드, 타이머 정지
#define TIMSK_SET()		TIMSK = (1<<TOIE0)	// 타이머0 오버플로 인터럽트 허용
#define TIMSK_RESET()	TIMSK = 0x00		// 타이머 인터럽트 마스크 초기화

//<<<<<<< HEAD
unsigned char CAR_ID;
#define CAR_INIT_POS	5-(CAR_ID-'0')		// Car Init position   'int'
#define FRONT_POS	5						// Car Init position
#define REAR_POS	7						// Car Init position
//<<<<<<<

/* Parsing Command char ( RPI -> AVR ) */
#define	INIT_CHAR			'I'
#define	MANUAL_CHAR			'M'
#define	AUTO_CHAR			'A'
#define REQUEST_CHAR		'R'
#define LED_ON_CHAR			'O'
#define LED_OFF_CHAR		'X'
#define EMAGNET_ON_CHAR		'Y'
#define EMAGNET_OFF_CHAR	'Z'

/*
[ Direction state ]
* 	KEEPING 			'N'	// 유지
*	DRIVE				'D'	// 이동
* 	STOP				'S'	// 정지
* 	INIT_FIND_RESET 	'I'	// Init 동작상태(Reset pos 찾아서 이동하는 상태)
* 	INIT_JUST_MOVE		'J'	// Init 동작상태( 단순 이동 모드 )
* 	ARRIVAL_RESET_POS	'A'	// 리셋 위치에 도착 후 대기위치로 이동
* 	MOVE_WAIT_POS		'K'	// 대기 위치로 이동하는 상태
* 	ARRIVAL_WAIT_POS	'M'	// 대기 위치에 도착한 상태
*/
#define NONE				'0'
#define KEEPING 			'N'
#define DRIVE				'D'	
#define STOP				'S'
#define INIT_FIND_RESET 	'I'	
#define INIT_JUST_MOVE		'J'	
#define ARRIVAL_RESET_POS	'A'	
#define MOVE_WAIT_POS		'K'	
#define ARRIVAL_WAIT_POS	'M'	

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

volatile char STEP_TBL_1_2[] = { 0x89, 0xc1, 0x43, 0x62, 0x26, 0x34, 0x1C, 0x98 };		// 1-2상
volatile char STEP_TBL_1[] = { 0x01, 0x02, 0x04, 0x08, 0x01, 0x02, 0x04, 0x08 };		// 1상 
volatile char STEP_TBL_2[] = { 0x09, 0x03, 0x06, 0x0C, 0x09, 0x03, 0x06, 0x0C };		// 2상 

volatile int global_step_idx=0;	// 인덱스 변수

int step_speed[5] = {20, 15, 10, 5, 2};			// motor speed array
volatile char current_position = 'A';	
volatile char direction_state = 'Z';
volatile char specific_position[8] = {
								'#', '1', '2', '3',
								'4', '5', '6', '7'
							};	// Specific Position array

volatile int step_count_array[8] = 	{ 0, };	// step_count_array

volatile int interrupt_count = 0;		// Interrupt count 
volatile int all_interrupt_count = 0;	// All interrupt count 

/* parsing variable datas start */
unsigned char id_receive;
unsigned char dir_receive;
unsigned char drive_state;
unsigned char speed_receive='1';
unsigned char position_receive='1';
unsigned char ack_nack = '1';
/* parsing variable datas end */

volatile int global_step_speed = 0;		// 스텝 모터 속도
volatile unsigned long step_count = 0;

// UART0 INT state
enum states { STX_STATE, DATA_STATE };
volatile enum states state;

volatile unsigned char rx_string[100];				// data from server, end with '\0'
volatile unsigned char save_rx_string[100];			// data from server, end with '\0'
volatile unsigned char rx_eflg = 0;					// USART0 통신 수신 완료 flag
volatile int rx_usi = 0, rx_udi = 0, rx_str_len=0;	// USART0 저장 및 리딩 포인터
volatile int save_rx_str_len = 0, colon_cnt = 0;

enum CMD_STATES {
					INIT, MANUAL, AUTO, NORMAL, 
					REQUEST ,LED_ON,LED_OFF, 
					EMAGNET_ON, EMAGNET_OFF
				};
				
enum CMD_STATES COMMAND_STATE = NORMAL;	// Command State : init = normal

unsigned char id_sw_char;				//confirm id  by  switch Character
unsigned int id_sw_int;					//confirm id  by  switch int
unsigned int id_operand_arr[8] = {1,3,5,7,9,11,13,15};	// find index 

/* My car info */
typedef struct __car_info {
	char id;
	char direction_state;
}CAR_INFO;
CAR_INFO my_car_info;

void port_init(void);
void interrupt_init(void);
void device_init(void);

void sw_step_motor(int step_speed);
void pass_reed_sw(void);

int server_parsing(unsigned char *pData);
void init_serial(unsigned long baud);
void init_debug_serial(unsigned long baud);

void id_confirm(void);

// serial tx, rx function start ===
void debug_data (unsigned char data);
void debug_string(unsigned char *data);

void tx_data (unsigned char data);
char rx_getchar_0(void);
char rx_getchar_1(void);
void serial_string(unsigned char *data);
// serial tx, rx function end ===

void position_check(void);
void send_protocol();
void init_message(void);
void action_func();

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
						
			     	  }	else {
					  	state = STX_STATE;
	                  }
					  break;
		
		case DATA_STATE :
					
					if(rx_usi >  200) {
						 rx_usi = 0;
						 state = STX_STATE;
						 
					} else {	 	
				
						if(data == ETX) {
							rx_string[rx_usi++] = data;
	             			state = STX_STATE;
	                    
	        				rx_str_len = (rx_usi-1);
							save_rx_str_len	= rx_str_len;		
							rx_eflg = 1;				
							
		            	} else {
	        				rx_string[rx_usi++] = data;
                			state = DATA_STATE;
					    }
	
					}
					break;
		default: break;

	}		

}

ISR(INT0_vect)		// update step_count with reed_sw
{
	interrupt_count++;
	pass_reed_sw();
	all_interrupt_count++;
	current_position = specific_position[all_interrupt_count];
		
	debug_string((unsigned char*)"\rINT0-");
	debug_data (all_interrupt_count+'0');
	debug_data ('\n');
		
	position_check();
	step_count = 0;				// 요청 사항
	interrupt_count = 0;  		//*********	
	
	/* debouncing */
	_delay_ms(interrupt_delay);	
	while(~PIND & 0x01);
	_delay_ms(interrupt_delay);	
	
	// EIFR = (1<<INTF0); 플래그 리셋 (다시 INT0으로 진입하는걸 피하기 위해)
	EIFR = 0x01;	

}

// pos_reset Interrupt
ISR(INT1_vect)
{	
	step_count = 0;		// step count init
	pass_reed_sw();
	
	all_interrupt_count++;
	
	if(direction_state == INIT_CHAR){
		drive_state = STOP;	
		direction_state = ARRIVAL_RESET_POS;
		all_interrupt_count=0;
		send_protocol();
	}
	
	interrupt_count = 0;	// interrupt count init
	current_position = specific_position[all_interrupt_count];	// init positioin
	
	debug_string((unsigned char*)"\rINT1-");
	debug_data (all_interrupt_count+'0');
	debug_data ('\n');
		
	/* debouncing */
	_delay_ms(interrupt_delay);	
	while(~PIND & 0x02);
	_delay_ms(interrupt_delay);	
	
	// EIFR = (1<<INTF1); 플래그 리셋 (다시 INT0으로 진입하는걸 피하기 위해)
	EIFR = 0x02;
}

int main(){	
	
	int parse_result=0;
	state = STX_STATE;
	
	device_init();
	
	// Init step motor info
	drive_state = STOP;
	global_step_idx=0;		// Init step motor table index
	global_step_speed = 15;	// Init step motor speed ( delay )
		
	/* Program Init Message */
	init_message();
				
	while(1) {
				
		// position_check();
		
		if((~PINC & 0x02) == 0x02) {
			global_step_speed = global_step_speed-2;
			if( global_step_speed < 2 ) global_step_speed=20;
			send_protocol();
			_delay_ms(500);
		}
		
		/* PORTC0 Pin Level Check */
		if((~PINC & sw_remote_control) == sw_remote_control) {
			drive_state = DRIVE;
			sw_step_motor(global_step_speed);		//car go straight on
		} else {
			//drive_state = DRIVE;
			sw_step_motor(global_step_speed);		//car go straight on	
		}

		/* receive complete check */
		if ( rx_eflg == 1 ) {
			
			// debug_string((unsigned char*)rx_string);
			/* ID Check */
			if ( rx_string[4] == CAR_ID ) {
				debug_string((unsigned char*)"\rrx_string[4] == CAR_ID\r");
				parse_result = server_parsing((unsigned char*)rx_string);	
			}
			
			rx_eflg = 0;	// Reset rx_eflg
		}
				
		action_func();
		
	}
	
} // end of main function

void sw_step_motor(int step_speed) {	// TIMER0 OVF
   	
	static int step_idx_toggle = 0;
	static int led_delay_cnt = 0;
	static int led_onoff_flag = 1;
	
	// DRIVE : 정회전, REVERSE : 역회전
	if( drive_state == DRIVE ) {
		
		PORTA = STEP_TBL_2[global_step_idx];
		
		global_step_idx++; 
		if(global_step_idx > 7) global_step_idx = 0;
		step_count++;
				
		step_idx_toggle = 0;
				
		if( led_delay_cnt == 10 ) {
			PORTF |= 0x01;	// LED OFF
			led_onoff_flag = 0;
		} else if( led_delay_cnt == 0 ) {
			PORTF = ~0x01;	// LED ON
			led_onoff_flag = 1;
		}
		
		if( led_onoff_flag == 1 ) {
			led_delay_cnt++;
		} else {
			led_delay_cnt--;
		}
		
    } else if ( drive_state == STOP ) {	
		
		PORTA = 0x00;	// STOP
		
		/* LED OFF */ 
		led_onoff_flag = 0;
		led_delay_cnt = 0;
		PORTF |= 0x01;	
		 
		if ( step_idx_toggle == 0 ) {
			if (global_step_idx > 0) {
				global_step_idx -= 1;			// step init
			}
			step_idx_toggle = 1;
		}
		
    }
	_delay_ms( step_speed );
}

void pass_reed_sw() {
	
	for(int i=0;i<70;i++){
	
		PORTA = STEP_TBL_2[global_step_idx++];
		step_count++;
		
		if(global_step_idx > 7) global_step_idx = 0;
		_delay_ms( global_step_speed );
		
		
	}
	
}

void port_init(void)
{
	DDRA = 0xff;		// PA 0~3 Stepping Motor 
	//DDRB |= 0x20;		//tractor  transform(servo moto)
	DDRC |= 0xfc;    // PC0(control mode toogle sw) PC1 (speed control sw)
	//DDRD |= ~0x01;//PD 0,1,2,3  INT0(reed sw) INT1(scan sw) debug(Rx Tx)
	//PE 0,1 Rx Tx
	DDRF |= 0x07;	//PF0-LED,1-Elock,2-?? 
	DDRG |= ~0x07;	// add dip 4sw  to define car ID	
	PORTF|= ~(0x01&0x02);
	//**********************************
}

//id 0~3 truck	4~7 tractor
void id_confirm(void) {
	
	unsigned char id_data = 0x00;
	id_data = (PING & 0x07);	// PG 0,1,2
		
	switch(id_data) {
		
		case 0x00: 	CAR_ID = '1';
					break;
					
		case 0x01: 	CAR_ID = '2';
					break;
					
		case 0x02: 	CAR_ID = '3';
					break;
					
		case 0x03: 	CAR_ID = '4';
					break;
					
		/*			
		case 0x04: 	id_sw_char = '4';
					break;
		case 0x05: 	id_sw_char = '5';
					break;
		case 0x06: 	id_sw_char = '6';
					break;
		case 0x07: 	id_sw_char = '7';
					break;
		*/					
		default: 	debug_string((unsigned char *)"\n id_confirm error");
					break;
	}
	
}

void interrupt_init(void)
{
	EIMSK=0x01|0x02;		//INT0 INT1
	EICRA=0x03|0x0c;		//falling falling

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

void device_init() {
	
	port_init();		// PORT Input / Output Configuration
	id_confirm();		// ID Initialize
	interrupt_init();	// Interrupt Initialize
    	
	init_serial(57600);			// uart 1 init
	init_debug_serial(57600);	// uart 0 init : debug print
		
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
		
	// Debug message	
	debug_string((unsigned char *)"\rposition_check function");
	debug_string((unsigned char *)"\rdirection_state : ");
	debug_data(direction_state);
	debug_string((unsigned char *)"\rposition_receive : ");
	debug_data(position_receive);
	
	if (direction_state == MOVE_WAIT_POS) {    // go ahead until waitting position  >>I,1,K,3,3
		
		//  ************ char or int ************
		if (all_interrupt_count == (position_receive-'0')){
			drive_state = STOP;	
			direction_state = ARRIVAL_WAIT_POS;
			send_protocol();
			all_interrupt_count = 0;
			debug_data(direction_state);
			debug_string((unsigned char *)" STOP \n");
		}
		
	} else if (direction_state == 'D') {
	//	COMMAND is MANUAL >> M,1,D,1,3
		
		//  ************ char or int ************
		if (interrupt_count == (position_receive-'0')) {
			
			drive_state = STOP;	
			direction_state = STOP;
			send_protocol();
			if(all_interrupt_count==7) all_interrupt_count=0;
			debug_string((unsigned char *)" STOP \n");
			
		}
	}
}

int server_parsing( unsigned char *pData ) {

	//unsigned int i=0,j=0;
	//unsigned int delimiter_count = 0, delimiter_max_count=4;
	//unsigned int id_operand=8;
	//unsigned int id_index = id_sw_int*id_operand;
	
	// Command store	
	switch( pData[2] ) {
		case INIT_CHAR: 	COMMAND_STATE = INIT;
							break;
		case MANUAL_CHAR: 	COMMAND_STATE = MANUAL;
							break;
		case AUTO_CHAR: 	COMMAND_STATE = AUTO;
							break;		
		case REQUEST_CHAR: 	COMMAND_STATE = REQUEST;
							break;	
		//??
		case LED_ON_CHAR: 	COMMAND_STATE = LED_ON;
								break;	
		case LED_OFF_CHAR: 	COMMAND_STATE = LED_OFF;
							break;		
		case EMAGNET_ON_CHAR: 	COMMAND_STATE = EMAGNET_ON;	
								break;	
		case EMAGNET_OFF_CHAR: 	COMMAND_STATE = EMAGNET_OFF;	
								break;	
		default:	
					break;
	}
	
	id_receive = pData[4];		//receive id
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
	
}

void send_protocol(){
	
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
		
		case INIT:		
						switch ( dir_receive ) {
													
							case KEEPING:				// holding
											break;
											
							case STOP: 		drive_state = STOP;		// stop
											break;
											
							case DRIVE: 		
											direction_state = 'D';
											drive_state = DRIVE;		// arrive my position
											debug_data ('D');			// move
											break;
											
							case INIT_FIND_RESET: 		
											
											// i'm finding reed sw
											direction_state = 'I';		
											drive_state = DRIVE;
											debug_data ('I');
											
											break;
											
							case INIT_JUST_MOVE: 		
											
											// the frount car is finding reed sw
											direction_state = 'J';
											drive_state = DRIVE;
											break;
							
							case MOVE_WAIT_POS: 		
												// after reed sw
												direction_state = 'K';		
												drive_state = DRIVE;
												break;
											
							case ARRIVAL_WAIT_POS: 		
													break;
											
							default: 
										break;
							
						}
						
						sw_step_motor(step_speed[CHAR2INT(speed_receive)]);
						send_protocol();
						debug_string((unsigned char*)"\rINIT : ");
						debug_data (dir_receive);
						COMMAND_STATE = NORMAL;
						break;
		
		case REQUEST:	
						switch(dir_receive){
							
							// holding
							case KEEPING:		
											break;
							
							case STOP: 		drive_state = STOP;		// stop
											break;
							
							case DRIVE: 	
											drive_state = DRIVE;	// move
											debug_data ('D');		
											break;
							
						}
						
						sw_step_motor(step_speed[CHAR2INT(speed_receive)]);
						send_protocol();
						debug_string((unsigned char*)"\rREQUEST : ");
						debug_data (dir_receive);
						COMMAND_STATE = NORMAL;
						break;
						
		case MANUAL:	switch ( dir_receive ) {

							case KEEPING:				// holding
											break;
							
							case STOP: 		drive_state = STOP;		// stop
											break;
							
							case DRIVE: 	
											direction_state = DRIVE;
											drive_state = DRIVE;	// arrive my position
											
											debug_data ('D');		// move
											break;
							
							default : 	break;
							
						}
						
						sw_step_motor(step_speed[CHAR2INT(speed_receive)]);
						send_protocol();
						interrupt_count = 0;
						debug_string((unsigned char*)"\rMANUAL : ");
						COMMAND_STATE = NORMAL;
						break;
						
		case AUTO:		
						break;
						
		case LED_ON:	PORTF = ~0x01;
						direction_state = 'N';
						debug_data (direction_state);
						COMMAND_STATE = NORMAL;
						break;
						
		case LED_OFF:	PORTF = 0x01;
						direction_state = 'N';
						debug_data (direction_state);
						COMMAND_STATE = NORMAL;
						break;
						
		case EMAGNET_ON:	PORTF = ~0x02;
							direction_state = 'N';	//   
							debug_data (direction_state);
							COMMAND_STATE = NORMAL;
							break;
						
		case EMAGNET_OFF:	PORTF = 0x02;
							direction_state = 'N';	//   
							debug_data (direction_state);
							COMMAND_STATE = NORMAL;
							break;
			
		default :	break;	
	}
		
}

void init_message(void) {
	
	debug_string((unsigned char *)"Car avr init complete. ");
	debug_data('\r');
	debug_string((unsigned char *)"ID : ");
	debug_data(CAR_ID);
	debug_data('\r');
	debug_string((unsigned char *)"POS : ");
	debug_data(CAR_INIT_POS+'0');
	debug_data('\r');
	debug_string((unsigned char*)"INT0 : ");
	debug_data (interrupt_count+'0');
	debug_data('\r');
	debug_string((unsigned char*)"INT1 : ");
	debug_data (all_interrupt_count+'0');
	debug_data('\r');
	debug_data ('\r');
	
}
