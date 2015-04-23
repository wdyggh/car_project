#ifdef 	_CAR__H__
#define _CAR__H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <termios.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>

// TCP/IP Socket
#define HOST_IP "localhost"
#define port_address 9000
#define BUF_SIZE 256

// UART
#define DEBUG		0
#define UART_SPEED 	9600	// Baudrate
#define STX 		0x02	// Start of text
#define ETX 		0x03	// End of text
#define DELIMITER	','		// Delimiter
#define ACK 	'8'
#define NACK 	'9'


// Serial Rx State Value
#define RX_STATE_STX	0
#define RX_STATE_DATA	1

unsigned int rx_state = RX_STATE_STX;

// file descriptor
unsigned int sock;		// sock fd
unsigned int uart_fd;	// uart fd

// signal struct
struct sigaction act;

// io pin array
unsigned int output_pin_array[10] = { 0, 1, 2, 3 };
unsigned int main_input_pin_array[10] = { 24, 25, 26, 27, 28, 29 };
unsigned int sub_input_pin_array[10] = { 4, 5, 6, 21, 22, 23 };

// cars infomation
typedef struct _carInfo {
	unsigned char step_count_str[10];
	unsigned int step_count;
	unsigned char position;
	unsigned char ack_nack;
} CARINFO;

typedef struct _tx_carInfo {
	unsigned char id;
	unsigned char dir;
	unsigned char pos;
	unsigned char speed;
} TX_CARINFO;

CARINFO car_info[10];		// car info
TX_CARINFO tx_car_info[10];	// tx car info

unsigned char tx_protocol[70];
unsigned int tx_protocol_len=0;

unsigned char car_speed[4] = { '1', '2', '3', '4'};
unsigned char car_direction[3] = { 'N', 'S', 'D'};
unsigned char car_position[6] 	= { 'A', 'B', 'C', 'D', 'E', 'F' };
unsigned char car_init_position[8] = { 'A', 'B', 'C', 'D', 'A', 'B', 'C', 'D' };

// Command State: init=0, manual=1, auto=2, normal=3, request=4
enum CMD_STATES 	= 	{ INIT, MANUAL, AUTO, NORMAL, REQUEST };
enum CMD_STATE COMMAND_STATE = NORMAL;

// Command Char Array
unsigned COMMAND[5] = { 'I', 'M', 'A', 'N', 'R'};

// 
enum INIT_STATES 	= 	{ READY, START, INFO_REQ, CHECK, DONE };
enum MANUAL_STATES 	= 	{ READY, START, INFO_REQ, CHECK, DONE };
enum AUTO_STATES 	= 	{ READY, START, INFO_REQ, CHECK, DONE };

enum INIT_STATES	init_state 		= READY;
enum MANUAL_STATES	manual_state 	= READY;
enum AUTO_STATES 	AUTO_STATES 	= READY;

void ouch(int sig);		// signal handler function
void error_handling(char *message);	// error message function

int uart_init();		// uart configure function
void socket_config();	// socket configure function
void pin_io_init();		// pin i/o configure function
void device_init();		// configure exec function


// thread function
void *serial2socket(void * arg);
void *server2client(void * arg);
void *main_action_func(void * arg);


/* common function */

// Make and Send tx Protocol ( rpi -> avr )
void make_protocol(void);
void send_cmd_protocol(char *pData, int len);
void ready(void);
void start(void);
void info_request(void);
int check(void);
void done(void);

int avr_data_parsing(char *pData, int len);		// avr data parsing

#endif