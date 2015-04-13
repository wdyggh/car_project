#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
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
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <wiringPi.h>

//#define HOST_IP	"220.68.139.15"
#define HOST_IP	"220.69.240.54"
#define port_address	9000
#define STX 0x02
#define ETX 0x03

// SPEED, START, STOP
#define START 'S'
#define STOP 'T'
#define SPEED_UP 'U'
#define SPEED_DOWN 'D'

#define RX_STATE_STX 0	
#define RX_STATE_DATA 1

// #define UART_SPEED B115200
#define UART_SPEED B9600

unsigned int rx_state = RX_STATE_STX;
int	sock;	// sock fd
int uart_fd;		// serial fd
char rx_string[100];	
char sock_msg[100];
int rx_length=0;
int rx_states;

struct sigaction act;
void error_handling(char *message);
int uart_init();
void socket_config();
void device_init();

void *serial2socket(void * arg);	// serial2socket thread main
void *socket2serial(void * arg);	// read uart thread main
void *rpi2avr_tx_func(void * arg);
void Slave_Request(char id, int fd, char speed, char dir);

int main(void) 
{
	// pthread_t serial2socket_thread, socket2serial_thread;
	pthread_t serial2socket_thread, socket2serial_thread, rpi2avr_thread;
	
	void * thread_return;
	int status;
	char tx_str[50];
	
	device_init();
			
	// 스레드 생성
	pthread_create(&serial2socket_thread, NULL, serial2socket, NULL);
	pthread_create(&socket2serial_thread, NULL, socket2serial, NULL);
	// pthread_create(&rpi2avr_thread, NULL, rpi2avr_tx_func, NULL);
	
	// 스레드 종료 대기
	pthread_join(serial2socket_thread, &thread_return);
	pthread_join(socket2serial_thread, &thread_return);
	// pthread_join(rpi2avr_thread, &thread_return);
		
	return 0;
	
}

void Slave_Request(char id, int fd, char speed, char dir)
{
	char buf[4]={0};
	unsigned char tx_string[30];
	int i = 0, j = 0, crc = 0;
	int byte_len = 0;
	
	tx_string[i++] = STX;		// STX	0x02
	tx_string[i++] = ':';		
	tx_string[i++] = id;		// NUM 	'1'==0x31
	tx_string[i++] = ':';		
	tx_string[i++] = '1';		// COMMAND 	'1'
	tx_string[i++] = ':';		
	tx_string[i++] = speed;		// SPEED
	tx_string[i++] = ':';
	tx_string[i++] = dir;		// START, STOP
	tx_string[i++] = ':';
	
	// DebugPrint("crc: %02X\r\nbuf[0]: %02x\r\nbuf[1]: %02x\r\n", crc, buf[0], buf[1]);
	
	for(j = 0; j < i; j++) crc ^= tx_string[j];	// CRC ..
	sprintf(buf, "%02X", crc&0xff);

	// DebugPrint("crc: %02X\r\nbuf[0]: %02x\r\nbuf[1]: %02x\r\n", crc, buf[0], buf[1]);
	
	tx_string[i++] = buf[0];		// CRC High 
	tx_string[i++] = buf[1];    	// CRC Low 	
	tx_string[i++] = ':';
	tx_string[i++] = ETX;       	// ETX 		0x03
	tx_string[i]   =  '\0';     	// NULL 	0

	// DebugPrint("tx_string: %c\r\ni: %d\r\n" , tx_string[1], i);
	// write(fd, "tx_data", 7);
	byte_len = write(fd, tx_string, strlen(tx_string));
	printf("\rtx string write data : %s\n", tx_string);
	printf("\rtx string write byte : %d\n", byte_len);


}

void error_handling(char *message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

int uart_init() {
		
	struct termios newtio, oldtio;
	int res;
	int status;
	
	// if ((uart_fd = open ("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK )) == -1)
	//if ((uart_fd = open ("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK )) == -1)
	if ((uart_fd = open ("/dev/ttyAMA0", O_RDWR | O_NOCTTY )) == -1)
		return -1 ;

	tcgetattr(uart_fd, &oldtio);
	memset(&newtio, 0, sizeof(newtio));
	newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;
	
	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &newtio);

	usleep (10000) ;	// 10mS
}


void socket_config() {
	
	struct sockaddr_in serv;
	int flag = 1;
	
	// 서버 socket 구조 설정
	bzero((char *)&serv, sizeof(serv));
	serv.sin_family = PF_INET;
	serv.sin_port=htons(port_address);
	serv.sin_addr.s_addr=inet_addr(HOST_IP);
	
	sock=socket(PF_INET, SOCK_STREAM, 0);   
	if(sock==-1)
		error_handling("socket() error");
	
	if(connect(sock, (struct sockaddr*)&serv, sizeof(serv))==-1)
		error_handling("connect() error!");
	else
		puts("Connected...........");
	
	// int result = setsockopt(sock,            /* socket affected */
							//IPPROTO_TCP,     /* set option at TCP level */
						//TCP_NODELAY,     /* name of option */
						//	(char *) &flag,  /* the cast is historical cruft */
							//sizeof(int));    /* length of option value */	
}


void device_init() {

	if(wiringPiSetup() == -1)
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		exit(1);
	}
	
	socket_config();
	uart_init();
	
}


void *socket2serial(void * arg) {
	
	char test_str[100];
	int test_str_len=0;
	
	while(1) {
		
		//pthread_mutex_lock(&mutex);
		//tcflush(uart_fd, TCIFLUSH);
		test_str_len = read(sock, test_str, sizeof(test_str));
		printf("\rserver -> avr read data :%s, length: %d \n", test_str, test_str_len);	
		//pthread_mutex_unlock(&mutex);
		
		//pthread_mutex_lock(&mutex);
		write(uart_fd, test_str, test_str_len);
		//pthread_mutex_unlock(&mutex);
	}
}

void *serial2socket(void * arg){
	
	char rx_string[100];
	int rx_str_len=0, idx=0;
	char rx_ch;
	
	while(1){
	    /*
		test_str_len = read( uart_fd, rx_string, sizeof(rx_string));
		write( sock, rx_string, rx_str_len);
		*/
				
		read( uart_fd, &rx_ch, 1);
		switch(rx_state) {
			case RX_STATE_STX: 	if( rx_ch == STX ) {
									//printf(" \nstx: %c\n", rx_ch);
									rx_string[idx++] = rx_ch;
									rx_state = RX_STATE_DATA;
								}	
								break;
			case RX_STATE_DATA:	
								if(rx_ch == ETX) {
									rx_string[idx++] = rx_ch;	
									rx_string[idx] = '\0';
									rx_str_len = idx;
									rx_state = RX_STATE_STX;
									idx=0;
									
									write(sock, rx_string, strlen(rx_string));
								} else {
									rx_string[idx++] = rx_ch;
								}
								
								break;
			default: 	break;							
		}
				
		//printf("\ravr -> server read data :%s, length: %d \n", test_str, test_str_len);
				
	}	
}

void *rpi2avr_tx_func(void * arg) {
	
	int id_idx;
	while(1) {
		for(id_idx=0; id_idx<1; id_idx++) {
			Slave_Request( id_idx+'1', uart_fd, SPEED_UP, STOP );
			sleep(1);
			
		}
	}
}
