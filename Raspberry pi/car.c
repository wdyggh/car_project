/*
 * 
 */

#include "car.h"

void ouch(int sig) {
	
	close(sock);
	// digitalWrite(output_pin_array[0], 1) ;       // Off
	write(1, "\rProgram Exit! \n", 17);
	exit(0);	// program exit

}

void error_handling(char *message) {
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}


int uart_init() {
	
	if( (uart_fd = serialOpen ("/dev/ttyAMA0", UART_SPEED)) < 0) {
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;
    } else {
		printf("\rSerial Open Sucess, BAUD : %d\n", UART_SPEED);
	}
	
	serialFlush(uart_fd); //fd
			
}

void socket_config() {
	struct sockaddr_in serv;
	
	bzero((char *)&serv, sizeof(serv));
	serv.sin_family=AF_INET;
	serv.sin_port=htons(port_address);
	serv.sin_addr.s_addr=inet_addr(HOST_IP);
	
	sock=socket(PF_INET, SOCK_STREAM, 0);   
	if(sock==-1)
		error_handling("socket() error");
	
	if(connect(sock, (struct sockaddr*)&serv, sizeof(serv))==-1)
		error_handling("connect() error!");
	else
		puts("Connected...........");
}

void pin_io_init() {
	
	int idx=0;
	
	for(idx=0; idx<6; idx++) {
		pinMode ( main_input_pin_array[idx], INPUT);
		pinMode ( sub_input_pin_array[idx], INPUT);
	}
		
	for(idx=0; idx<4; idx++) {
		pinMode ( output_pin_array[idx], OUTPUT);
	}
	
	// softPwmCreate (output_pin_array[0], 0, 100);
	softPwmCreate (output_pin_array[1], 0, 100);
}

void device_init() {
	
	if(wiringPiSetup() == -1) {
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		exit(1);
	}
	
	// signal handler func setting
	act.sa_handler = ouch;
	sigemptyset(&act.sa_mask);
	act.sa_flags = 0;			
	
	pin_io_init();
	uart_init();		// uart init
	//socket_config();	// Socket Configure
}

int avr_data_parsing(char *pData, int len) {

	int i=0, j=0;
	int delimiter_count=0;
	int id_num=0;

	for(i=0; i<len; i++) {
		
		if(pData[i] == DELIMITER) {
			delimiter_count++;
		} else if ( delimiter_count == 0 ) { 	/* STX */
			
		} else if ( delimiter_count == 1 ) { 	/* ID */
			
			id_num = atoi( &(pData+i) )-1;	// index: 0~9

		} else if ( delimiter_count == 2 ) { 	/* Step count */

			car_info[id_num].step_count_arr[j++] =  pData[i];

		} else if ( delimiter_count == 3 ) { 	/* Position */
			
			car_info[id_num].position =  pData[i];

		} else if ( delimiter_count == 4 ) {	/* ACK_NACK */
			
			car_info[id_num].ack_nack =  pData[i];
			
			if ( car_info[id_num].ack_nack ==  ACK) {
				car_info[id_num].step_count = strtoul(car_info[id_num].step_count_str, NULL, 16);
				// car_info[id_num].step_count = atoi(car_info[id_num].step_count_arr);
				return 1;
			} else {
				return -1;
			}

		} 
	}
	
}

void make_protocol(void) {

	int i, j;

	tx_protocol[0] = STX;
	tx_protocol[1] = DELIMITER;
	tx_protocol[2] = COMMAND[COMMAND_STATE];
	tx_protocol[3] = DELIMITER;

	switch( COMMAND_STATE ) {

		case INIT:	// make init protocol
					for( i=0; i<8; i++ ) {

						tx_car_info[i].id = (i+1)+48;
						tx_car_info[i].dir =  car_direction[2];		// 0:N, 1:S, 2:D
						tx_car_info[i].pos =  car_position[j++];
						tx_car_info[i].speed =  car_speed[2];
						
						if( j > 4 ) j = 0;	// car_position index init, for lead car
					}

					for( i=4, j=0; i<68; i+=8, j++) {

						tx_protocol[i++] = tx_car_info[j].id;
						tx_protocol[i++] = DELIMITER;
						tx_protocol[i++] = tx_car_info[j].dir;
						tx_protocol[i++] = DELIMITER;
						tx_protocol[i++] = tx_car_info[j].pos;
						tx_protocol[i++] = DELIMITER;
						tx_protocol[i++] = tx_car_info[j].speed;
						tx_protocol[i++] = DELIMITER;

					}
					tx_protocol[i++] = ETX;
					tx_protocol[i++] = '\0';
					tx_protocol_len = i;	
					break;

		case AUTO:
					break;
						
		default : 	break;	

	}

	
}

void send_cmd_protocol(char *pData, int len) {
	write(uart_fd, pData, len);	
}

void *server2client(void * arg) {
	
	char sock_str[100];
	int sock_str_len=0;
	
	while(1) {
		
		// ctrl + c input check
		if( sigaction(SIGINT, &act, 0) == -1 ) {
			perror("sigaction error");
			exit(1);
		}
		
		sock_str_len = read(sock, sock_str, sizeof(sock_str));
		
		// printf("\rserver -> avr read data :%s, length: %d \n", sock_str, sock_str_len);	
				
		// write(uart_fd, sock_str, sock_str_len);
	}
}

void *serial2socket(void * arg) {
	
	char uart_str[100];
	int uart_str_len=0, idx=0;
	char rx_ch;
	
	while(1){
	    
		// ctrl + c input check
		if( sigaction(SIGINT, &act, 0) == -1 ) {
			perror("sigaction error");
			exit(1);
		}
				
		read( uart_fd, &rx_ch, 1);

		switch(rx_state) {
			case RX_STATE_STX: 	if( rx_ch == STX ) {
									//printf(" \nstx: %c\n", rx_ch);
									uart_str[idx++] = rx_ch;
									rx_state = RX_STATE_DATA;
								}	
								break;
			case RX_STATE_DATA:	
								if(rx_ch == ETX) {
									uart_str[idx++] = rx_ch;	
									uart_str[idx] = '\0';
									uart_str_len = idx;
									rx_state = RX_STATE_STX;
									idx=0;
									
									avr_data_parsing(uart_str, uart_str_len);
									// write(sock, uart_str, strlen(uart_str));

								} else {
									uart_str[idx++] = rx_ch;
								}
								
								break;
			default: 	break;							
		}
	}
}

void *main_action_func(void * arg) {
	
	int position_check_count=0;
	
	switch( COMMAND_STATE ) {

		case NORMAL:	
						break;

		case INIT:	switch(init_state) {
						
						case READY :	ready();
										make_protocol();
										init_state = START;
										break; 
						
						case START :	start();
										init_state = INFO_REQ;
										break;
						
						case INFO_REQ : info_request();
										delay(50);
										init_state = CHECK;
										break;
						
						case CHECK :	init_state = CHECK;
										position_check_count = check();
										if( position_check_count > 0 ) {
											init_state = INFO_REQ;
										} else {
											init_state = DONE;
										}
										
										break;

						case DONE :		init_state = READY;
										COMMAND_STATE = NORMAL;
										break;												
					}
					break;

		case AUTO:
						break;
						
		default : 		break;								
	}

}

void ready(void) {
	/*
	* LED ON, Door OPEN
	*/
}

void start(void) {
	send_cmd_protocol(tx_protocol, tx_protocol_len);
}

void info_request(void) {
	
	char info_req_protocol[15];
	int idx=0, id;
	int info_req_protocol_len = 8;

	info_req_protocol[0] = STX;			// start of text
	info_req_protocol[1] = DELIMITER;	
	info_req_protocol[2] = COMMAND[REQUEST];	// command
	
	info_req_protocol[3] = DELIMITER;
	info_req_protocol[5] = DELIMITER;
	info_req_protocol[6] = ETX;
	info_req_protocol[7] = '\0';

	for (id=1; id<=8; id++) {
		
		info_req_protocol[4] = id + 48;	// ID
				
		write(uart_fd, info_req_protocol, info_req_protocol_len);
		delay(20);
	}
}

int check(void) {
	
	switch( COMMAND_STATE ) {
		
		case INIT:	int i=0;
					int position_check_count = 0;
					
					for( i=0; i<8; i++ ) {
						if( car_info[i].position != car_init_position[i] ) {
							position_check_count++;
						}
					}
										
					break;
		case AUTO:
					break;
						
		default : 	break;	
	}					
	
	return position_check_count;
}

void done(void) {

}
