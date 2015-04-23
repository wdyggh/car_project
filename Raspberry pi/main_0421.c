
#include "car.h"

// Main Function Start ================================
int main( ) 
{	
	pthread_t 	serial2socket_thread, 
				server2serial_thread, 
				main_action_thread;

	void * thread_return;
 
	device_init();
		
	//pthread_create(&serial2socket_thread, NULL, serial2socket, NULL);
	//pthread_create(&socket2serial_thread, NULL, socket2serial, NULL);
	pthread_create(&main_action_thread, NULL, main_action_func, NULL);
	
	//pthread_join(serial2socket_thread, &thread_return);
	//pthread_join(socket2serial_thread, &thread_return);
	pthread_join(main_action_thread, &thread_return);
	
	return 0;
	
}
// Main Function End ================================
