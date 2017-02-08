#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


//Function to configure the pins for motion
void motion_pin_config (void){
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function to initialize ports
void port_init(){
 motion_pin_config();
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction){
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibble as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibble to 0
 PortARestore |= Direction; 	// adding lower nibble for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}


void forward (void){     //both wheels forward
	motion_set(0x06);
}

void back (void){       //both wheels backward
	motion_set(0x09);
}

void left (void){       //Left wheel backward, Right wheel forward
	motion_set(0x05);
}

void right (void){      //Left wheel forward, Right wheel backward
	motion_set(0x0A);
}

void soft_left (void){  //Left wheel stationary, Right wheel forward
	motion_set(0x04);
}

void soft_right (void){ //Left wheel forward, Right wheel is stationary
	motion_set(0x02);
}

void soft_left_2 (void){//Left wheel backward, right wheel stationary
	motion_set(0x01);
}

void soft_right_2 (void){ //Left wheel stationary, Right wheel backward
	motion_set(0x08);
}

void stop (void){       //hard stop
	motion_set(0x00);
}

//Function to initialize the device
void init_devices (void){
 cli(); //Clears the global interrupts
 port_init();
 sei(); //Enables the global interrupts
}


//Main Function
int main(){

	init_devices();//initialize
		
	while(1){//infinite while loop
	
		forward(); //both wheels forward
		_delay_ms(1000);

		stop();						
		_delay_ms(500);
	
		back(); //bpth wheels backward						
		_delay_ms(1000);

		stop();						
		_delay_ms(500);
		 
	}
}

