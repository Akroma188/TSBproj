/*
 * main.c
 *
 * Created: 03/03/2017 17:55:39
 * Author : Dinis Rodrigues
 */ 
#define F_CPU	16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>


#define CONF_CANBT1  0x06         
#define CONF_CANBT2  0x04        
#define CONF_CANBT3  0x13

#define NBR_OF_MOB		6


#define FALSE	0

#define TRUE	1




void can_init(void);
uint8_t can_setup_rx(uint32_t mob_id, uint32_t mob_msk, uint8_t mob_dlc);
uint8_t can_setup_tx(uint32_t mob_id, uint8_t * mob_data, uint8_t mob_dlc);
void can_enable(void);
void can_disable(void);
uint8_t can_free_rx(uint8_t mob);
uint32_t _can_get_id(void);
void _can_set_id(uint32_t identifier);
void _can_set_msk(uint32_t mask);
uint8_t _can_get_free_mob();



/*##################################################################################################################################################
												Inicialization of CAN protocol:
-> Take attencion to the baudrate, your communication may not work if its not the appropriate baudrate for your system (baudrate/lenght of wire)
->Clearing all mobs its a MUST
####################################################################################################################################################*/
void can_init(void) {
	CANGCON = (1<<SWRES); // reset CAN
	CANTCON = 0x00; //set timing prescaler to zero

	CANBT1 = CONF_CANBT1; // set baudrate
	CANBT2 = CONF_CANBT2; // set baudrate
	CANBT3 = CONF_CANBT3; // set baudrate

	CANGIE = 0xFE; // (1<<ENIT) | (1<<ENRX) | (1<<ENTX); //enable TXOK an RXOK interrupts
	CANIE1 = 0; // for compatibility
	CANIE2 = (1<<IEMOB5) | (1<<IEMOB4) | (1<<IEMOB3) | (1<<IEMOB2) | (1<<IEMOB1) | (1<<IEMOB0); // enable interrupts on all MOb

	//clear all MOb
	for (uint8_t mob_number = 0; mob_number < NBR_OF_MOB; mob_number++) {
		CANPAGE = (mob_number << MOBNB0); // select each MOb in turn

		//initiate everything to zero. IMPORTANT!!
		CANCDMOB = 0x00;
		CANSTMOB = 0x00;

		CANIDT4 = 0x00;
		CANIDT3 = 0x00;
		CANIDT2 = 0x00;
		CANIDT1 = 0x00;
		CANIDM4 = 0x00;
		CANIDM3 = 0x00;
		CANIDM2 = 0x00;
		CANIDM1 = 0x00;
	}
}




/*##################################################################################################################################################
														Inicialization RX:
-> You need to do this at least one time in order to enable the communication, according to the datasheet
-> Setting a mask to 0x00 could make things easier to init
####################################################################################################################################################*/
uint8_t can_setup_rx(uint32_t mob_id, uint32_t mob_msk, uint8_t mob_dlc) {

	uint8_t free_mob = _can_get_free_mob(); // first free mob
	if (free_mob == 0xFF) {
		return 0xFF; //no free mob, error
	}
	CANPAGE = free_mob << MOBNB0; // select first free MOb for use

	_can_set_id(mob_id); //id to compare against
	_can_set_msk(mob_msk); //mask for comparing id

	mob_dlc = (mob_dlc > 8) ? 8 : mob_dlc; // expected number of data bytes

	CANCDMOB = (1 << CONMOB1) | (1 << IDE) | (mob_dlc << DLC0); // configure MOb for reception of mob_dlc number of data bytes

	return free_mob; // the configured MOb
}




/*##################################################################################################################################################
														Inicialization TX:
-> If you are working with a sytem where you only want to trasmit information, use this function on your loop
-> Take close attention to datasheet to undersant the protocol, 29bits or 11 bits, it will alter you ID
####################################################################################################################################################*/
uint8_t can_setup_tx(uint32_t mob_id, uint8_t * mob_data, uint8_t mob_dlc) {

	uint8_t free_mob = _can_get_free_mob(); //first free MOb
	if (free_mob == 0xFF) {
		return 0xFF; //no free mob, error
	}
	CANPAGE = free_mob << MOBNB0; // select first free MOb for use

	CANSTMOB = 0x00; //clear MOb status

	_can_set_id(mob_id); // configure ID

	for (uint8_t i = mob_dlc; i > 0; i--) {
		CANMSG = mob_data[i-1]; // Set data. Reversed to send uint16_t non inverted. AVR is little endian, this way we can read the information with CANview. Arrays are sent backwards.
	}

	CANCDMOB = (1<<CONMOB0) | (1 << IDE) | (mob_dlc << DLC0); // enable transmission and set DLC

	return free_mob; // return the MOb used
}



/*##################################################################################################################################################
														CAN Enable:
-> Simple funtion after setting uo the RX init, this will enable your communication after the 11 bits are receieved 

####################################################################################################################################################*/

void can_enable(void) {
	CANGCON |= (1 << ENASTB); //enable CAN
}



/*##################################################################################################################################################
														CAN disable:
-> Turn off the communication system 

####################################################################################################################################################*/

void can_disable(void) {
	CANGCON &= ~(1 << ENASTB); //disable CAN
}

/*##################################################################################################################################################
														Free the mob used in RX:
-> Only if you want to disable it, not really necessary

####################################################################################################################################################*/
uint8_t can_free_rx(uint8_t mob) {
	if (mob >= NBR_OF_MOB) {
		return 0; // not a mob, error
	}
	CANPAGE = mob << MOBNB0;

	//reset everything to zero
	CANCDMOB = 0x00;
	CANSTMOB = 0x00;

	CANIDT4 = 0x00;
	CANIDT3 = 0x00;
	CANIDT2 = 0x00;
	CANIDT1 = 0x00;
	CANIDM4 = 0x00;
	CANIDM3 = 0x00;
	CANIDM2 = 0x00;
	CANIDM1 = 0x00;

	return 1;
}

/*##################################################################################################################################################
														Get ID:
->------------------
####################################################################################################################################################*/
uint32_t _can_get_id(void) {
	return ((uint32_t)CANIDT4 >> 3) | ((uint32_t)CANIDT3 << 5) | ((uint32_t)CANIDT2 << 13) | ((uint32_t)CANIDT1 << 21);
}




/*##################################################################################################################################################
														Set ID:
->Input parameter is a 32 bit integer, remember that
->Also remember the datasheet, ID varies for protocol A and B (ID3 & ID4 will not be used in protocol A)
####################################################################################################################################################*/
void _can_set_id(uint32_t identifier) {
	uint32_t id_v = (identifier << 3);

	CANIDT1 = *((uint8_t *) &id_v + 3);
	CANIDT2 = *((uint8_t *) &id_v + 2);
	CANIDT3 = *((uint8_t *) &id_v + 1);
	CANIDT4 = *((uint8_t *) &id_v + 0);
}



/*##################################################################################################################################################
														Set mask:
-> For testing, you should use 0x00, this will let anything pass through
-> As you go along, then you can test other masks
####################################################################################################################################################*/
void _can_set_msk(uint32_t mask) {
	uint32_t mask_v = (mask << 3);
	CANIDM1 = *((uint8_t *) &mask_v + 3);
	CANIDM2 = *((uint8_t *) &mask_v + 2);
	CANIDM3 = *((uint8_t *) &mask_v + 1);
	CANIDM4 = *((uint8_t *) &mask_v + 0) | (1<<RTRMSK) | (1<<IDEMSK);
}

/*##################################################################################################################################################
														Gets the first free mob:
-> This is like an automated process, in the protocol, the mob is freed as its used.
-> This function will pick the first one that is free to use and configure
####################################################################################################################################################*/
uint8_t _can_get_free_mob(void) {
	for (uint8_t mob = 0; mob < NBR_OF_MOB; mob++) {
		if (!(CANEN2 & (1 << mob))) {
			return mob;
		}
	}
	return 0xFF;
}


/*##################################################################################################################################################
														Funtion in interrupt for RX flag:
-> Handles the "RXOK flag"
####################################################################################################################################################*/
void _can_handle_RXOK() {
	//uint8_t mob = (CANPAGE & 0xF0) >> 4; // get mob number
	//uint32_t id = _can_get_id(); // get id
	//uint8_t dlc = CANCDMOB & 0x0F; // get dlc
	//uint8_t data[dlc]; // create vector for data

	//read data
	//for (uint8_t i = dlc; i > 0; i--) {
	//	data[i-1] = CANMSG; //CANMSG autoincrements, !AINC = 0. Reversed to send uint16_t non inverted. AVR is little endian, this way we can read the information with CANview. Arrays are sent backwards. inverted.
	//}

	// send information to extern function in application to act on information
	//CAN_ISR_RXOK(mob, id, dlc, data);

	CANCDMOB &= ~((1 << CONMOB1) | (1 << CONMOB0)); //disable MOb
	//_NOP();
	CANCDMOB |= (1 << CONMOB1); // re-enable reception
}



/*##################################################################################################################################################
														Funtion in interrupt for TX flag:
-> Clears the register
####################################################################################################################################################*/
void _can_handle_TXOK() {
	//uint8_t mob = (CANPAGE & 0xF0) >> 4; // get mob number
	//uint32_t id = _can_get_id(); // get id
	//uint8_t dlc = CANCDMOB & 0x0F; // get dlc
	//uint8_t data[dlc]; // create vector for data

	//read data
	//for (uint8_t i = dlc; i > 0; i--) {
	//	data[i-1] = CANMSG; //CANMSG autoincrements, !AINC = 0. Reversed to send uint16_t non inverted. AVR is little endian, this way we can read the information with CANview. Arrays are sent backwards.
	//}

	//CAN_ISR_TXOK(mob, id, dlc, data); // extern function if more actions are required after TXOK
	CANCDMOB = 0x00; // clear control register
}



//uint8_t can_data_equals(uint8_t * data1, uint8_t * data2, uint8_t dlc) {
//	for (uint8_t i = 0; i < dlc; i++) {
//		if (data1[i] != data2[i]) {
//			return FALSE;
// 		}
//	}
//	return TRUE;
//}



/*##################################################################################################################################################
														Interruption funtion, when a flag or error is set:
-> Handles all the erros and flags that may occur.
-> It only resets the errors and flags, does not solve your problem if there is one!
####################################################################################################################################################*/


ISR (CAN_INT_vect) {
	uint8_t save_CANPAGE = CANPAGE; // save CANPAGE
	
	// check for valid MOb!!!
	uint8_t mob = (CANHPMOB & 0xF0) >> 4; // select MOb with highest priority interrupt
	
	if (CANSIT2 && (1 << mob)) {
		CANPAGE = mob << 4;
		if (CANSTMOB & (1 << RXOK)) { //test for RXOK
			CANSTMOB &= ~(1 << RXOK); // clear interrupt flag
			_can_handle_RXOK(); //handle RXOK
		}
		else if (CANSTMOB & (1 << TXOK)) { //test for TXOK
			CANSTMOB &= ~(1 << TXOK); // clear interrupt flag
			_can_handle_TXOK(); //handle TXOK
		}
		else { // any other interrupt, most likely an error
			//FIXME: Restart RX if needed.
			//CAN_ISR_OTHER(); // extern function, handles errors
			CANSTMOB = 0x00; // clear interrupt flag, FIXME: errors not handled well
		}
	}
	else {
		CANGIT = 0xFF; // clear general interrupts
	}
	CANPAGE = save_CANPAGE; //restore CANPAGE
}




/*##################################################################################################################################################
														Main Function:
-> This main, is intended to only transmit!
-> We setup RX only to enable the communication!
-> In the loop it's where we transmit
####################################################################################################################################################*/

int main(void){
	uint8_t mob;
	uint8_t data[8];
	
	
	//for(uint8_t i=0; i<8 ; i ++){
	//	data[0] = 0xFF;
	//}
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFF;
	
	can_init();
	mob=can_setup_rx(0x00, 0x00, 4);
	sei();
	can_enable();
	
	while(1) {
		_delay_ms(1000);
		can_setup_tx(0xFFFFFFFF, data, 8); 
	}
}
