

#ifndef INC_LCDCOMMUNICATION_H_
#define INC_LCDCOMMUNICATION_H_

typedef enum{
	IDLE_STATE,
	SEND_ACK_STATE,
	SENDING_DATA_STATE,
	ERROR_STATE
}State;


#endif /* INC_LCDCOMMUNICATION_H_ */
