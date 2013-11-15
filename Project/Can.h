
#ifndef CAN_H_
#define CAN_H_

//--------------------------------------------------------------------------------
// Prototypes
//--------------------------------------------------------------------------------
void CAN1_RX0_IRQHandler(void);
void init_CAN(void);
void can_send_PDO(unsigned char *data);
void can_set_send_PDO_mb(unsigned char lenght, unsigned int id, unsigned char rtr);
void can_send_SDO(unsigned char *data, unsigned char node, unsigned char rtr, unsigned char lenght);

//--------------------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------------------
#define COB_ID_EPOS_1 0x601
#define COB_ID_EPOS_2 0x602
#define COB_ID_EPOS_3 0x604

// Read Protocol
#define RECEIVE_ACT_POS 0x00606443		// Read actual Position
#define RECEIVE_ACT_VEL 0x00202843	// Read actual Velocity
#define RECEIVE_ACT_CUR 0x0020274B	    // Read actual current

// ACK
#define ACK_POS_SEND 0x00606460			// ACK of sent position
#define ACK_VEL_SEND 0x00202860			// ACK of sent velocity
#define ACK_CUR_SEND 0x00202760			// ACK of sent current




#endif /* CAN_H_ */
