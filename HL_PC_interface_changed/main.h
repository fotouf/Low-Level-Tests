



//Additional definitions---------------------------------------------------------//

/* Transmit buffer size */
#define BUFFERSIZE                       10
#define MESS_LENGTH_RX          20
#define SYNCX_H                     'H'                 // 'H' = 72

// Mode control
enum control{C_POSITION, C_VELOCITY, C_ACCELERATION, C_STOP, C_FREEZE, C_ROTONDO, C_HULL_POSITION, C_ACCELERATION_WZ, C_MODE_4,
			 C_MODE_5, C_MODE_6, C_MODE_7, C_MODE_8, C_MODE_9, C_MODE_10, C_MODE_11};
enum control Control_Mode;


