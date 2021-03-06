

/* IMU serial interface functions */
void USART_IMU_Config(void);
void SendMsgIMU(unsigned short DataNumber,uint8_t *StartAdress);
void Start_Continious_Mode_IMU(void);
unsigned short calcCRC(unsigned char *pBuffer, unsigned short bufferSize);
void Read_data_IMU(int shift);
void Stop_Continious_Mode(void);

/* High Level PC serial interface functions */
void USART_PC_Config(void);
void Start_Continious_Mode_PC(void);
void SendMsgHELIOS(unsigned short DataNumber,uint8_t *StartAdress);
void Read_data_Helios(int shift);
void Send_Sensor_Values_to_HELIOS(unsigned char Anzahl);
unsigned char calcModulo256(char *pBuffer, unsigned short bufferSize);

/* Exported typedef ----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;


/*------------------------------- IMU------------------------------ */
/* Message */
#define SYNCX_IMU   		0xFF
#define STX_IMU     		0x02
#define ETX         		0x03
#define Command_Size        11
#define MESS_LENGTH_RX      60
#define IMU_TX_BUFFER		&TxMessageIMU;
#define IMU_RX_BUFFER		&RxMessageIMU;


/* IMU commands */
#define SBG_SET_CONTINUOUS_MODE 			0x53
#define SBG_CONTINIOUS_DEFAULT_OUTPUT       0x90


/*--------------------------------High Level PC---------------------*/

/* Message */
#define SYNCX_H                     'H'                 // 'H' = 72
#define CMD_SEND_DATA_HELIOS        'D'                 // 'D' = 68
#define TX_BUFFERSIZE				165
#define RX_BUFFERSIZE 				20
#define HELIOS_TX_BUFFER        	&TxMessageHELIOS[0]
#define HELIOS_RX_BUFFER        	&RxMessageHELIOS[0]
#define ANZ_SENSOR_BYTE_HELIOS  	160 // 40 floats = 160 bytes

//--------------------------------------------------------------------------------
// extern
//--------------------------------------------------------------------------------
extern unsigned char RxMessageIMU[60];
extern unsigned char TxMessageIMU[48];

extern unsigned char RxMessageHELIOS[20];
extern unsigned char TxMessageHELIOS[200];

extern unsigned int IMU_data_for_PC;






