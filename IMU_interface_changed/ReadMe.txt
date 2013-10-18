

Title: IMU_interface_changed

What is does : Simple program for the tesintg of usart3, the serial port on which the IMU is connected.

Notes: 	The transmitting part is tested using Putty.. and works perfectly

	The receiving part is tested using the IMU, which responds to Start_continious_mode command. The board receives whole data packets
				and after using a synchronization method, it seems that it desynchronizes only once.
				
				Further testing of the syncronization function should be done.
				
				A function which holds the state of the synchronization procedure during the previous interrupt and if it is the same 
				as the one now it justs sets the counter again, possibly without the need of re-enabling the DMA stream. In the circular mode DMA 
				doesn't have to be re-enabled, if the data counter stays the same.
 
		The receiving interrupt function doesn't test for serial communication errors after every byte is received, but only after the 
		DMA data transfer is complete.
	
	Not sure about the DMA circular mode, it seems to work for the usart6. It is used to avoid enabling the stream every time, which is time consuming.  

	It is only a testing program and doesn't have all the features the actual interface should have.The SendMsgIMU is back to the version
													that creates the message for the IMU


