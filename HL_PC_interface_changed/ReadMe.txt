

Title: HL_PC_interface_changed

What is does : Simple program for the tesintg of usart6, the serial port on which the High Level PC is connected.

Notes: 	The transmitting part is tested using Putty.. and works perfectly

	The receiving part is tested with the high level PC and works fine (the board received C_STOP and C_Position).
		One concern is that if we manage to start unsynchronized, there will be many synchronization errors 
		and the board won't always receive a correct data package.
		
		We should use the synchronization function of the IMU interface.

		The receiving interrupt function doesn't test for serial communication errors after every byte is received, but only after the 
		DMA data transfer is complete.
	
	It is only a testing program and doesn't have all the features the actual interface should have. E.g. it doesn't send the right data
	to the PC, it sends a test package.