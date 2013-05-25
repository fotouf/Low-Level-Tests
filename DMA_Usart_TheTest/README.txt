This is test code for the DMA-USART.

This version takes data from the RAM and stores them in the DMA FIFO with a 8byte uninterruptable burst.
Then it should send the data to the USART with single byte uninterruptable bursts, waiting after each one the USART to be ready to receive the next one.
Despite that the USART flags TXE and TC have the values that they should and the DMA controller flags too, the DMA controller doesn't transfer any byte.

The initial versions had the same DMA configuration with the one the ST examples have, but data didn't even been transfered to the DMA controller.

Note: I was putting the functions that switch on and off the leds based on the led conditions at a previous test step.      