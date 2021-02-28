/*
    TMP Test Code
	5-31-10
    Copyright Spark Fun Electronics© 2010
    Nathan Seidle
	
	Example code for the TMP102 11-bit I2C temperature sensor
	
	You will need to connect the ADR0 pin to one of four places. This code assumes ADR0 is tied to VCC. 
	This results in an I2C address of 0x93 for read, and 0x92 for write.
	
	This code assumes regular 12 bit readings. If you want the extended mode for higher temperatures, the code
	will have to be modified slightly.

*/
//#include <stdio.h>
//#include <avr/io.h>
//
//#include "i2c.h" //Needed  for I2C sensors

#include "drivers/tmp102.h"


//int main (void)
//{
	//int16_t temperature = 0;
//
    //ioinit(); //Boot up defaults
//
	//printf("TMP102 Example\n");
//
	//while(1)
	//{
		//temperature = tmp102Read();
		//printf("temperature = %dC\n", temperature);
//
		//if ((UCSR0A & _BV(RXC0))) //Check for incoming RX characters
			//if(UDR0 == 'x') break;
			//
		//delay_ms(500); //4Hz readings by default
		//
		//PORTB ^= (1<<5); //Toggle status LED
	//}
	//
	//printf("You've gone too far!");
	//while(1);
//
    //return(0);
//}

//Read a tmp102 sensor on a given temp_number or channel
int16_t sens_t_tmp102_read(void)
{
	//uint8_t msb, lsb;
	//int16_t temp;
	
	const portTickType max_block_time_ticks = (portTickType) ( 200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;


	// Configure the read_parameters structure to define the read operation.
	i2c_pck.chip = SENS_TEMP_ADDR;
	i2c_pck.buffer = data_buffer;
	i2c_pck.length = 2;
	i2c_pck.addr[0] = (uint8_t) SENS_TEMP_REG;
	//read_parameters.addr[1] = (uint8_t) SENS_TEMP_REG + 1;
	i2c_pck.addr_length = 1;
	// Receive the data into receive_buffer. Don't wait any longer than 200ms
	// to get exclusive access to the port (other FreeRTOS tasks will
	// execute during any wait time.
	if(freertos_twi_read_packet(freertos_twi, &i2c_pck,	max_block_time_ticks) != STATUS_OK)
	{
		//Failed
		return 0;
	}
	else
	{

		uint16_t t;
		
		t = (data_buffer[0]<<8) | data_buffer[1];
		t >>= 4; //The TMP102 temperature registers are left justified, correctly right justify them
		//
		////The tmp102 does twos compliment but has the negative bit in the wrong spot, so test for it and correct if needed
		if(t & (1<<11))
			t |= 0xF800; //Set bits 11 to 15 to 1s to get this reading into real twos compliment

		//printf("%02d\n", temp);

		//But if we want, we can convert this directly to a celsius temp reading
		//temp *= 0.0625; //This is the same as a divide by 16
		//temp >>= 4; //Which is really just a shift of 4 so it's much faster and doesn't require floating point
		//Shifts may not work with signed ints (negative temperatures). Let's do a divide instead
		//temp /= 16;

		return(t);
	}
	return(1);
	
}

int16_t sens_t_tmp102_setup(void)
{
		const portTickType max_block_time_ticks = (portTickType) ( 200UL / portTICK_RATE_MS) * portTICK_RATE_MS_LESS_1_MS;
	
		// Configure the twi_packet_t structure to define the write operation
		i2c_pck.chip = SENS_TEMP_ADDR;
		i2c_pck.buffer = data_buffer;
		i2c_pck.length = 1;
		i2c_pck.addr[0] = (uint8_t) (SENS_TEMP_REG & 0xff);
		//write_parameters.addr[1] = (uint8_t) (calculated_address & 0xff);
		i2c_pck.addr_length = 1;
		// Attempt to write the data to the port referenced by the freertos_twi
		// handle. Wait a maximum of 200ms to get exclusive access to the port
		// (other FreeRTOS tasks will execute during any waiting time).
		if(freertos_twi_write_packet(freertos_twi, &i2c_pck, max_block_time_ticks) != STATUS_OK)
		{
			// The data was not written successfully, either because there was
			// an error on the TWI bus, or because exclusive access to the TWI
			// port was not obtained within 200ms.
			return(0);
		}
	
		return(1);
}
	



	//i2cSendStart();	
    //i2cWaitForComplete();
	//
	//i2cSendByte(TMP_WR); //We want to write a value to the TMP
	//i2cWaitForComplete();
//
	//i2cSendByte(TEMP_REG); //Set pointer regster to temperature register (it's already there by default, but you never know)
	//i2cWaitForComplete();
	//
	//i2cSendStart();
	//
	//i2cSendByte(TMP_RD); // Read from this I2C address, R/*W Set
	//i2cWaitForComplete();
	//
	//i2cReceiveByte(TRUE);
	//i2cWaitForComplete();
	//msb = i2cGetReceivedByte(); //Read the MSB data
	//i2cWaitForComplete();
//
	//i2cReceiveByte(FALSE);
	//i2cWaitForComplete();
	//lsb = i2cGetReceivedByte(); //Read the LSB data
	//i2cWaitForComplete();
	//
	//i2cSendStop();
	
	//printf("0x%02X ", msb);
	//printf("0x%02X ", lsb);
	
	//Test
	//msb = 0b11100111;
	//lsb = 0b00000000; //From the datasheet, -25C
	
	//temp = (msb<<8) | lsb;
	//temp >>= 4; //The TMP102 temperature registers are left justified, correctly right justify them
	//
	////The tmp102 does twos compliment but has the negative bit in the wrong spot, so test for it and correct if needed
	//if(temp & (1<<11))
		//temp |= 0xF800; //Set bits 11 to 15 to 1s to get this reading into real twos compliment

	//printf("%02d\n", temp);

	//But if we want, we can convert this directly to a celsius temp reading
	//temp *= 0.0625; //This is the same as a divide by 16
	//temp >>= 4; //Which is really just a shift of 4 so it's much faster and doesn't require floating point
	//Shifts may not work with signed ints (negative temperatures). Let's do a divide instead
	//temp /= 16;