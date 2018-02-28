/*
 * NAND_Flash.c
 *
 * Created: 2/23/2018 6:34:06 PM
 *  Author: kmcarrin
 */ 

#include "NAND_Flash.h"

/* TODO:
 * 1) Update "SPI_0" component to be named something better, like "SPI_FLASH".
 * 2) Get updated config file for line ending stuff.
 * 3) CS pins will ultimately need unique names for each device connected to micro.
 */

/* CONSTANT DECLARATIONS */
const uint8_t RESET[1]      = {0xFF};                   //Command to reset the memory device
const uint8_t GET_FEAT[2]   = {0x0F, 0xC0};             //Command to get the current contents of the status register
const uint8_t SET_WEL[1]    = {0x06};                   //Command to set the write enable bit in in the status register
const uint8_t PROG_LOAD[3]  = {0x02, 0x00, 0x00};       //Command to load data from micro into cache of memory device. Last 2 bytes page column address
const uint8_t PEXEC[4]      = {0x10, 0x00, 0x00, 0x00}; //Command to program the main memory array from the memory devices cache register
const uint8_t PAGE_READ[4]  = {0x13, 0x00, 0x00, 0x10}; //Command to read data from main array into data cache
const uint8_t READ_CACHE[3] = {0x03, 0x00, 0x00};       //Command to read data from memory device cache to SPI buffer

/* MASKS */
const uint8_t BUSY_MASK     = 0x01;                     //Mask for checking if the flash is busy
const uint8_t PROG_FAIL     = 0b00001000;               //Mask for checking if the memory was programmed successfully
const uint8_t WEL_BIT       = 0x02;                     //Mask for checking if write enable is high

void flash_InitSPI()
{
	/* Associate flash buffers with SPI device and set
     * buffer size. */
    spi_buff.size  = BUFFER_SIZE;
    spi_buff.rxbuf = MISO;
    spi_buff.txbuf = MOSI;

    /* Setup SPI IO */
	/* Set clock mode and enable SPI */
	spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
	spi_m_sync_enable(&SPI_0);
}

void flash_InitBuffers()
{
	int i; 
	
	/* Initialize input and output buffers */
	for(i = 0; i < BUFFER_SIZE; i++)
	{
		MISO[i] = 0xFF;
		MOSI[i] = 0x00;
	}
}

void flash_SetSPI_BufferSize(int newSize)
{
    spi_buff.size = newSize;
}


/**********************
* RESET SLAVE DEVICE *
**********************/ 
/* Issue the RESET command: 0xFF. Device reset on startup
* is not mandatory for this part, but is recommended. This
* also ensures previous run settings are reset within the 
* device. 
* WILL ONLY RESET IF DEVICE IS NOT BUSY - nonblocking */
// returns status register value
uint8_t flash_Reset()
{
    /* Get the current status of the device. */
    uint8_t status = flash_Status();

    /* Only reset device if the device is not currently busy. */
    if((status & BUSY_MASK) == 0)
    {
        /* Set SPI buffer to send only 1 byte of command data. 
         * Put the command in the output buffer. */
        flash_SetSPI_BufferSize(1);
        MOSI[0] = RESET[0];

        /* Set slave select (CS) active low to communicate. */
        gpio_set_pin_level(GPIO_PIN(CS), false);

        /* Read/write over SPI */
        spi_m_sync_transfer(&SPI_0, &spi_buff);

        /* De-select device by pulling CS high. */
        gpio_set_pin_level(GPIO_PIN(CS), true);

        /* Reinitialize output buffer. */
        MOSI[0] = 0;

        /* Get updated status. */
        status = flash_Status();
    }

    return (status);
}

/*************************
* CHECK STATUS REGISTER *
*************************/ 
/* Issue the GET FEATURES command: 0x0F 
* Send address of FEATURES register: 0xC0
* Put command in MOSI buffer.  */
//returns status register value
uint8_t flash_Status()
{
    /* Set buffer size to 3 and put command in output buffer:
     *      1 byte of command data
     *      1 byte of address
     *      1 additional byte as we wait to receive data from slave device */
    flash_SetSPI_BufferSize(3);
    MOSI[0] = GET_FEAT[0];
    MOSI[1] = GET_FEAT[1];

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS), false);

    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* De-select device by pulling CS high. */
    gpio_set_pin_level(GPIO_PIN(CS), true);

    /* Reinitialize output buffer. */
    MOSI[0] = 0;
    MOSI[1] = 0;

    return (MISO[2]);
}

/* Issue the SET WEL BIT command to enable writing: 0x06
* Issue GET FEATURES command and register address: 0x0FC0 */
/* In order for the chip to recognize the SET WEL BIT command, 
* ONLY one byte of data can be sent over the SPI connection
* containing the command. The status is checked immediately
* after in a different SPI transaction. */
//returns status register value
//NON BLOCKING - IF DEVICE IS BUSY, COMMAND WILL NOT BE SENT
uint8_t flash_SetWEL()
{
    uint8_t status = flash_Status();

    if((status & BUSY_MASK) == 0)
    {
        /* Set SPI buffer to send only 1 byte of command data. 
         * Put the command in the output buffer. */
        flash_SetSPI_BufferSize(1);
        MOSI[0] = SET_WEL[0];	
	
        /* Set slave select (CS) active low to communicate. */
        gpio_set_pin_level(GPIO_PIN(CS), false);
    
        /* Read/write over SPI */
        spi_m_sync_transfer(&SPI_0, &spi_buff);
    
        /* De-select device by pulling CS high. */
        gpio_set_pin_level(GPIO_PIN(CS), true);

        /* Reinitialize output buffer. */
        MOSI[0] = SET_WEL[0];

        status = flash_Status();
    }

    return (status);
}

//returns status register value. col addr must be 2 bytes, page block 3 bytes
//BLOCKING - WAITS UNTIL CACHE IS DONE PROGRAMMING TO MOVE ON. 
uint8_t flash_WritePage(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[])
{   
    uint8_t status = flash_Status();

    /* If the device is not busy, attempt to program it. */
    if((status & BUSY_MASK) == 0)
    {
        status = ProgramCache(data, dataSize, colAddress);

        /* Wait until device is not busy. */

        /* Make sure the program failure flag is not set before executing the write. */
        if((status & PROG_FAIL) == 0)
        {
            status = ExecuteProgram(pageBlockAddress);
        }
    }

    return (status);
}

//sit in an empty loop until flash is no longer busy
//make sure this doesn't get optimized out? 
void flash_WaitUntilNotBusy()
{
    do { /* NOTHING */ } while ((flash_Status() & BUSY_MASK) != 0);
}

/* INTERNAL FUNCTIONS - NOT A PART OF API */
uint8_t ProgramCache(uint8_t data[], int dataSize, uint8_t colAddress[]);
uint8_t ExecuteProgram(uint8_t pageBlockAddress[]);