/*
 * NAND_Flash.c
 *
 * Created: 2/23/2018 6:34:06 PM
 *  Author: Krystine Carrington
 */ 

#include "NAND_Flash.h"

/* TODO:
 * 1) Update "SPI_0" component to be named something better, like "SPI_FLASH".
 * 2) Get updated config file for line ending stuff.
 * 3) CS pins will ultimately need unique names for each device connected to micro.
 */

/* CONSTANT DECLARATIONS */
const uint8_t RESET[1]          = {0xFF};                   //Command to reset the memory device
const uint8_t GET_FEAT[2]       = {0x0F, 0xC0};             //Command to get the current contents of the status register
const uint8_t SET_WEL[1]        = {0x06};                   //Command to set the write enable bit in in the status register
const uint8_t PROG_LOAD[3]      = {0x02, 0x00, 0x00};       //Command to load data from micro into cache of memory device. Last 2 bytes page column address
const uint8_t PEXEC[4]          = {0x10, 0x00, 0x00, 0x00}; //Command to program the main memory array from the memory devices cache register
const uint8_t PAGE_READ[4]      = {0x13, 0x00, 0x00, 0x10}; //Command to read data from main array into data cache
const uint8_t READ_CACHE[3]     = {0x03, 0x00, 0x00};       //Command to read data from memory device cache to SPI buffer
const uint8_t ERASE[1]          = {0xD8};                   //Command to erase a block of data 
const uint8_t GET_BLOCK_LOCK[2] = {0x0F, 0xA0};             //Command to check the block lock status
const uint8_t UNLOCK_BLOCKS[3]  = {0x1F, 0xA0, 0x00};       //Command to check the block lock status

/* MASKS */
const uint8_t BUSY_MASK         = 0x01;                     //Mask for checking if the flash is busy
const uint8_t PROG_FAIL         = 0b00001000;               //Mask for checking if the memory was programmed successfully
const uint8_t WEL_MASK          = 0x02;                     //Mask for checking if write enable is high

/*************************************************************
 * FUNCTION: flash_InitSPI()
 * -----------------------------------------------------------
 * This function initializes the SPI communication that will
 * be used to communicate with the memory device. It sets the
 * input and output buffers as well as the SPI buffer size. 
 * In addition, it sets the SPI mode to Mode 0 and enables
 * the SPI device. 
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
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

/*************************************************************
 * FUNCTION: flash_InitBuffers()
 * -----------------------------------------------------------
 * This function initializes the buffers that will be used for
 * SPI communication. The input buffer is initialized to all
 * 0xFF values. The output buffer, on the other hand, is 
 * initialized to all zeros. This is because 0xFF is 
 * recognized as a command by the NAND device and we don't 
 * want to inadvertently send the command. 0x00 is not 
 * recognized as a command by the NAND device.
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
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

/*************************************************************
 * FUNCTION: flash_BufferSize()
 * -----------------------------------------------------------
 * This function is a utility function used to set the size
 * of the SPI buffer.
 *
 * Parameters:
 *      newSize :   New size of SPI buffer
 *
 * Returns: void
 *************************************************************/
void flash_SetSPI_BufferSize(int newSize)
{
    spi_buff.size = newSize;
}

/*************************************************************
 * FUNCTION: flash_Reset()
 * -----------------------------------------------------------
 * This function issues the RESET command: 0xFF. The command 
 * is placed in the output buffer and then sent over the SPI
 * connection only if the device is not currently busy. If
 * the device is busy when this function is called, the 
 * current status of the device will be returned to the 
 * calling function without any further operations being 
 * performed. Device reset on startup is not mandatory for 
 * this part, but is recommended. This also ensures previous 
 * run settings are reset within the device during testing. 
 *
 * Parameters: none
 *
 * Returns: 
 *      status  :   Current status of the device
 *************************************************************/
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

/*************************************************************
 * FUNCTION: flash_Status()
 * -----------------------------------------------------------
 * Issue the GET FEATURES command: 0x0F 
 * Send address of FEATURES register: 0xC0
 * Put command in MOSI buffer.  
 *
 * Parameters: none
 *
 * Returns:
 *      status  :   Current status of the device
 *************************************************************/
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

/*************************************************************
 * FUNCTION: flash_SetWEL()
 * -----------------------------------------------------------
 * This function issues the Set Write Enable Latch command to
 * the memory device. The command is placed into the output 
 * buffer and sent to the device. The command must be sent as
 * a single byte otherwise the memory device will not 
 * recognize it. The command will only be issued if the device
 * is not busy. If it is busy, the current status will be 
 * returned to the calling function without any further 
 * operations occurring. 
 *
 * Parameters: none
 *
 * Returns:
 *      status  :   Current status of the device
 *************************************************************/
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

/*************************************************************
 * FUNCTION: flash_WritePage()
 * -----------------------------------------------------------
 * This function writes a page of data to the Flash device.
 * First, the write enable flag (WEL bit) is set. Next, the
 * device's cache is loaded with the given data. Once that
 * Operation is complete, the data is written from the
 * device's cache into its main memory array. The status of
 * the operation is returned to the calling program after the
 * operation concludes. If the device is currently busy when
 * this function is called, then the status is returned and
 * no further processing occurs.
 *
 *  NOTE: This function is blocking. The function will wait 
 *        until the cache is done being programmed before 
 *        issuing the execute command to move the data into 
 *        the main memory array. It is important to note that
 *        the maximum amount of time this delay may take is
 *        200us.
 *
 * Parameters:
 *      data[]              :   Data to write to flash. Max
 *                              size of 2176 bytes
 *      dataSize            :   Size of data to store. Max
 *                              size of 2176 bytes
 *      colAddress[]        :   Page offset. First 3 bits (most 
 *                              significant) are zeros. 
 *      pageBlockAddress[]  :   Block and Page to store data in
 *
 * Returns:
 *      status              :   Current status of the device
 *************************************************************/
uint8_t flash_WritePage(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[])
{   
    uint8_t status = flash_Status();

    /* If the device is not busy, attempt to program it. */
    if((status & BUSY_MASK) == 0)
    {
        status = flash_SetWEL();
        
        /* Make sure Write Enable flag was set. */
        if((status & WEL_MASK) != 0)
        {
            status = ProgramCache(data, dataSize, colAddress);

            /* Wait until device is not busy. */
            flash_WaitUntilNotBusy();

            /* Make sure the program failure flag is not set before executing the write. */
            if((status & PROG_FAIL) == 0)
            {
                status = ExecuteProgram(pageBlockAddress);
            }
        }            
    }

    return (status);
}

/*************************************************************
 * FUNCTION: flash_WaitUntilNotBusy()
 * -----------------------------------------------------------
 * This function continuously checks the status register. The
 * function returns once the status register shows that the 
 * device is no longer busy. 
 *
 *  NOTE: This function is blocking. The function will wait 
 *        until the status register shows that the device is
 *        no longer busy. 
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void flash_WaitUntilNotBusy()
{
    do { /* NOTHING */ } while ((flash_Status() & BUSY_MASK) != 0);
}

/*************************************************************
 * FUNCTION: flash_ReadPage()
 * -----------------------------------------------------------
 * This function reads a page of data from the Flash device.
 * First, a page of data is read from the main memory array of
 * the device into the data cache. Then, the data is
 * transfered from the cache to the input buffer of the micro.
 * The status of the operation is returned to the calling 
 * program after the operation concludes. If the device is 
 * currently busy when this function is called, then the 
 * status is returned and no further processing occurs.
 *
 *  NOTE: This function is blocking. The function will wait 
 *        until the data is done being loaded into the cache
 *        before it begins sending data along the MISO line.
 *        It is important to note that the maximum amount of 
 *        time this delay may take is 25us.
 *
 * Parameters: 
 *      colAddress[]        :   Page offset. First 3 bits (most 
 *                              significant) are zeros. 
 *      blockPageAddress[]  :   Block and Page to store data in
 *
 * Returns:
 *      status              :   Current status of the device
 *************************************************************/
uint8_t flash_ReadPage(uint8_t blockPageAddress[], uint8_t columnAddress[], uint8_t pageData[])
{
    uint8_t status = flash_Status();

    /* If the device is not busy, attempt to read a page. */
    if((status & BUSY_MASK) == 0)
    {
        status = PageRead(blockPageAddress);

        /* Wait until device is not busy. */
        flash_WaitUntilNotBusy();

        /* Start streaming data back from slave device. */
        status = ReadFromCache(columnAddress, pageData);
    }
    
    return (status);
}

/*************************************************************
 * FUNCTION: flash_ReadPage()
 * -----------------------------------------------------------
 * This function checks the busy flag in the status register. 
 * If the device is NOT busy, then the status will become
 * false, signifying that the device is not busy.
 *
 * Parameters: none
 *
 * Returns:
 *      status  :  True if the device is busy, false otherwise.
 *************************************************************/
bool flash_IsBusy()
{
    /* Initialize status to busy. */
    bool status = true;
    
    /* Checks the busy flag in the status register. If the device is
     * NOT busy, then the status will become false for not busy. */
    if((flash_Status() & BUSY_MASK) == 0)
    {
        status = false;
    }
    
    return (status);
}

/*************************************************************
 * FUNCTION: flash_BlockErase()
 * -----------------------------------------------------------
 * This function erases an entire block of memory from the 
 * flash device. A block is the minimum size unit that is able
 * to be erased within the device. Only a single block may be
 * erased at a time. A block consists of 64 pages or 136k 
 * bytes. Before a block is able to be erased, the write 
 * enable flag must be set. 
 *
 * Parameters: 
 *      blockAddress[]  :   Address of block to be erased.
 *
 * Returns:
 *      status          :   Current status of device.
 *************************************************************/
uint8_t flash_BlockErase(uint8_t blockAddress[])
{
     uint8_t status = flash_Status();

    /* If the device is not busy, attempt to program it. */
    if((status & BUSY_MASK) == 0)
    {
        status = flash_SetWEL();
        
        /* Make sure Write Enable flag was set. */
        if((status & WEL_MASK) != 0)
        {
           /* Set SPI buffer to send only 1 byte of command data and
            * 3 bytes for the block address. Put the command in the 
            * output buffer. */
            flash_SetSPI_BufferSize(4);
            MOSI[0] = ERASE[0];
            MOSI[1] = blockAddress[0];
            MOSI[2] = blockAddress[1];
            MOSI[3] = blockAddress[2];

            /* Set slave select (CS) active low to communicate. */
            gpio_set_pin_level(GPIO_PIN(CS), false);
    
            /* Read/write over SPI */
            spi_m_sync_transfer(&SPI_0, &spi_buff);
    
            /* De-select device by pulling CS high. */
            gpio_set_pin_level(GPIO_PIN(CS), true);
            
            /* Reinitialize output buffer and get status. */
            MOSI[0] = 0;
            MOSI[1] = 0;
            MOSI[2] = 0;
            MOSI[3] = 0;
            
            status = flash_Status();
        }            
    }

    return (status);
}

/*************************************************************
 * FUNCTION: flash_BlockLockStatus()
 * -----------------------------------------------------------
 * This function  
 *
 * Parameters: none
 *
 * Returns:
 *      status  :   Current status of block lock register.
 *************************************************************/
uint8_t flash_BlockLockStatus()
{
    /* Set buffer size to 3 and put command in output buffer:
     *      1 byte of command data
     *      1 byte of address
     *      1 additional byte as we wait to receive data from slave device */
    flash_SetSPI_BufferSize(3);
    MOSI[0] = GET_BLOCK_LOCK[0];
    MOSI[1] = GET_BLOCK_LOCK[1];

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

uint8_t flash_UnlockBlocks()
{
    /* Set buffer size to 3 and put command in output buffer:
     *      1 byte of command data
     *      1 byte of address
     *      1 additional byte as we wait to receive data from slave device */
    flash_SetSPI_BufferSize(3);
    MOSI[0] = UNLOCK_BLOCKS[0];
    MOSI[1] = UNLOCK_BLOCKS[1];
    MOSI[2] = UNLOCK_BLOCKS[2];

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS), false);

    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* De-select device by pulling CS high. */
    gpio_set_pin_level(GPIO_PIN(CS), true);

    /* Reinitialize output buffer. */
    MOSI[0] = 0;
    MOSI[1] = 0;
    MOSI[2] = 0;
    
    return (flash_BlockLockStatus());
}

/* INTERNAL FUNCTIONS - NOT A PART OF API */
uint8_t ProgramCache(uint8_t data[], int dataSize, uint8_t colAddress[])
{
    int i, j;

    /* Set SPI buffer to send 1 byte of command data, 2 address bytes, 
     * and a page of data. Put the command in the output buffer. 
     * The plus 3 for the size is to take into account the time it takes
     * to send the actual command. */
    flash_SetSPI_BufferSize(PAGE_SIZE + 3);
    MOSI[0] = PROG_LOAD[0];
    MOSI[1] = colAddress[0];
    MOSI[2] = colAddress[1];

    /* Fill up to an entire page of data. If less data is passed in,
     * a full page of data will still be sent, but the remainder of the
     * data will be all zeros. */
    j = 3;
    for(i = 0; (i < dataSize) && (i < PAGE_SIZE); i++)
    {
        MOSI[j] = data[i];
        j++;
    }

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS), false);
    
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS), true);

    /* De-select device by pulling CS high. */
    ReinitializeOutBuff();

    return (flash_Status());
}

uint8_t ExecuteProgram(uint8_t pageBlockAddress[])
{
    /* Next, write the contents of the cache register into the main
     * memory array. Pull chip select high as soon as address is done
     * transmitting. */
    flash_SetSPI_BufferSize(4);
    MOSI[0] = PEXEC[0];
    MOSI[1] = pageBlockAddress[0];
    MOSI[2] = pageBlockAddress[1];
    MOSI[3] = pageBlockAddress[2];
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS), false);
        
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
        
    /* De-select device by pulling CS high. */
    gpio_set_pin_level(GPIO_PIN(CS), true);

    /* Reinitialize output buffer. */
    MOSI[0] = 0;
    MOSI[1] = 0;
    MOSI[2] = 0;
    MOSI[3] = 0;

    return (flash_Status());
}

void ReinitializeOutBuff()
{
    int i;
    
    /* Initialize input and output buffers */
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        MOSI[i] = 0x00;
    }
}


/********************
* READ FROM MEMORY *
********************/ 
/* First, read a page from the main  memory array into the 
* data cache. */
//3 byte address. upper 7 bits low, next 11 bits block address, last 6 bits page address
uint8_t PageRead(uint8_t blockPageAddress[])
{
    /* Set SPI buffer to send 1 byte of command data and 3 bytes of address
     * data. Put the command in the output buffer. */
    flash_SetSPI_BufferSize(4);
    MOSI[0] = PAGE_READ[0];
    MOSI[1] = blockPageAddress[0];
    MOSI[2] = blockPageAddress[1];
    MOSI[3] = blockPageAddress[2];
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* De-select device by pulling CS high. */
    gpio_set_pin_level(GPIO_PIN(CS), true);
        
    /* Reinitialize output buffer and get status again */
    MOSI[0] = 0;
    MOSI[1] = 0;
    MOSI[2] = 0;
    MOSI[3] = 0;
    
    return (flash_Status());
}    
    
 //col address most likely zeros. offset where to start reading
 uint8_t ReadFromCache(uint8_t columnAddress[], uint8_t pageData[])
{
    int i, j; 
    
    /* Read the data from the cache register to the SPI
     * MISO line. */
    flash_SetSPI_BufferSize(PAGE_SIZE + 3);
    MOSI[0] = READ_CACHE[0];
    MOSI[1] = columnAddress[0];
    MOSI[2] = columnAddress[1];
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
        
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
        
    /* De-select device by pulling CS high. */
    gpio_set_pin_level(GPIO_PIN(CS), true);
        
    /* Reinitialize output buffer and get status again */
    MOSI[0] = 0;
    MOSI[1] = 0;
    MOSI[2] = 0;
    
    j = 3;
    for(i = 0; i < PAGE_SIZE; i++)
    {
        pageData[i] = MISO[j];
        j++;
    }        
        
    return (flash_Status());
}