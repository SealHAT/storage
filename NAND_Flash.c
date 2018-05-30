/*
 * NAND_Flash.c
 *
 * Created: 2/23/2018 6:34:06 PM
 *  Author: Krystine Carrington
 */ 

#include "NAND_Flash.h"

/* TODO:
 * 3) CS pins will ultimately need unique names for each device connected to micro.
 * 4) Validate dataSize and colAddress when writing.
 */

/* CONSTANT DECLARATIONS */
static uint8_t RESET[1]               = {0xFF};                   /* Command to reset the memory device */
static uint8_t GET_FEAT[2]            = {0x0F, 0xC0};             /* Command to get the current contents of the status register */
static uint8_t SET_WEL[1]             = {0x06};                   /* Command to set the write enable bit in in the status register */
static uint8_t PROG_LOAD[3]           = {0x02, 0x00, 0x00};       /* Command to load data from micro into cache of memory device. Last 2 bytes page column address */
static uint8_t PEXEC[4]               = {0x10, 0x00, 0x00, 0x00}; /* Command to program the main memory array from the memory devices cache register */
static uint8_t PAGE_READ[4]           = {0x13, 0x00, 0x00, 0x10}; /* Command to read data from main array into data cache */
static uint8_t READ_CACHE[3]          = {0x03, 0x00, 0x00};       /* Command to read data from memory device cache to SPI buffer */
static uint8_t ERASE[1]               = {0xD8};                   /* Command to erase a block of data  */
static uint8_t GET_BLOCK_LOCK[2]      = {0x0F, 0xA0};             /* Command to check the block lock status */
static uint8_t UNLOCK_BLOCKS[3]       = {0x1F, 0xA0, 0x00};       /* Command to check the block lock status */

/* MASKS */
uint8_t BUSY_MASK                     = 0x01;                     /* Mask for checking if the flash is busy */
uint8_t PROG_FAIL                     = 0b00001000;               /* Mask for checking if the memory was programmed successfully */
uint8_t WEL_MASK                      = 0x02;                     /* Mask for checking if write enable is high */
char SIGNATURE[SIGNATURE_SIZE] = "SealHAT!";

uint32_t badBlockTable[MAX_BAD_BLOCKS];

static uint8_t buf[PAGE_SIZE_EXTRA];                             /* Holds a page worth (PAGE_SIZE bytes) of data. */

/* SPI COMMUNICATION BUFFERS */
uint8_t seal_flash_SPI_buf[NAND_BUFFER_SIZE];
//uint8_t flash_MOSI[NAND_BUFFER_SIZE];                            /* Master's output buffer */
//uint8_t flash_MISO[NAND_BUFFER_SIZE];                            /* Master's input buffer */
struct  spi_xfer spi_flash_buff;                                 /* SPI transfer descriptor */
uint8_t activeFlashChip;                                         /* Current active flash chip. Currently supports up to three chips, with the value 00 being all chips deselected. */

SUPERBLOCK_t superblock;

/*************************************************************
 * FUNCTION: seal_flash_init()
 * -----------------------------------------------------------
 * This function calls the other flash initialization 
 * functions to initialize SPI buffers, SPI device, and 
 * initialize the bad block table. 
 *
 * Parameters:
 *    buf : Buffer for holding a page of data.
 *              -> Must be of size PAGE_SIZE_EXTRA
 *
 * Returns: void
 *************************************************************/
void seal_flash_init()
{
    uint8_t status;
    
    /* Initialize SPI and SPI buffers. */
    seal_flash_initSPI();
    seal_flash_init_buffers();
    seal_flash_init_BBT();
    
    /* Reset slave device (memory chip). */ 
    status = seal_flash_reset();     //status should be 0x01 if busy and 0x00 when done
    
    /* Minimum 1.25ms delay after reset command before any
     * other commands can be issued. Rounded up to 2ms. */
    delay_ms(2);
    
    /* Unlock all blocks (locked by default at power up). Returns status of the
     * block lock register. If the blocks are already unlocked, the unlock 
     * command will not be sent. */
    status = seal_flash_block_lock_status();
    
    if(status > 0)
    {
        status = seal_flash_unlock_all_blocks();
    } 

    /* Read superblock data. If superblock does not exist on device, create one. */
    seal_flash_read_superblock(buf);
}

/*************************************************************
 * FUNCTION: seal_flash_read_superblock()
 * -----------------------------------------------------------
 * This function reads the first page of the device for data.
 * The data stored here should be a struct of superblock
 * values, as defined in the SUPERBLOCK struct in the
 * NAND_Flash.h file. This function will first check for the 
 * signature. If the signature is found, then the bad block
 * table will be created from the data stored here. If the 
 * signature is not found, a bad block table will be created
 * from scratch (this may take up to 15 minutes). If a new
 * superblock is created, the signature will be the last 
 * parameter updated. This ensures that if the signature is
 * present, then so is the bad block table. The bad block
 * table size will be stored in the superblock as well.
 *
 * Parameters:
 *      buf : Buffer for holding a page of data.
 *              -> Must be of size PAGE_SIZE_EXTRA
 *
 * Returns: void
 *************************************************************/
void seal_flash_read_superblock()
{
    uint32_t address;                   /* Block and page offset */
    uint32_t colAddress;                /* Column offset within a page */
    bool     valid;                     /* Status of the superblock data */
    int      i;
    int      j;
    
    /* INITIALIZATIONS - start at block zero, column zero. */
    address     = 0x000000;
    colAddress  = 0x0000;

    /* Read first page of memory to check for superblock. */
    seal_flash_read_page((uint8_t *) &address, (uint8_t *) &colAddress, buf);
    
    /* Read the data into a struct for further processing. */
    seal_init_cache_superblock(buf, PAGE_SIZE_EXTRA);
    
    /* Call the validate function to ensure the superblock data is valid. If it
     * is not valid, a new superblock will be created and written to the flash. */
    valid = seal_validate_superblock();
    
    /* If superblock does not exist, create one. Keep copy on micro, but write 
     * back immediately to flash as well. */
    if(valid == false)
    {
        /* Erase the first page of the device since it was invalid. A new page will
         * be written at the bottom of this function. */
        seal_flash_block_erase((uint8_t *) &address);
        
        seal_flash_wait_until_not_busy();
        
        /* Calls the bad block table builder. This will iterate through the entire
         * device and determine which blocks should not be used. This bad block 
         * table will be stored on the first page of the first block of the device. */
        buf[8] = seal_build_bad_block_table();

        /* The bad block table generates a table of 32-bit addresses for 
         * blocks that should not be written to or read from. These values
         * need to be serialized so they may be sent over the SPI connection.
         * This loop breaks a 32-bit number into four consecutive bytes. 
         * The value of page[8] contains the size of the bad block table. */
        i = 9;
        j = 0;
        while(j < buf[8])
        {
            buf[i]   = (uint8_t) (badBlockTable[j] >> 24);
            buf[++i] = (uint8_t) (badBlockTable[j] >> 16);
            buf[++i] = (uint8_t) (badBlockTable[j] >> 8);
            buf[++i] = (uint8_t) (badBlockTable[j]);
            i++;
            j++;
        }
            
        /* The remainder of the page data (except the spare area) is filled
         * with zeros. */
        while(i < PAGE_SIZE_LESS)
        {
            buf[i] = 0;
            i++;
        }
        
        /* The signature is the last thing written. This ensures all other data was 
         * loaded if this value is valid. */
        for(i = 0; i < SIGNATURE_SIZE; i++)
        {
            buf[i] = SIGNATURE[i];
        }
        
        /* Keep a copy of the superblock data on the micro during runtime. Also write the
         * data back to the device immediately to ensure a copy gets preserved. */
        seal_init_cache_superblock(buf, PAGE_SIZE_EXTRA);
        seal_flash_wait_until_not_busy();
        seal_flash_write_page(buf, PAGE_SIZE_EXTRA, (uint8_t *) &colAddress, (uint8_t*) &address);
        seal_flash_wait_until_not_busy();
    }
}

/*************************************************************
 * FUNCTION: seal_validate_superblock()
 * -----------------------------------------------------------
 * This function takes the superblock data and validates it.
 * The device signature is checked first to ensure a valid 
 * signature exists on the device. If no signature is found,
 * or if the signature is incorrect, then the superblock is
 * considered invalid. The superblock is also considered 
 * invalid when the stored bad block count is greater than
 * the calculated maximum bad block value. 
 *
 * Parameters: none
 *
 * Returns:
 *      valid   :   Status of the superblock data.
 *************************************************************/
bool seal_validate_superblock()
{
    bool valid;         /* Keeps track of if superblock params are valid */
    int  i;
    
    /* INITIALIZATIONS */
    valid = true;
    
    /* Iterate through 8 bytes of data checking for a valid device signature. */
    i = 0;
    while(i < SIGNATURE_SIZE && valid)
    {
        if(superblock.signature[i] != SIGNATURE[i])
        {
            valid = false;
        }
        i++;
    }
    
    /* The manufacturer guarantees that only a certain number of blocks will
     * go bad within the lifetime of the device. If the value of this is 
     * outside of the MAX_BAD_BLOCKS calculation, then there was either an 
     * error when determining bad blocks or the device is bad and should
     * not be used. */
    if(superblock.badBlockCount > MAX_BAD_BLOCKS)
    {
        valid = false;
    }
    
    return (valid);
}

/*************************************************************
 * FUNCTION: seal_init_cache_superblock()
 * -----------------------------------------------------------
 * This function takes the superblock data and stores it in a
 * struct that may be accessed during runtime. This function
 * does not check to see if the data is valid. That check is
 * done in the validate_superblock function. 
 *
 * Parameters:
 *      page[]      :   Contains a page of data. 
 *      pageSize    :   Size of the page data.
 *
 * Returns: void
 *************************************************************/
void seal_init_cache_superblock(uint8_t page[], int pageSize)
{
    /* Superblock is global in this file. */
    int i;          /* Current page index. */
    int j;          /* Bad block table index. */
    (void) pageSize;
    
    /* The first 8 bytes of data are the signature. */
    for(i = 0; i < SIGNATURE_SIZE; i++)
    {
        superblock.signature[i] = page[i];
    }
    
    /* Next index. */
    i = SIGNATURE_SIZE;
    
    /* The next byte is the number of bad blocks currently on the device */
    superblock.badBlockCount = page[i];
    i++;
    
    /* The next number of bytes specified by the badBlockCount attribute 
     * contain the addresses of the known bad blocks. */
    j = 0;
    while((j < superblock.badBlockCount) && (j < MAX_BAD_BLOCKS))
    {
        /* Each address is 32 bits, but each array index is only one 
         * byte long. Four values are read in and bit shifted to create
         * a single 32-bit value. MSB -> LSB */
        superblock.badBlockTable[j] = (((uint32_t) page[i]) << 24) | (((uint32_t) page[i+1]) << 16) | 
                                      (((uint32_t) page[i+2]) << 8) | ((uint32_t) page[i+4]);
        i += 4;
        j++;
    }
    
    /* The bad block pointer just holds the array index of the next bad block
     * in the bad block table. */
    superblock.badBlockIndex = 0;
}

/*************************************************************
 * FUNCTION: seal_flash_initSPI()
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
void seal_flash_initSPI()
{
	/* Associate flash buffers with SPI device and set
     * buffer size. */
    spi_flash_buff.size  = NAND_BUFFER_SIZE;
    spi_flash_buff.rxbuf = seal_flash_SPI_buf;
    spi_flash_buff.txbuf = seal_flash_SPI_buf;

    /* Setup SPI IO */
	/* Set clock mode and enable SPI */
	spi_m_sync_set_mode(&SPI_MEMORY, SPI_MODE_0);
	spi_m_sync_enable(&SPI_MEMORY);
}

/*************************************************************
 * FUNCTION: seal_flash_init_buffers()
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
void seal_flash_init_buffers()
{
	int i; 
	
	/* Initialize input and output buffers */
	for(i = 0; i < NAND_BUFFER_SIZE; i++)
	{
		seal_flash_SPI_buf[i] = 0xFF;
		seal_flash_SPI_buf[i] = 0x00;
	}
}

/*************************************************************
 * FUNCTION: seal_flash_init_BBT()
 * -----------------------------------------------------------
 * This function initializes the bad block table that will
 * hold addresses of known bad blocks within the device.
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void seal_flash_init_BBT()
{
    int i;
    
    for(i = 0; i < MAX_BAD_BLOCKS; i++)
    {
        badBlockTable[i] = 0;
    }
}

/*************************************************************
 * FUNCTION: seal_flash_setSPI_buffer_size()
 * -----------------------------------------------------------
 * This function is a utility function used to set the size
 * of the SPI buffer.
 *
 * Parameters:
 *      newSize :   New size of SPI buffer
 *
 * Returns: void
 *************************************************************/
void seal_flash_setSPI_buffer_size(int newSize)
{
    spi_flash_buff.size = newSize;
}

/*************************************************************
 * FUNCTION: seal_flash_reset()
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
uint8_t seal_flash_reset()
{
    /* Get the current status of the device. */
    uint8_t status = seal_flash_status();

    /* Only reset device if the device is not currently busy. */
    if((status & BUSY_MASK) == 0)
    {
        /* Set SPI buffer to send only 1 byte of command data. 
         * Put the command in the output buffer. */
        seal_flash_setSPI_buffer_size(1);
        seal_flash_SPI_buf[0] = RESET[0];

        /* Complete an SPI transaction */
        seal_flash_spi_transaction();

        /* Reinitialize output buffer. */
        seal_flash_SPI_buf[0] = 0;

        /* Get updated status. */
        status = seal_flash_status();
    }

    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_status()
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
uint8_t seal_flash_status()
{
    /* Set buffer size to 3 and put command in output buffer:
     *      1 byte of command data
     *      1 byte of address
     *      1 additional byte as we wait to receive data from slave device */
    seal_flash_setSPI_buffer_size(3);
    seal_flash_SPI_buf[0] = GET_FEAT[0];
    seal_flash_SPI_buf[1] = GET_FEAT[1];

    /* Complete an SPI transaction */
    seal_flash_spi_transaction();

    /* Reinitialize output buffer. */
    seal_flash_SPI_buf[0] = 0;
    seal_flash_SPI_buf[1] = 0;

    return (seal_flash_SPI_buf[2]);
}

/*************************************************************
 * FUNCTION: seal_flash_set_WEL()
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
uint8_t seal_flash_set_WEL()
{
    uint8_t status = seal_flash_status();

    if((status & BUSY_MASK) == 0)
    {
        /* Set SPI buffer to send only 1 byte of command data. 
         * Put the command in the output buffer. */
        seal_flash_setSPI_buffer_size(1);
        seal_flash_SPI_buf[0] = SET_WEL[0];	
	
        /* Complete an SPI transaction */
        seal_flash_spi_transaction();

        /* Reinitialize output buffer. */
        seal_flash_SPI_buf[0] = SET_WEL[0];

        status = seal_flash_status();
    }

    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_write_page()
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
uint8_t seal_flash_write_page(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[])
{   
    uint8_t status = seal_flash_status();

    /* If the device is not busy, attempt to program it. */
    if((status & BUSY_MASK) == 0)
    {
        status = seal_flash_set_WEL();
        
        /* Make sure Write Enable flag was set. */
        if((status & WEL_MASK) != 0)
        {
            status = seal_program_load(data, dataSize, colAddress);

            /* Wait until device is not busy. */
            seal_flash_wait_until_not_busy();

            /* Make sure the program failure flag is not set before executing the write. */
            if((status & PROG_FAIL) == 0)
            {
                status = seal_execute_program(pageBlockAddress);
                
                /* Wait until device is not busy. */
                seal_flash_wait_until_not_busy();
            }
        }            
    }

    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_wait_until_not_busy()
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
void seal_flash_wait_until_not_busy()
{
    do { /* NOTHING */ } while ((seal_flash_status() & BUSY_MASK) != 0);
}

/*************************************************************
 * FUNCTION: seal_flash_read_page()
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
uint8_t seal_flash_read_page(uint8_t blockPageAddress[], uint8_t columnAddress[], uint8_t pageData[])
{
    uint8_t status = seal_flash_status();

    /* If the device is not busy, attempt to read a page. */
    if((status & BUSY_MASK) == 0)
    {
        status = seal_page_read(blockPageAddress);

        /* Wait until device is not busy. */
        seal_flash_wait_until_not_busy();

        /* Start streaming data back from slave device. */
        status = seal_read_from_cache(columnAddress, pageData);
    }
    
    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_read()
 * -----------------------------------------------------------
 * This function calls the flash_read_page function to read a 
 * page of data. This occurs only after the block address has
 * been adjusted to account for the bad blocks that were 
 * skipped. This occurs by calling the calculate_block_offset
 * function before the flash_read_page function is called. 
 *
 * Parameters: 
 *      blockAddress    :   Given block address before offset
 *      columnAddress   :   Where to start reading a page
 *      dataBuffer[]    :   Holds the data that is read
 *      dataSize        :   Size of data to read
 *
 * Returns:
 *      status          :   Current status of the device
 *************************************************************/
uint8_t seal_flash_read(uint32_t blockAddress, uint32_t columnAddress, uint8_t dataBuffer[], int dataSize)
{
    uint32_t offsetBlockAddress;    /* Corrected address of the page we should read from. */
    uint8_t  status;                /* Value of the status register. */
    
    /* Shift the block address to account for bad blocks. */
    offsetBlockAddress = seal_calculate_block_offset(blockAddress);
    (void)dataSize;
    
    /* Call page read with the updated block address. */
    status = seal_flash_read_page((uint8_t *) &offsetBlockAddress, (uint8_t *) &columnAddress, dataBuffer);
    
    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_write()
 * -----------------------------------------------------------
 * This function calls the flash_write_page function to write
 * a page of data. This occurs only after the block address
 * has  been adjusted to account for the bad blocks that were
 * skipped. This occurs by calling the calculate_block_offset
 * function before the flash_write_page function is called.
 *
 * Parameters: 
 *      blockAddress    :   Given block address before offset
 *      columnAddress   :   Where to start reading a page
 *      dataBuffer[]    :   Holds the data that is read
 *      dataSize        :   Size of data to read
 *
 * Returns:
 *      status          :   Current status of the device
 *************************************************************/
uint8_t seal_flash_write(uint32_t blockAddress, uint32_t columnAddress, uint8_t dataBuffer[], int dataSize)
{
    uint32_t offsetBlockAddress;    /* Corrected address of the page we should read from. */
    uint8_t  status;                /* Value of the status register. */
        
    /* Shift the block address to account for bad blocks. */
    offsetBlockAddress = seal_calculate_block_offset(blockAddress);
    (void)dataSize;
    
    status = seal_flash_write_page(dataBuffer, dataSize, (uint8_t *) &columnAddress, (uint8_t *) &offsetBlockAddress);
   
   return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_is_busy()
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
bool seal_flash_is_busy()
{
    /* Initialize status to busy. */
    bool status = true;
    
    /* Checks the busy flag in the status register. If the device is
     * NOT busy, then the status will become false for not busy. */
    if((seal_flash_status() & BUSY_MASK) == 0)
    {
        status = false;
    }
    
    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_block_erase()
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
uint8_t seal_flash_block_erase(uint8_t blockAddress[])
{
    volatile uint8_t status = seal_flash_status();

    /* If the device is not busy, attempt to program it. */
    if((status & BUSY_MASK) == 0)
    {
        status = seal_flash_set_WEL();
        
        /* Make sure Write Enable flag was set. */
        if((status & WEL_MASK) != 0)
        {
           /* Set SPI buffer to send only 1 byte of command data and
            * 3 bytes for the block address. Put the command in the 
            * output buffer. */
            seal_flash_setSPI_buffer_size(4);
            seal_flash_SPI_buf[0] = ERASE[0];
            seal_flash_SPI_buf[1] = blockAddress[2];
            seal_flash_SPI_buf[2] = blockAddress[1];
            seal_flash_SPI_buf[3] = blockAddress[0];

            /* Complete an SPI transaction */
            seal_flash_spi_transaction();
            
            /* Reinitialize output buffer and get status. */
            seal_flash_SPI_buf[0] = 0;
            seal_flash_SPI_buf[1] = 0;
            seal_flash_SPI_buf[2] = 0;
            seal_flash_SPI_buf[3] = 0;
            
            status = seal_flash_status();
        }            
    }

    return (status);
}

/*************************************************************
 * FUNCTION: seal_flash_erase_device()
 * -----------------------------------------------------------
 * This function erases the entire flash device except for the
 * first block of data. 
 *
 *  NOTE: This function is blocking!
 *
 * Parameters: none
 *
 * Returns:
 *      status          :   Current status of device.
 *************************************************************/
uint8_t seal_flash_erase_device()
{
    /* VARIABLE DECLARATIONS */
    int i;                          /* Loop control variable */
    uint32_t address;               /* The micro stores values little endian but is configured to send SPI data big endian */
    uint32_t nextBlock;             /* Amount to increase address by to get to next block */
    uint8_t  status;                /* Value of the status register */
        
    /* INITIALIZATIONS */
    nextBlock   = 0x40;
    address     = 0x000040;
        
    /* Iterate through all blocks in both planes */
    for(i = 1; i < NUM_BLOCKS; i++)
    {
        /* Only a single block is able to be erased at any given time. */
        //tempAddress = LitToBigEndian(address);
        status = seal_flash_block_erase((uint8_t *) &address);
        
        /* Wait until device is not busy. */
        seal_flash_wait_until_not_busy();
        
        /* Go to next block. */
        address += nextBlock;
    }     
    
    return (status);   
}

/*************************************************************
 * FUNCTION: seal_flash_block_lock_status()
 * -----------------------------------------------------------
 * This function gets the status of the block lock register.
 * The block lock register lists the blocks of the flash 
 * device which are currently locked. Check the data sheet
 * to determine which blocks are currently locked based on
 * the return value.
 *
 * Parameters: none
 *
 * Returns:
 *      status  :   Current status of block lock register.
 *************************************************************/
uint8_t seal_flash_block_lock_status()
{
    /* Set buffer size to 3 and put command in output buffer:
     *      1 byte of command data
     *      1 byte of address
     *      1 additional byte as we wait to receive data from slave device */
    seal_flash_setSPI_buffer_size(3);
    seal_flash_SPI_buf[0] = GET_BLOCK_LOCK[0];
    seal_flash_SPI_buf[1] = GET_BLOCK_LOCK[1];

    /* Complete an SPI transaction */
    seal_flash_spi_transaction();

    /* Reinitialize output buffer. */
    seal_flash_SPI_buf[0] = 0;
    seal_flash_SPI_buf[1] = 0;

    return (seal_flash_SPI_buf[2]);    
}

/*************************************************************
 * FUNCTION: seal_flash_unlock_all_blocks()
 * -----------------------------------------------------------
 * This function unlocks all blocks within the flash device
 * for reading and writing. 
 *
 *  NOTE: Take special care when using this function not to 
 *        overwrite critical device data or bad block marks.
 *
 * Parameters: none
 *
 * Returns:
 *      status  :   Current status of block lock register.
 *************************************************************/
uint8_t seal_flash_unlock_all_blocks()
{	
    /* Set buffer size to 3 and put command in output buffer:
     *      1 byte of command data
     *      1 byte of address
     *      1 additional byte as we wait to receive data from slave device */
    seal_flash_setSPI_buffer_size(3);
    seal_flash_SPI_buf[0] = UNLOCK_BLOCKS[0];
    seal_flash_SPI_buf[1] = UNLOCK_BLOCKS[1];
    seal_flash_SPI_buf[2] = UNLOCK_BLOCKS[2];

    /* Complete an SPI transaction */
    seal_flash_spi_transaction();

    /* Reinitialize output buffer. */
    seal_flash_SPI_buf[0] = 0;
    seal_flash_SPI_buf[1] = 0;
    seal_flash_SPI_buf[2] = 0;
    
    return (seal_flash_block_lock_status());
}

/* INTERNAL FUNCTIONS - NOT A PART OF API */
/*************************************************************
 * FUNCTION: seal_program_load()
 * -----------------------------------------------------------
 * This function loads data from the host device into the 
 * memory device's data cache. Only a maximum of PAGE_SIZE
 * bytes will be sent to be loaded into the cache. If the user
 * data array is less than PAGE_SIZE, then the last 
 * (PAGE_SIZE - dataSize) bytes will be all zeros. The write
 * enable flag must be set before this command is called.
 *
 *  TODO: validate dataSize and colAddress.
 *
 * Parameters:
 *      data[]      :   User data to store.
 *      dataSize    :   Size of data to store <= PAGE_SIZE.
 *      colAddress  :   Where on the page to store the data.
 *                      Ranges [0, (PAGE_SIZE - 1)].
 *
 * Returns:
 *      status      :   Current value of the status register.
 *************************************************************/
uint8_t seal_program_load(uint8_t data[], int dataSize, uint8_t colAddress[])
{
    int i;  /* Used for indexing the data array */
    int j;  /* Used for indexing the SPI buffer */

    /* Set SPI buffer to send 1 byte of command data, 2 address bytes, 
     * and a page of data. Put the command in the output buffer. 
     * The plus 3 for the size is to take into account the time it takes
     * to send the actual command. */
    if(dataSize > PAGE_SIZE_LESS)
    {
        dataSize = PAGE_SIZE_LESS;
    }
    
    seal_flash_setSPI_buffer_size(dataSize + 3);
    seal_flash_SPI_buf[0] = PROG_LOAD[0];
    seal_flash_SPI_buf[1] = colAddress[0];
    seal_flash_SPI_buf[2] = colAddress[1];

    /* Fill up to an entire page of data. If less data is passed in,
     * a full page of data will still be sent, but the remainder of the
     * data will be all zeros. WILL NOT OVERWRITE EXTRA SPACE. */
    j = 3;
    for(i = 0; (i < dataSize) && (i < PAGE_SIZE_LESS); i++)
    {
        seal_flash_SPI_buf[j] = data[i];
        j++;
    }

    /* Complete an SPI transaction */
    seal_flash_spi_transaction();

    /* De-select device by pulling CS high. */
    seal_reinitialize_out_buff();

    return (seal_flash_status());
}

/*************************************************************
 * FUNCTION: seal_execute_program()
 * -----------------------------------------------------------
 * This function takes data from the memory device's data
 * cache and moves them into the main memory array. This 
 * operation can cause the device to be busy for a maximum of
 * 200us. 
 *
 *  TODO: validate block address.
 *
 * Parameters:
 *      blockAddress    :   Block address in main memory array.
 *
 * Returns:
 *      status          :   Value of the status register.
 *************************************************************/
uint8_t seal_execute_program(uint8_t blockAddress[])
{
    /* Write the contents of the cache register into the main
     * memory array. Pull chip select high as soon as address is done
     * transmitting. */
    seal_flash_setSPI_buffer_size(4);
    seal_flash_SPI_buf[0] = PEXEC[0];
    seal_flash_SPI_buf[1] = blockAddress[2];
    seal_flash_SPI_buf[2] = blockAddress[1];
    seal_flash_SPI_buf[3] = blockAddress[0];
    
    /* Complete an SPI transaction */
    seal_flash_spi_transaction();

    /* Reinitialize output buffer. */
    seal_flash_SPI_buf[0] = 0;
    seal_flash_SPI_buf[1] = 0;
    seal_flash_SPI_buf[2] = 0;
    seal_flash_SPI_buf[3] = 0;

    return (seal_flash_status());
}

/*************************************************************
 * FUNCTION: seal_reinitialize_out_buff()
 * -----------------------------------------------------------
 * This function reinitializes the output buffer used in the
 * SPI transactions.
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void seal_reinitialize_out_buff()
{
    int i;
    
    /* Initialize input and output buffers */
    for(i = 0; i < NAND_BUFFER_SIZE; i++)
    {
        seal_flash_SPI_buf[i] = 0x00;
    }
}

/*************************************************************
 * FUNCTION: seal_page_read()
 * -----------------------------------------------------------
 * This function reads a page of data from the memory device's
 * main memory array and puts it into the data cache. The
 * upper 7 bits of the address parameter are low. The next 11 
 * bits are the block address, and the last 6 bits are the 
 * page address.
 *
 * Parameters:
 *      blockPageAddress[]  :   Three byte block/page address.
 *
 * Returns:
 *      status              :   Value of the status register.
 *************************************************************/
uint8_t seal_page_read(uint8_t blockPageAddress[])
{
    /* Set SPI buffer to send 1 byte of command data and 3 bytes of address
     * data. Put the command in the output buffer. */
    seal_flash_setSPI_buffer_size(4);
    seal_flash_SPI_buf[0] = PAGE_READ[0];
    seal_flash_SPI_buf[1] = blockPageAddress[2];
    seal_flash_SPI_buf[2] = blockPageAddress[1];
    seal_flash_SPI_buf[3] = blockPageAddress[0];
    
    /* Complete an SPI transaction */
    seal_flash_spi_transaction();
        
    /* Reinitialize output buffer and get status again */
    seal_flash_SPI_buf[0] = 0;
    seal_flash_SPI_buf[1] = 0;
    seal_flash_SPI_buf[2] = 0;
    seal_flash_SPI_buf[3] = 0;
    
    return (seal_flash_status());
}    

/**************************************************************
 * FUNCTION: seal_read_from_cache()
 * ------------------------------------------------------------
 * This function reads a page of data from the memory device's
 * data cache and sends it to the host device's input buffer
 * via an SPI transaction. 
 *
 * Parameters:
 *      columnAddress[] :   Where on the page to start reading.
 *      pageData[]      :   Data array to store returned data.
 *
 * Returns:
 *      status              :   Value of the status register.
 *************************************************************/
 uint8_t seal_read_from_cache(uint8_t columnAddress[], uint8_t pageData[])
{
    int i;  /* Used to index the return data array. */
    int j;  /* Used to index the SPI input buffer */
    
    /* Read the data from the cache register to the SPI
     * MISO line. */
    seal_flash_setSPI_buffer_size(PAGE_SIZE_EXTRA + 3);
    seal_flash_SPI_buf[0] = READ_CACHE[0];
    seal_flash_SPI_buf[1] = columnAddress[0];
    seal_flash_SPI_buf[2] = columnAddress[1];
    
    /* Complete an SPI transaction */
    seal_flash_spi_transaction();
        
    /* Reinitialize output buffer and get status again */
    seal_flash_SPI_buf[0] = 0;
    seal_flash_SPI_buf[1] = 0;
    seal_flash_SPI_buf[2] = 0;
    
    j = 4;
    for(i = 0; i < PAGE_SIZE_EXTRA; i++)
    {
        pageData[i] = seal_flash_SPI_buf[j];
        j++;
    }        
        
    return (seal_flash_status());
}

/**************************************************************
 * FUNCTION: seal_build_bad_block_table()
 * ------------------------------------------------------------
 * This function creates a list of bad blocks within the 
 * device. It just keeps track of the addresses that should 
 * not be written to when performing write operations. These
 * addresses will also be skipped when reading data from the
 * device. 
 *
 * Parameters: none
 *
 * Returns:
 *      badCount : Number of bad blocks found in the device.
 *************************************************************/
uint8_t seal_build_bad_block_table(uint8_t *buf)
{
    /* VARIABLE DECLARATIONS */
    int i;                          /* Loop control variable */
    uint32_t address;               /* The micro stores values little endian but is configured to send SPI data big endian */
    uint8_t  badCount;              /* Total number of bad blocks found */
    uint32_t nextBlock;             /* Amount to increase address by to get to next block */
    uint32_t colAddress;            /* Column address for in-page offset */
    uint8_t  tableIndex;            /* The current index of the bad block table */
    
    /* INITIALIZATIONS */
    badCount    = 0;
    nextBlock   = 0x40;
    address     = 0x000000;
    colAddress  = 0x0000;
    tableIndex  = 0;
    
    /* Iterate through all blocks in both planes */
    for(i = 0; i < NUM_BLOCKS; i++)
    {
        /* Read the first page of each block. Casts the integer value of the address to an
         * "array" for the Read function. */
        seal_flash_read_page((uint8_t *) &address, (uint8_t *) &colAddress, buf);
        
        /* If the first address in the spare area of the first page of a block is not 0xFF,
         * that means it was marked as a bad block and should not be used. */
        if(buf[BAD_BLK_ADDR] != 0xFF)
        {
            if(tableIndex < MAX_BAD_BLOCKS)
            {
                badBlockTable[tableIndex] = LitToBigEndian(address);
            }
            
            badCount++;
            tableIndex++;
        }
        
        /* Go to next address and make big endian */
        address += nextBlock;
    }

    return (badCount);
}

/**************************************************************
 * FUNCTION: seal_flash_get_superblock()
 * ------------------------------------------------------------
 * This function returns the address of the superblock. Only 
 * for testing.
 *
 * Parameters: none
 *
 * Returns:
 *      SUPERBLOCK * : Pointer to the superblock data.
 *************************************************************/
SUPERBLOCK_t *seal_flash_get_superblock()
{
    return (&superblock);
}

/*************************************************************
 * FUNCTION: seal_calculate_block_offset()
 * -----------------------------------------------------------
 * This function calculates what the "correct" address should
 * be for a given block address based on the bad block table.
 * Bad blocks in the system will be skipped, and thus an 
 * offset must occur.
 *
 * Parameters: 
 *      startingBlockAddress    :   Given address
 *
 * Returns:
 *      returnBlockAddress      :   Address after the offset
 *************************************************************/
uint32_t seal_calculate_block_offset(uint32_t startingBlockAddress)
{
    uint32_t blockOffsetSize;             /* Amount to increase address by to get to next block. */
    uint32_t returnBlockAddress;    /* Block address plus appropriate offset. */
    
    /* Initializations */
    blockOffsetSize = 0x40;
    superblock.badBlockIndex = 0;
    
    /* This loop figures out how many blocks this read should be shifted in order to get the data
     * at the "correct" address. It also makes sure no collisions will occur with the new address. */
    while((superblock.badBlockIndex < superblock.badBlockCount) && 
         ((startingBlockAddress + (superblock.badBlockIndex*blockOffsetSize)) >= superblock.badBlockTable[superblock.badBlockIndex]))
    {
        superblock.badBlockIndex++;
    }
    
    /* Calculate the appropriate offset and return. */
    if((superblock.badBlockCount > 0) && (superblock.badBlockIndex == superblock.badBlockCount))
    {
        returnBlockAddress = startingBlockAddress + ((superblock.badBlockCount + 1)*blockOffsetSize);
    }
    else
    {
        returnBlockAddress = startingBlockAddress + (superblock.badBlockIndex*blockOffsetSize);
    }
    
    return (returnBlockAddress);
}

/*************************************************************
 * FUNCTION: seal_flash_spi_transaction()
 * -----------------------------------------------------------
 * This function completes and SPI transaction with the SPI
 * descriptor currently associated with the flash chip(s). The
 * process begins by pulling the chip select line low since 
 * this signal is active low. The transaction continues by 
 * sending whatever data is in the SPI buffer specified by 
 * the given descriptor for whatever size is currently set
 * within the descriptor. Once the transaction is complete, 
 * the chip select line is once again pulled high. No error
 * checking occurs within this function. 
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void seal_flash_spi_transaction()
{
    if(activeFlashChip == 0)
    {
        /* Set slave select to communicate. */
        gpio_set_pin_level(MEM_CS0, false);
        gpio_set_pin_level(MEM_CS1, true);
    }
    else if(activeFlashChip == 1)
    {
        /* Set slave select to communicate. */
        gpio_set_pin_level(MEM_CS0, true);
        gpio_set_pin_level(MEM_CS1, false);
    }
    else
    {
        gpio_set_pin_level(MEM_CS0, true);
        gpio_set_pin_level(MEM_CS1, true);
    }

    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_MEMORY, &spi_flash_buff);
    
    /* De-select device. */
    gpio_set_pin_level(MEM_CS0, true);
    gpio_set_pin_level(MEM_CS1, true);
}

/*************************************************************
 * FUNCTION: seal_set_active_chip()
 * -----------------------------------------------------------
 * This function sets the active chip to a new value. The 
 * active value can presently accept values of 0, or 1. 
 * This value is used to determine which memory chip in the 
 * system should be written to for all operations. 
 *
 * Parameters:
 *      newActiveChip   :   New chip to select for operations.
 *
 * Returns: void
 *************************************************************/
void seal_set_active_chip(uint8_t newActiveChip)
{
    /* New chip represents an index. Chip 0 is the first chip
     * to be used. */
    if(newActiveChip < MAX_NUM_CHIPS)
    {
        activeFlashChip = newActiveChip;
    }
} 