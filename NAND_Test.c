/*
 * NAND_Test.c
 *
 * Created: 10-Mar-18 12:20:56 AM
 *  Author: Krystine
 */ 

#include "NAND_Test.h"

static uint32_t dataIdx;

/*************************************************************
 * FUNCTION: nand_test_driver()
 * -----------------------------------------------------------
 * This function 
 *
 * Parameters: none
 *
 * Returns:
 *      status  :   Current value of the status register.
 *************************************************************/
uint8_t nand_test_driver()
{
    uint8_t  status;                    /* Value of the flash status register. */
    uint8_t  retVal;                    /* Return value of functions that don't return a status. */
    int      blockCount;                /* Loop control variable for writes and reads. Iterate over blocks. */
    uint32_t pageCount;                 /* Loop control variable for writes and reads. Iterate over pages. */
    uint32_t blockAddress;              /* The micro stores values little endian but is configured to send SPI data big endian */
    uint32_t colAddress;                /* Column address for in-page offset */
    uint32_t address;                   /* The OR'd bits of block and page addresses to create a single 3 byte address. */
    uint32_t nextBlock;                 /* Amount to increase address by to get to next block */
    uint8_t  page[PAGE_SIZE_EXTRA];     /* Holds a page worth (PAGE_SIZE bytes) of data. */

    
    /* INITIALIZATIONS */
    nextBlock    = 0x40;
    blockAddress = 0x000000;
    colAddress   = 0x0000;
    dataIdx      = 0;
    
    /**************
     * INITIALIZE *
     **************/
    /* Set up USB connection */
    /* Wait for USB connection to be made with computer. */
    do { /* NOTHING */ } while (!usb_dtr());
        
    /* Write welcome message to PC console. */
    retVal = usb_write((uint8_t *) WELCOME, sizeof(WELCOME));
    
    /* Initialize the flash interface. */
    flash_init();
    
    /* Start by erasing the entire device (except the superblock). */
    status = flash_erase_device();
    
    /* Wait until device is done erasing. */
    flash_wait_until_not_busy();
    
    /*********
     * WRITE *
     *********/
    do {
        retVal = usb_write((uint8_t *) START_WRITE, sizeof(START_WRITE));
    } while(retVal < 0);
    
    /* Iterate over a total of NUM_TEST_BLOCKS blocks while writing data to each
     * page within the block. */
    for(blockCount = 0; blockCount < NUM_TEST_BLOCKS; blockCount++)
    {
        /* Iterate through 10 unique pages worth of data to fill up all 64 pages
         * of the block. */
        for(pageCount = 0; pageCount < PAGES_PER_BLOCK; pageCount++)
        {
            /* Get next page data to write. */
            nand_test_load_data(page, PAGE_SIZE_LESS);
            
            /* Bitwise OR block count a page count to get 3-byte data address. */
            address = (blockAddress | pageCount);
            
            //write test data
            status = flash_write(address, colAddress, page, PAGE_SIZE_LESS);
        
            /* Wait until device is done erasing. */
            flash_wait_until_not_busy();
        }
        
        blockAddress += nextBlock;
    }
    
    /********
     * READ *
     ********/
    blockAddress = 0;
    
    do {
        retVal = usb_write((uint8_t *) DONE_WRITE, sizeof(DONE_WRITE));
    } while(retVal < 0);
    
    /* Read data back and print over USB. Data integrity will 
     * be checked on the PC end. */
    do {
        retVal = usb_write((uint8_t *) START_READ, sizeof(START_READ));
    } while(retVal < 0);
    
    // TODO - write read code here. print via usb connection
    
    do {
        retVal = usb_write((uint8_t *) DONE_READ, sizeof(DONE_READ));
    } while(retVal < 0);
  
}


  
void nand_test_load_data(uint8_t page[], int pageSize)
{
    
    // TODO - load data from page data array
}
//uint8_t TEST_DATA[TEST_DATA_SIZE]
