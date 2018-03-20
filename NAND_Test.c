/*
 * NAND_Test.c
 *
 * Created: 10-Mar-18 12:20:56 AM
 *  Author: Krystine
 */ 

#include "NAND_Test.h"

static uint32_t dataIdx;

char WELCOME[31]     = "About to initialize device...\n";
char GOODBYE[10]     = "Goodbye!\n";
char START_WRITE[22] = "Device writing begin!\n";
char DONE_WRITE[25]  = "Device writing complete.\n";
char START_READ[22]  = "Begin device reading.\n";
char DONE_READ[25]   = "Device reading complete!\n";
char NEW_LINE        = '\n';
static char charBuffer[5];

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
    volatile uint8_t  status;                    /* Value of the flash status register. */
    int      retVal;                    /* Return value of functions that don't return a status. */
    int      blockCount;                /* Loop control variable for writes and reads. Iterate over blocks. */
    uint32_t pageCount;                 /* Loop control variable for writes and reads. Iterate over pages. */
    uint32_t blockAddress;              /* The micro stores values little endian but is configured to send SPI data big endian */
    uint32_t colAddress;                /* Column address for in-page offset */
    volatile uint32_t address;                   /* The OR'd bits of block and page addresses to create a single 3 byte address. */
    uint32_t nextBlock;                 /* Amount to increase address by to get to next block */
    uint8_t  page[PAGE_SIZE_EXTRA];     /* Holds a page worth (PAGE_SIZE bytes) of data. */
    int      i;                         /* Loop control variable for printing. */
    
    /* INITIALIZATIONS */
    nextBlock    = 0x40;
    blockAddress = 0x000040;
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
            
            /* Write test data. */
            status = flash_write(address, colAddress, page, PAGE_SIZE_LESS);
        
            /* Wait until device is done erasing. */
            flash_wait_until_not_busy();
        }
        
        /* Go to next block. */
        blockAddress += nextBlock;
    }
    
    /********
     * READ *
     ********/
    blockAddress = 0x000040;
    
    do {
        retVal = usb_write((uint8_t *) DONE_WRITE, sizeof(DONE_WRITE));
    } while(retVal < 0);
    
    /* Read data back and print over USB. Data integrity will 
     * be checked on the PC end. */
    do {
        retVal = usb_write((uint8_t *) START_READ, sizeof(START_READ));
    } while(retVal < 0);

    /* Iterate over a total of NUM_TEST_BLOCKS blocks while reading data from each
     * page within the block. */
    for(blockCount = 0; blockCount < NUM_TEST_BLOCKS; blockCount++)
    {
        /* Iterate through every page in the block and print the data to the PC. */
        for(pageCount = 0; pageCount < PAGES_PER_BLOCK; pageCount++)
        {            
            /* Bitwise OR block count a page count to get 3-byte data address. */
            address = (blockAddress | pageCount);
            
            /* Read test data. */
            status = flash_read(address, colAddress, page, PAGE_SIZE_LESS);
        
            /* Wait until device is done erasing. */
            flash_wait_until_not_busy();
            
            for(i = 0; i < PAGE_SIZE_LESS; i++)
            {
                sprintf(charBuffer, "%d\n", page[i]);
                
                /* Print page data. */
                do {
                    retVal = usb_write(charBuffer, strlen(charBuffer));
                } while(retVal != USB_OK || retVal < 0);
                
                /* Print new line character. */
                //do {
                //    retVal = usb_write(&NEW_LINE, 1);
                //} while(retVal < 0);
            }                
                
        } /* End pages per block loop. */
        
        /* Go to next block. */
        blockAddress += nextBlock;
        
    } /* End blocks per device loop. */

    /* Print done message. */
    do {
        retVal = usb_write((uint8_t *) DONE_READ, sizeof(DONE_READ));
    } while(retVal < 0);
  
  return 0;
}

 /*************************************************************
 * FUNCTION: nand_test_load_data()
 * -----------------------------------------------------------
 * This function loads a page of data based on the PAGE_SIZE.
 * Data is taken from a static TEST_DATA array containing 
 * TEST_DATA_SIZE elements. The value of TEST_DATA_SIZE must
 * be greater than PAGE_SIZE.
 *
 * Parameters:
 *      page[]      :   Array to hold new page data.
 *      PAGE_SIZE   :   Amount of data to load into page[].
 *
 * Returns:
 *      page[]      :   New data returned by reference.
 *************************************************************/ 
void nand_test_load_data(uint8_t page[], const int PAGE_SIZE)
{
    int i;  /* Loop control variable. */
    
    /* Load data from page data array. */
    for(i = 0; i < PAGE_SIZE; i++)
    {
        page[i] = TEST_DATA[dataIdx];
        
        dataIdx++;
        
        /* Move data array index back to zero if it is at the
         * last byte of data. */
        if(TEST_DATA_SIZE == dataIdx)
        {
            dataIdx = 0;
        }
    }
}