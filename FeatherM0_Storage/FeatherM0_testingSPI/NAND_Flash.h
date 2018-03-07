/*
 * NAND_Flash.h
 *
 * Created: 2/23/2018 6:34:30 PM
 *  Author: kmcarrin
 */ 

/*
 *  ALL DATA AND ADDRESSES ARE ASSUMED TO BE BIG ENDIAN WHEN THEY ARRIVE
 *  WITHIN A FUNCTION. 
 */

#ifndef NAND_FLASH_H_
#define NAND_FLASH_H_

#include <hal_spi_m_sync.h>
#include <hal_delay.h>

#include "driver_init.h"
#include "HelperFunctions.h"

/* DEFINES */
#define PAGE_SIZE_EXTRA  (2176)         //maximum NAND Flash page size (including extra space)
#define PAGE_SIZE_LESS   (2048)         //maximum NAND Flash page size (excluding extra space)
#define BUFFER_SIZE      (2180)         //max SPI buffer size (page size plus room for commands)
#define BAD_BLK_ADDR     (0x800)        //Address of the bad block mark on the first page of each block
#define ECC_START_ADDR   (0x840)        //Address of start of error correction flags (last 8 bytes)
#define MAX_PAGE_SIZE    (2176)         //Maximum bytes per page. Includes spare area. 
#define NUM_BLOCKS       (2048)         //Number of blocks per pane. 2Gb device has 2 panes.

/* CONSTANT DECLARATIONS */
extern const uint8_t RESET[1];          //Command to reset the memory device
extern const uint8_t GET_FEAT[2];       //Command to get the current contents of the status register
extern const uint8_t SET_WEL[1];        //Command to set the write enable bit in in the status register
extern const uint8_t PROG_LOAD[3];      //Command to load data from micro into cache of memory device. Last 2 bytes page column address
extern const uint8_t PEXEC[4];          //Command to program the main memory array from the memory devices cache register
extern const uint8_t PAGE_READ[4];      //Command to read data from main array into data cache
extern const uint8_t READ_CACHE[3];     //Command to read data from memory device cache to SPI buffer
extern const uint8_t ERASE[1];          //Command to erase a block of data
extern const uint8_t GET_BLOCK_LOCK[2]; //Command to check the block lock status
extern const uint8_t UNLOCK_BLOCKS[3];  //Command to check the block lock status

/* MASKS */
extern const uint8_t BUSY_MASK;         //Mask for checking if the flash is busy
extern const uint8_t PROG_FAIL;         //Mask for checking if the memory was programmed successfully
extern const uint8_t WEL_MASK;          //Mask for checking if write enable is high

/* SPI COMMUNICATION BUFFERS */
uint8_t  MOSI[BUFFER_SIZE];             //Master's output buffer
uint8_t  MISO[BUFFER_SIZE];             //Master's input buffer
struct   spi_xfer spi_buff;             //SPI transfer descriptor

/* FUNCTION DECLARATIONS */
/*************************************************************
 * FUNCTION: flash_initSPI()
 * -----------------------------------------------------------
 * This function initializes the SPI communication that will
 * be used to communicate with the memory device. It sets the
 * input and output buffers as well as the SPI buffer size. 
 * In addition, it sets the SPI mode to Mode 0 and enables
 * the SPI device. 
 *************************************************************/
void flash_initSPI();

/*************************************************************
 * FUNCTION: flash_init_buffers()
 * -----------------------------------------------------------
 * This function initializes the buffers that will be used for
 * SPI communication. The input buffer is initialized to all
 * 0xFF values. The output buffer, on the other hand, is 
 * initialized to all zeros. 
 *************************************************************/
void flash_init_buffers();

/*************************************************************
 * FUNCTION: flash_setSPI_buffer_size()
 * -----------------------------------------------------------
 * This function is a utility function used to set the size
 * of the SPI buffer.
 *************************************************************/
void flash_setSPI_buffer_size(int newSize);

/*************************************************************
 * FUNCTION: flash_reset()
 * -----------------------------------------------------------
 * This function issues the RESET command: 0xFF. The command 
 * is placed in the output buffer and then sent over the SPI
 * connection only if the device is not currently busy.
 *************************************************************/
uint8_t flash_reset();

/*************************************************************
 * FUNCTION: flash_status()
 * -----------------------------------------------------------
 * Issue the GET FEATURES command: 0x0F 
 * Send address of FEATURES register: 0xC0
 * Put command in MOSI buffer. 
 *************************************************************/
uint8_t flash_status();

/*************************************************************
 * FUNCTION: flash_set_WEL()
 * -----------------------------------------------------------
 * This function issues the Set Write Enable Latch command to
 * the memory device. The command is placed into the output 
 * buffer and sent to the device. 
 *************************************************************/
uint8_t flash_set_WEL();

/*************************************************************
 * FUNCTION: flash_write_page()
 * -----------------------------------------------------------
 * This function writes a page of data to the Flash device.
 * First, the write enable flag (WEL bit) is set. Next, the
 * device's cache is loaded with the given data. Once that
 * Operation is complete, the data is written from the
 * device's cache into its main memory array.
 *************************************************************/
uint8_t flash_write_page(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[]);

/*************************************************************
 * FUNCTION: flash_wait_until_not_busy()
 * -----------------------------------------------------------
 * This function continuously checks the status register. The
 * function returns once the status register shows that the 
 * device is no longer busy.
 *************************************************************/
void flash_wait_until_not_busy();

/*************************************************************
 * FUNCTION: flash_read_page()
 * -----------------------------------------------------------
 * This function reads a page of data from the Flash device.
 * First, a page of data is read from the main memory array of
 * the device into the data cache. Then, the data is
 * transfered from the cache to the input buffer of the micro.
 *************************************************************/
uint8_t flash_read_page(uint8_t blockPageAddress[], uint8_t columnAddress[], uint8_t pageData[]);

/*************************************************************
 * FUNCTION: flash_is_busy()
 * -----------------------------------------------------------
 * This function checks the busy flag in the status register. 
 * If the device is NOT busy, then the status will become
 * false, signifying that the device is not busy.
 *************************************************************/
bool flash_is_busy();

/*************************************************************
 * FUNCTION: flash_block_erase()
 * -----------------------------------------------------------
 * This function erases an entire block of memory from the 
 * flash device. A block is the minimum size unit that is able
 * to be erased within the device.
 *************************************************************/
uint8_t flash_block_erase(uint8_t blockAddress[]);

/*************************************************************
 * FUNCTION: flash_block_lock_status()
 * -----------------------------------------------------------
 * This function gets the status of the block lock register.
 * The block lock register lists the blocks of the flash 
 * device which are currently locked. 
 *************************************************************/
uint8_t flash_block_lock_status();

/*************************************************************
 * FUNCTION: flash_unlock_all_blocks()
 * -----------------------------------------------------------
 * This function unlocks all blocks within the flash device
 * for reading and writing.
 *************************************************************/
uint8_t flash_unlock_all_blocks();

/* INTERNAL FUNCTIONS - NOT A PART OF API */
/*************************************************************
 * FUNCTION: program_load()
 * -----------------------------------------------------------
 * This function loads data from the host device into the 
 * memory device's data cache. 
 *************************************************************/
uint8_t program_load(uint8_t data[], int dataSize, uint8_t colAddress[]);

/*************************************************************
 * FUNCTION: execute_program()
 * -----------------------------------------------------------
 * This function takes data from the memory device's data
 * cache and moves them into the main memory array.
 *************************************************************/
uint8_t execute_program(uint8_t blockAddress[]);

/*************************************************************
 * FUNCTION: reinitialize_out_buff()
 * -----------------------------------------------------------
 * This function reinitializes the output buffer used in the
 * SPI transactions.
 *************************************************************/
void reinitialize_out_buff();

/*************************************************************
 * FUNCTION: page_read()
 * -----------------------------------------------------------
 * This function reads a page of data from the memory device's
 * main memory array and puts it into the data cache.
 *************************************************************/
uint8_t page_read(uint8_t blockPageAddress[]);

/**************************************************************
 * FUNCTION: build_bad_block_table()
 * ------------------------------------------------------------
 * This function reads a page of data from the memory device's
 * data cache and sends it to the host device's input buffer
 * via an SPI transaction.
 *************************************************************/
uint8_t read_from_cache(uint8_t columnAddress[], uint8_t pageData[]); 

//returns number of bad blocks found
uint32_t build_bad_block_table();

#endif /* NAND_FLASH_H_ */