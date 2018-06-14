/*
 * NAND_Flash.h
 *
 * Created: 2/23/2018 6:34:30 PM
 *  Author: kmcarrin
 */ 

/*
 *  ALL DATA AND ADDRESSES ARE ASSUMED TO BE LITTLE ENDIAN WHEN THEY ARRIVE
 *  WITHIN A FUNCTION. They are then swapped to big endian before being sent
 *  to the flash device. IF THE MASTER DEVICE IS BIG ENDIAN, THE FOLLOWING
 *  FUNCTIONS MUST BE UPDATED:
 *      execute_program()       - address bytes must be swapped before SPI transfer
 *      page_read()             - address bytes must be swapped before SPI transfer
 *      flash_read_superblock() - bitshift ops addresses must be swapped
 *      init_cache_superblock() - bitshift ops addresses must be swapped
 *      flash_block_erase()     - address bytes must be swapped before SPI transfer
 *      build_bad_block_table() - remove func call to LitToBigEndian
 */

#ifndef NAND_FLASH_H_
#define NAND_FLASH_H_

#include "driver_init.h"
#include "HelperFunctions.h"

/* DEFINES */
#define PAGE_SIZE_EXTRA     (2176)              /* Maximum NAND Flash page size (*including* extra space) */
#define PAGE_SIZE_LESS      (2048)              /* Maximum NAND Flash page size (*excluding* extra space) */
#define MAX_BAD_BLOCKS      (40)                /* Guaranteed maximum number of bad blocks for this device */
#define NAND_BUFFER_SIZE    (2180)              /* Max SPI buffer size (page size plus room for commands) */
#define BAD_BLK_ADDR        (0x800)             /* Address of the bad block mark on the first page of each block */
#define ECC_START_ADDR      (0x840)             /* Address of start of error correction flags (last 8 bytes) */
#define MAX_PAGE_SIZE       (2176)              /* Maximum bytes per page. Includes spare area */
#define NUM_BLOCKS          (2048)              /* Maximum number of blocks within the device */
#define PAGES_PER_BLOCK     (64)                /* Number of pages within each block of a device */
#define SIGNATURE_SIZE      (8)                 /* The signature in the superblock is 8 bytes long */

#if SEALHAT_HARDWARE_VERSION == 10060
    #define MAX_NUM_CHIPS       (4)             /* Update this value to match however many flash chips the system uses. */
#else
    #define MAX_NUM_CHIPS       (2)             /* Update this value to match however many flash chips the system uses. */
#endif

typedef struct 
{
    char      signature[8];                     /* Contains an 8-byte code to ensure the device has been configured. */
    uint8_t   badBlockCount;                    /* How many bad blocks are currently known on the device. */
    uint32_t  badBlockTable[MAX_BAD_BLOCKS];    /* Contains an array of bad block addresses that should not be written to. */
    uint8_t   badBlockIndex;                    /* Keeps track of next bad block to look out for. */
} SUPERBLOCK_t;

/* MASKS */
extern uint8_t BUSY_MASK;                       /* Mask for checking if the flash is busy */
extern uint8_t PROG_FAIL;                       /* Mask for checking if the memory was programmed successfully */
extern uint8_t WEL_MASK;                        /* Mask for checking if write enable is high */

/* SPI COMMUNICATION BUFFERS */
extern struct  spi_xfer spi_flash_buff;         /* SPI transfer descriptor */
extern uint8_t activeFlashChip;                 /* Current active flash chip. Currently supports up to three chips, with the value 00 being all chips deselected. */

/* BAD BLOCK TABLE - the device is guaranteed to have a maximum of 2% of its blocks go bad 
 * within its lifetime. For this device, that means a maximum total of 41 blocks. */
extern uint32_t badBlockTable[MAX_BAD_BLOCKS];

/* FUNCTION DECLARATIONS */
/*************************************************************
 * FUNCTION: seal_flash_init()
 * -----------------------------------------------------------
 * This function calls the other flash initialization 
 * functions to initialize SPI buffers, SPI device, and 
 * initialize the bad block table.
 *************************************************************/
void seal_flash_init();

/*************************************************************
 * FUNCTION: seal_flash_read_superblock()
 * -----------------------------------------------------------
 * This function reads the first page of the device for data.
 * The data stored here should be a struct of superblock
 * values.
 *************************************************************/
void seal_flash_read_superblock();

/*************************************************************
 * FUNCTION: seal_validate_superblock()
 * -----------------------------------------------------------
 * This function takes the superblock data and validates it.
 * Both the device signature and the number of bad blocks are
 * validated. 
 *************************************************************/
bool seal_validate_superblock();

/*************************************************************
 * FUNCTION: seal_init_cache_superblock()
 * -----------------------------------------------------------
 * This function takes the superblock data and stores it in a
 * struct that may be accessed during runtime. This function
 * does not check to see if the data is valid. That check is
 * done in the validate_superblock function.
 *************************************************************/
void seal_init_cache_superblock(uint8_t page[], int pageSize);

/*************************************************************
 * FUNCTION: seal_flash_initSPI()
 * -----------------------------------------------------------
 * This function initializes the SPI communication that will
 * be used to communicate with the memory device. It sets the
 * input and output buffers as well as the SPI buffer size. 
 * In addition, it sets the SPI mode to Mode 0 and enables
 * the SPI device. 
 *************************************************************/
void seal_flash_initSPI();

/*************************************************************
 * FUNCTION: seal_flash_init_buffers()
 * -----------------------------------------------------------
 * This function initializes the buffers that will be used for
 * SPI communication. The input buffer is initialized to all
 * 0xFF values. The output buffer, on the other hand, is 
 * initialized to all zeros. 
 *************************************************************/
void seal_flash_init_buffers();

/*************************************************************
 * FUNCTION: seal_flash_init_BBT()
 * -----------------------------------------------------------
 * This function initializes the bad block table that will
 * hold addresses of known bad blocks within the device.
 *************************************************************/
void seal_flash_init_BBT();

/*************************************************************
 * FUNCTION: seal_flash_setSPI_buffer_size()
 * -----------------------------------------------------------
 * This function is a utility function used to set the size
 * of the SPI buffer.
 *************************************************************/
void seal_flash_setSPI_buffer_size(int newSize);

/*************************************************************
 * FUNCTION: seal_flash_reset()
 * -----------------------------------------------------------
 * This function issues the RESET command: 0xFF. The command 
 * is placed in the output buffer and then sent over the SPI
 * connection only if the device is not currently busy.
 *************************************************************/
uint8_t seal_flash_reset();

/*************************************************************
 * FUNCTION: seal_flash_status()
 * -----------------------------------------------------------
 * Issue the GET FEATURES command: 0x0F 
 * Send address of FEATURES register: 0xC0
 * Put command in MOSI buffer. 
 *************************************************************/
uint8_t seal_flash_status();

/*************************************************************
 * FUNCTION: seal_flash_set_WEL()
 * -----------------------------------------------------------
 * This function issues the Set Write Enable Latch command to
 * the memory device. The command is placed into the output 
 * buffer and sent to the device. 
 *************************************************************/
uint8_t seal_flash_set_WEL();

/*************************************************************
 * FUNCTION: seal_flash_write_page()
 * -----------------------------------------------------------
 * This function writes a page of data to the Flash device.
 * First, the write enable flag (WEL bit) is set. Next, the
 * device's cache is loaded with the given data. Once that
 * Operation is complete, the data is written from the
 * device's cache into its main memory array.
 *************************************************************/
uint8_t seal_flash_write_page(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[]);

/*************************************************************
 * FUNCTION: seal_flash_wait_until_not_busy()
 * -----------------------------------------------------------
 * This function continuously checks the status register. The
 * function returns once the status register shows that the 
 * device is no longer busy.
 *************************************************************/
void seal_flash_wait_until_not_busy();

/*************************************************************
 * FUNCTION: seal_flash_read_page()
 * -----------------------------------------------------------
 * This function reads a page of data from the Flash device.
 * First, a page of data is read from the main memory array of
 * the device into the data cache. Then, the data is
 * transfered from the cache to the input buffer of the micro.
 *************************************************************/
uint8_t seal_flash_read_page(uint8_t blockPageAddress[], uint8_t columnAddress[], uint8_t pageData[]);

/*************************************************************
 * FUNCTION: seal_flash_read()
 * -----------------------------------------------------------
 * This function calls the flash_read_page function to read a 
 * page of data. This occurs only after the block address has
 * been adjusted to account for the bad blocks that were 
 * skipped.
 *************************************************************/
uint8_t seal_flash_read(uint32_t blockAddress, uint32_t columnAddress, uint8_t dataBuffer[], int dataSize);

/*************************************************************
 * FUNCTION: seal_flash_write()
 * -----------------------------------------------------------
 * This function calls the flash_write_page function to write
 * a page of data. This occurs only after the block address
 * has  been adjusted to account for the bad blocks that were
 * skipped.
 *************************************************************/
uint8_t seal_flash_write(uint32_t blockAddress, uint32_t columnAddress, uint8_t dataBuffer[], int dataSize);

/*************************************************************
 * FUNCTION: seal_flash_is_busy()
 * -----------------------------------------------------------
 * This function checks the busy flag in the status register. 
 * If the device is NOT busy, then the status will become
 * false, signifying that the device is not busy.
 *************************************************************/
bool seal_flash_is_busy();

/*************************************************************
 * FUNCTION: seal_flash_block_erase()
 * -----------------------------------------------------------
 * This function erases an entire block of memory from the 
 * flash device. A block is the minimum size unit that is able
 * to be erased within the device.
 *************************************************************/
uint8_t seal_flash_block_erase(uint8_t blockAddress[]);

/*************************************************************
 * FUNCTION: seal_flash_erase_device()
 * -----------------------------------------------------------
 * This function erases the entire flash device except for the
 * first block of data. 
 *************************************************************/
uint8_t seal_flash_erase_device();

/*************************************************************
 * FUNCTION: seal_flash_block_lock_status()
 * -----------------------------------------------------------
 * This function gets the status of the block lock register.
 * The block lock register lists the blocks of the flash 
 * device which are currently locked. 
 *************************************************************/
uint8_t seal_flash_block_lock_status();

/*************************************************************
 * FUNCTION: seal_flash_unlock_all_blocks()
 * -----------------------------------------------------------
 * This function unlocks all blocks within the flash device
 * for reading and writing.
 *************************************************************/
uint8_t seal_flash_unlock_all_blocks();

/* INTERNAL FUNCTIONS - NOT A PART OF API */
/*************************************************************
 * FUNCTION: seal_program_load()
 * -----------------------------------------------------------
 * This function loads data from the host device into the 
 * memory device's data cache. 
 *************************************************************/
uint8_t seal_program_load(uint8_t data[], int dataSize, uint8_t colAddress[]);

/*************************************************************
 * FUNCTION: seal_execute_program()
 * -----------------------------------------------------------
 * This function takes data from the memory device's data
 * cache and moves them into the main memory array.
 *************************************************************/
uint8_t seal_execute_program(uint8_t blockAddress[]);

/*************************************************************
 * FUNCTION: seal_reinitialize_out_buff()
 * -----------------------------------------------------------
 * This function reinitializes the output buffer used in the
 * SPI transactions.
 *************************************************************/
void seal_reinitialize_out_buff();

/*************************************************************
 * FUNCTION: seal_page_read()
 * -----------------------------------------------------------
 * This function reads a page of data from the memory device's
 * main memory array and puts it into the data cache.
 *************************************************************/
uint8_t seal_page_read(uint8_t blockPageAddress[]);

/**************************************************************
 * FUNCTION: seal_build_bad_block_table()
 * ------------------------------------------------------------
 * This function reads a page of data from the memory device's
 * data cache and sends it to the host device's input buffer
 * via an SPI transaction.
 *************************************************************/
uint8_t seal_read_from_cache(uint8_t columnAddress[], uint8_t pageData[]); 

/**************************************************************
 * FUNCTION: seal_build_bad_block_table()
 * ------------------------------------------------------------
 * This function creates a list of bad blocks within the 
 * device.
 *************************************************************/
uint8_t seal_build_bad_block_table();

/**************************************************************
 * FUNCTION: seal_flash_get_superblock()
 * ------------------------------------------------------------
 * This function returns the address of the superblock. Only 
 * for testing.
 *************************************************************/
SUPERBLOCK_t *seal_flash_get_superblock();

/*************************************************************
 * FUNCTION: seal_calculate_block_offset()
 * -----------------------------------------------------------
 * This function calculates what the "correct" address should
 * be for a given block address based on the bad block table.
 * Bad blocks in the system will be skipped, and thus an 
 * offset must occur.
 *************************************************************/
uint32_t seal_calculate_block_offset(uint32_t startingBlockAddress);

/*************************************************************
 * FUNCTION: seal_flash_spi_transaction()
 * -----------------------------------------------------------
 * This function completes and SPI transaction with the SPI
 * descriptor currently associated with the flash chip(s).
 *************************************************************/
void seal_flash_spi_transaction();

/*************************************************************
 * FUNCTION: seal_set_active_chip()
 * -----------------------------------------------------------
 * This function sets the active chip to a new value. The 
 * active value can presently accept values of 0, or 1.
 *************************************************************/
void seal_set_active_chip(uint8_t newActiveChip);

uint16_t seal_flash_read_id();

#endif /* NAND_FLASH_H_ */