/*
 * flash_io.h
 *
 * Created: 4/16/2018 3:10:50 PM
 *  Author: kmcarrin
 */ 


#ifndef FLASH_IO_H_
#define FLASH_IO_H_

#include "NAND_Flash.h"

#define BUF_0       (0)     /* Buffer zero of the ping-pong buffer. */
#define BUF_1       (1)     /* Buffer one of the ping-pong buffer. */

/* Struct containing page data buffer, page size, and a count for the number of buffer bytes to use. 
 * Instantiate only a single descriptor in the main driving program. */
typedef struct 
{
    uint8_t buf_0[PAGE_SIZE_EXTRA];     /* Buffer for holding a page worth of data. Buffer 0 of the ping-pong buffer. */
    uint8_t buf_1[PAGE_SIZE_EXTRA];     /* Buffer for holding a page worth of data. Buffer 1 of the ping-pong buffer. */
    int     buffer_index;               /* Current read/write index of active buffer. */
    int     PAGE_SIZE;                  /* Size of page that user will see (should not include extra space bits). */
    int     active_buffer;              /* Keeps track of which buffer in the ping-pong buffer is currently in use. */
}FLASH_DESCRIPTOR;

/* THIS STRUCT IS FOR INTERNAL PROCESSING ONLY - DO NOT INSTANTIATE ANYWHERE ELSE. */
typedef struct
{
    uint32_t currentAddress;                    /* Address currently being written or read to/from. Updated after operation. */
    uint32_t nextAddress;                       /* Address that should be used next. Updated before operation. */
    uint8_t  currentChipInUse;                  /* Which chip is currently in use. [0, numChips-1]. Will be initialized to 0. */
} FLASH_ADDRESS_DESCRIPTOR;

/* THIS STRUCT IS FOR INTERNAL PROCESSING ONLY - DO NOT INSTANTIATE ANYWHERE ELSE. */
extern FLASH_ADDRESS_DESCRIPTOR flash_address;

/*************************************************************
 * FUNCTION: flash_io_init()
 * -----------------------------------------------------------
 * This function initializes the flash descriptor. This
 * function initializes the active buffer, buffer index, and
 * flash page size.
 *************************************************************/
void flash_io_init(FLASH_DESCRIPTOR *fd, int page_size);

/*************************************************************
 * FUNCTION: flash_io_read()
 * -----------------------------------------------------------
 * This function reads a number of bytes specified by "count".
 * If the value of count exceeds a page, only a full page of 
 * data will be returned through the buffer. The number of 
 * bytes actually read is returned from this function. 
 *************************************************************/
uint32_t flash_io_read(FLASH_DESCRIPTOR *fd, uint8_t *buf, size_t count);

/*************************************************************
 * FUNCTION: flash_io_write()
 * -----------------------------------------------------------
 * This function writes a specific number of bytes to the 
 * flash device's write buffer. The number of bytes to write
 * is specified by "count".
 *************************************************************/
uint32_t flash_io_write(FLASH_DESCRIPTOR *fd, uint8_t *buf, size_t count);

/*************************************************************
 * FUNCTION: flash_io_is_busy()
 * -----------------------------------------------------------
 * This function returns true if the flash device is currently
 * busy. It returns false if the device is currently not busy.
 *************************************************************/
bool flash_io_is_busy();

/*************************************************************
 * FUNCTION: flash_io_flush()
 * -----------------------------------------------------------
 * This function flushes the write buffer. All data currently
 * in the buffer will be written out to the flash device.
 *************************************************************/
void flash_io_flush(FLASH_DESCRIPTOR *fd);

/*************************************************************
 * FUNCTION: flash_io_reset_addr()
 * -----------------------------------------------------------
 * This function resets the address pointer back to the 
 * beginning of the flash device. The pointer goes back to 
 * block one of the device, which is the first addressable 
 * block by the user.
 *************************************************************/
void flash_io_reset_addr();

/*************************************************************
 * FUNCTION: update_next_address()
 * -----------------------------------------------------------
 * This function goes to the next address that should be 
 * written to or read from.
 *************************************************************/
uint32_t update_next_address();

/*************************************************************
 * FUNCTION: update_current_address()
 * -----------------------------------------------------------
 * This function goes to the next address that should be
 * written to or read from.
 *************************************************************/
uint32_t update_current_address();

/*************************************************************
 * FUNCTION: get_current_address()
 * -----------------------------------------------------------
 * This function returns the value currently stored in the 
 * address descriptor's current address value.
 *************************************************************/
uint32_t get_current_address();

/*************************************************************
 * FUNCTION: get_next_address()
 * -----------------------------------------------------------
 * This function returns the value currently stored in the 
 * address descriptor's current address value.
 *************************************************************/
uint32_t get_next_address();

/*************************************************************
 * FUNCTION: reset_address_info()
 * -----------------------------------------------------------
 * This function reinitializes the address pointer back to the 
 * beginning of the device. It points back to block one of the 
 * device.
 *************************************************************/
void reset_address_info();

#endif /* FLASH_IO_H_ */