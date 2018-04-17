/*
 * flash_io.h
 *
 * Created: 4/16/2018 3:10:50 PM
 *  Author: kmcarrin
 */ 


#ifndef FLASH_IO_H_
#define FLASH_IO_H_

#include "NAND_Flash.h"

/* Struct containing page data buffer, page size, and a count for the number of buffer bytes to use. */
typedef struct
{
    uint8_t buf_0[PAGE_SIZE_EXTRA];     /* Buffer for holding a page worth of data. Buffer 0 of the ping-pong buffer. */
    uint8_t buf_1[PAGE_SIZE_EXTRA];     /* Buffer for holding a page worth of data. Buffer 1 of the ping-pong buffer. */
    int     PAGE_SIZE;                  /* Size of page that user will see (should not include extra space bits). */
    bool    active_descriptor;
} FLASH_DESCRIPTOR;

/*************************************************************
 * FUNCTION: flash_io_read()
 * -----------------------------------------------------------
 * This function reads 
 *
 * Parameters:
 *      fd      :   File descriptor for IO buffers.
 *      buf     :   Buffer to return read bytes in.
 *      count   :   Number of bytes to read. 
 *
 * Returns:
 *      size    :   Number of bytes actually read. 
 *************************************************************/
ssize_t flash_io_read(FLASH_DESCRIPTOR fd, void *buf, size_t count);

/*************************************************************
 * FUNCTION: flash_io_write()
 * -----------------------------------------------------------
 * This function 
 *
 * Parameters:
 *      fd      :   File descriptor for IO buffers.
 *      buf     :   Buffer containing bytes to write.
 *      count   :   Number of bytes to write. 
 *
 * Returns:
 *      size    :   Number of bytes actually written. 
 *************************************************************/
ssize_t flash_io_write(FLASH_DESCRIPTOR fd, void *buf, size_t count);

/*************************************************************
 * FUNCTION: flash_io_is_busy()
 * -----------------------------------------------------------
 * This function returns true if the flash device is currently
 * busy. It returns false if the device is currently not busy.
 *
 * Parameters: none
 *
 * Returns:
 *      isBusy    :   Returns TRUE if flash is busy.
 *************************************************************/
bool flash_io_is_busy();

#endif /* FLASH_IO_H_ */