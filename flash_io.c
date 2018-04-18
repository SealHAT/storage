/*
 * flash_io.c
 *
 * Created: 4/16/2018 3:10:23 PM
 *  Author: kmcarrin
 */ 

#include "flash_io.h"

/*************************************************************
 * FUNCTION: flash_io_read()
 * -----------------------------------------------------------
 * This function reads a number of bytes specified by "count".
 * If the value of count exceeds a page, only a full page of 
 * data will be returned through the buffer. The number of 
 * bytes actually read is returned from this function. 
 *
 * Parameters:
 *      fd      :   File descriptor for IO buffers.
 *      buf     :   Buffer to return read bytes in.
 *      count   :   Number of bytes to read. 
 *
 * Returns:
 *      size    :   Number of bytes actually read. 
 *************************************************************/
ssize_t flash_io_read(FLASH_DESCRIPTOR fd, void *buf, size_t count) {
    ssize_t amountRead = 0;
    /* Read data from the ping pong buffer into the user's given buffer. */
    /* If the user asks for more than a full page of data, only a full page
     * will be returned based on the size of PAGE_SIZE_EXTRA. */

    return (amountRead);
}

/*************************************************************
 * FUNCTION: flash_io_write()
 * -----------------------------------------------------------
 * This function writes a specific number of bytes to the 
 * flash device's write buffer. The number of bytes to write
 * is specified by "count". If count is greater than a page,
 * the buffer will not overflow and all data will still be 
 * written to the device. The number of bytes actually
 * written to the flash buffer is returned from this function.
 *
 * Parameters:
 *      fd      :   File descriptor for IO buffers.
 *      buf     :   Buffer containing bytes to write.
 *      count   :   Number of bytes to write. 
 *
 * Returns:
 *      size    :   Number of bytes actually written. 
 *************************************************************/
ssize_t flash_io_write(FLASH_DESCRIPTOR fd, void *buf, size_t count) {
    ssize_t amountWritten = 0;
    
    /* Write the full size of data into the ping pong buffer. If free space in 
     * ping pong buffer minus the count of data to write is less than or equal
     * to zero, then the buffer must be flushed and switched. */ 
    /* Loop until all data is written to the buffer. */ 
    /* Address will need to be updated after each write operation. */

    return (amountWritten);
}

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