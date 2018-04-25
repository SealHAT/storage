/*
 * flash_io.c
 *
 * Created: 4/16/2018 3:10:23 PM
 *  Author: kmcarrin
 */ 

#include "flash_io.h"

/* GLOBALS */
FLASH_ADDRESS_DESCRIPTOR flash_address;

/*************************************************************
 * FUNCTION: flash_io_init()
 * -----------------------------------------------------------
 * This function initializes the flash descriptor. This
 * function initializes the active buffer, buffer index, and
 * flash page size.
 *
 * Parameters:
 *      fd          :   Pointer to flash descriptor. 
 *      page_size   :   Page size of flash device.
 *
 * Returns: void
 *************************************************************/
void flash_io_init(FLASH_DESCRIPTOR *fd, int page_size)
{
    /* Initialize first ping-pong buffer to buffer 0 and it's index to 0. */
    fd->active_buffer = BUF_0;
    fd->buffer_index  = 0;
    
    /* Set the page size for this flash. */
    fd->PAGE_SIZE = page_size;
}

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
uint32_t flash_io_read(FLASH_DESCRIPTOR *fd, uint8_t *buf, size_t count)
{
    uint8_t  status;
    uint32_t amountRead = 0;
    
    if(flash_io_is_busy() == false)
    {
        /* Update next address pointer. */
        update_next_address();
            
        /* Read data into the active buffer. */
        if(fd->active_buffer == BUF_0)
        {
            /* Read into buffer 0. */
            status = flash_read(flash_address.currentAddress, 0x00, fd->buf_0, PAGE_SIZE_LESS);
            
            /* Point the user's buffer to the data just read. */
            buf = fd->buf_0;
            
            /* Set the active buffer to buffer 1. */
            fd->active_buffer = BUF_1;
        }
        else
        {
            /* Read into buffer 1. */
            status = flash_read(flash_address.currentAddress, 0x00, fd->buf_1, PAGE_SIZE_LESS);
            
            /* Point the user's buffer to the data just read. */
            buf = fd->buf_1;
            
            /* Set the active buffer to buffer 0. */
            fd->active_buffer = BUF_0;
        }

        /* Check for busy. Will block until read is complete. */
        if(flash_io_is_busy() == true)
        {
            /* Wait until device is done reading. */
            flash_wait_until_not_busy();
        }
        
        /* Update current address pointer. */
        update_current_address();
            
        /* Return amount. */
        if(count <= PAGE_SIZE_LESS)
        {
            amountRead = count;
        }
        else
        {
            amountRead = PAGE_SIZE_LESS;
        }
    } /* END if(flash_io_is_busy() == false) */
    else /* Device is busy, return error. */
    {
        amountRead = -1;
    }    

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
uint32_t flash_io_write(FLASH_DESCRIPTOR *fd, uint8_t *buf, size_t count)
{     
    /* Write the full size of data into the ping pong buffer. If free space in 
     * ping pong buffer minus the count of data to write is less than or equal
     * to zero, then the buffer must be flushed and switched. */ 
    /* Loop until all data is written to the buffer. */ 
    /* Address will need to be updated after each write operation. */
    uint8_t  status;
    uint32_t amountWritten = 0;
    bool     failed        = false;
    
    if(flash_io_is_busy() == false)
    {
        /* Write data into the buffers until all data has been written or until the operation fails. */
        do /* while((amountWritten < count) && (failed == false)) */
        {
            /* Update next address pointer. */
            update_next_address();
        
            /* Write data from the active buffer. */
            if(fd->active_buffer == BUF_0)
            {
                while((amountWritten < count) && (fd->buffer_index < PAGE_SIZE_LESS))
                {
                    fd->buf_0[fd->buffer_index] = buf[amountWritten];
                    fd->buffer_index++;
                }
            
                /* If the buffer is full, write it out to the flash chip after switching which buffer is active. */
                if(fd->buffer_index == PAGE_SIZE_LESS)
                {
                    /* Switch active buffer and reinitialize buffer index. */
                    fd->active_buffer = BUF_1;
                    fd->buffer_index  = 0;
                
                    /* Flush data if device is not busy. */
                    if(flash_io_is_busy() == false) {
                        status = flash_write(flash_address.currentAddress, 0x00, fd->buf_0, PAGE_SIZE_LESS);
                    } else {
                        failed = true;
                    }
                
                    /* Set failed flag if the write failed. */
                    if((status&PROG_FAIL) != 0)
                    {
                        failed = true;
                    }
                }
            }
            else /* (fd.active_buffer == BUF_1) */
            {
                while((amountWritten < count) && (fd->buffer_index < PAGE_SIZE_LESS))
                {
                    fd->buf_1[fd->buffer_index] = buf[amountWritten];
                    fd->buffer_index++;
                }
                        
                /* If the buffer is full, write it out to the flash chip after switching which buffer is active. */
                if(fd->buffer_index == PAGE_SIZE_LESS)
                {
                    /* Switch active buffer. */
                    fd->active_buffer = BUF_0;
                    fd->buffer_index  = 0;
                            
                    /* Flush data if device is not busy. */
                    if(flash_io_is_busy() == false) {
                        status = flash_write(flash_address.currentAddress, 0x00, fd->buf_1, PAGE_SIZE_LESS);
                    } else {
                        failed = true;
                    }                    
                    
                    /* Set failed flag if the write failed. */
                    if((status&PROG_FAIL) != 0)
                    {
                        failed = true;
                    }
                } /* END if(fd->buffer_index == PAGE_SIZE_LESS) */
            }
            
            /* Update current address pointer if the program operation didn't fail. */
            if(failed == false)
            {
                update_current_address();
            }            
        
        } while((amountWritten < count) && (failed == false));
        
        /* Return amount of data actually written. */
        if(failed == true)
        {
            if(amountWritten > PAGE_SIZE_LESS) {
                amountWritten -= PAGE_SIZE_LESS;
            } else {
                amountWritten = 0;
            }
        }

    } /* END if(flash_io_is_busy() == false) */
    else /* Device is busy, return error. */
    {
        amountWritten = -1;
    }

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
bool flash_io_is_busy() {
    return (flash_is_busy());
}

/*************************************************************
 * FUNCTION: flash_io_flush()
 * -----------------------------------------------------------
 * This function flushes the write buffer. All data currently
 * in the buffer will be written out to the flash device. 
 *
 * Parameters:
 *      fd  :   Descriptor containing buffers. 
 *
 * Returns: void
 *************************************************************/
void flash_io_flush(FLASH_DESCRIPTOR *fd)
{
    uint8_t status;
    bool    failed = false;
    /* Update next address pointer. */
    update_next_address();
    
    /* Write data from the active buffer. */
    if(fd->active_buffer == BUF_0)
    {
        /* Switch active buffer and reinitialize buffer index. */
        fd->active_buffer = BUF_1;
        fd->buffer_index  = 0;
            
        /* Flush data if device is not busy. */
        if(flash_io_is_busy() == false) {
            status = flash_write(flash_address.currentAddress, 0x00, fd->buf_0, fd->buffer_index);
        } else {
            failed = true;
        }
            
        if((status&PROG_FAIL) != 0)
        {
            failed = true;
        }
    }
    else /* (fd.active_buffer == BUF_1) */
    {
        /* Switch active buffer. */
        fd->active_buffer = BUF_0;
        fd->buffer_index  = 0;
            
        /* Flush data if device is not busy. */
        if(flash_io_is_busy() == false) {
            status = flash_write(flash_address.currentAddress, 0x00, fd->buf_1, fd->buffer_index);
        } else {
            failed = true;
        }
            
        if((status&PROG_FAIL) != 0)
        {
            failed = true;
        }
    }
    
    /* Update current address pointer if the program operation didn't fail. */
    if(failed == false)
    {
        update_current_address();
    }
    
}

/*************************************************************
 * FUNCTION: flash_io_reset_addr()
 * -----------------------------------------------------------
 * This function resets the address pointer back to the 
 * beginning of the flash device. The pointer goes back to 
 * block one of the device, which is the first addressable 
 * block by the user. 
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void flash_io_reset_addr()
{
    /* Calls the NAND_Flash function to reset the address pointer. */
    reset_address_info();
}