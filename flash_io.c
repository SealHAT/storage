/*
 * flash_io.c
 *
 * Created: 4/16/2018 3:10:23 PM
 *  Author: kmcarrin
 */ 

#include "flash_io.h"

/* GLOBALS */
static FLASH_ADDRESS_DESCRIPTOR flash_address;
bool FLASH_IS_FULL;

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
    int i = 0;  /* Loop control variable. */
    
    /* Will be true after all memory is filled */
    FLASH_IS_FULL = false;
    
    /* Initialize the external flash device(s). */
    while(i < MAX_NUM_CHIPS)
    {
        seal_flash_init();
        seal_flash_erase_device();
        seal_set_active_chip(i);
        i++;
    }
    
    /* Set the active chip back to chip 0. */
    seal_set_active_chip(0);
    
    /* Initialize the buffer index to 0. */
    fd->buffer_index = 0;
    
    /* Set the page size for this flash. */
    fd->PAGE_SIZE = page_size;
    
    /* Initialize the address descriptor. */
    flash_io_reset_addr();
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
    uint32_t amountRead = 0;    /* Number of bytes actually read. */
    (void) fd;
    
    if(flash_io_is_busy() == false)
    {
        /* Update next address pointer. */
        update_next_address();
            
        /* Read data into the active buffer. */
        seal_flash_read(flash_address.currentAddress, 0x00, buf, PAGE_SIZE_LESS);
            
        /* Check for busy. Will block until read is complete. */
        if(flash_io_is_busy() == true)
        {
            /* Wait until device is done reading. */
            seal_flash_wait_until_not_busy();
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
    uint8_t  status;                /* Status of flash write operations. */
    uint32_t amountWritten = 0;     /* Amount of data actually written. */
    bool     failed        = false; /* Holds success or failure of write operation. */
    
    /* Write data to flash buffer if the device is not currently busy. */
    if(flash_io_is_busy() == false)
    {
        /* Write data into the buffers until all data has been written or until the operation fails. */
        do /* while((amountWritten < count) && (failed == false)) */
        {        
            /* Write data from the active buffer. */

            while((amountWritten < count) && (fd->buffer_index < PAGE_SIZE_LESS))
            {
                fd->buf_0[fd->buffer_index] = buf[amountWritten];
                fd->buffer_index++;
                amountWritten++;
            }
            
            /* If the buffer is full, write it out to the flash chip after switching which buffer is active. */
            if(fd->buffer_index == PAGE_SIZE_LESS)
            {
                /* Update next address pointer. */
                update_next_address();
                
                /* Switch active buffer and reinitialize buffer index. */
                fd->buffer_index = 0;
                
                /* Flush data if device is not busy. */
                if(flash_io_is_busy() == false) {
                    status = seal_flash_write(flash_address.currentAddress, 0x00, fd->buf_0, PAGE_SIZE_LESS);
                } else {
                    failed = true;
                }
                
                /* Set failed flag if the write failed. */
                if((status&PROG_FAIL) != 0)
                {
                    failed = true;
                }
                
                /* Update current address pointer if the program operation didn't fail. */
                if(failed == false)
                {
                    update_current_address();
                } 
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
    return (seal_flash_is_busy());
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
    
    if(fd->buffer_index != 0)
    {
        /* Update next address pointer. */
        update_next_address();
            
        /* Flush data if device is not busy. */
        if(flash_io_is_busy() == false) {
            status = seal_flash_write(flash_address.currentAddress, 0x00, fd->buf_0, fd->buffer_index);
        } else {
            failed = true;
        }
        
        /* Write data from the active buffer. */
        /* Reinitialize buffer index. */
        fd->buffer_index  = 0;
            
        if((status&PROG_FAIL) != 0)
        {
            failed = true;
        }

        /* Update current address pointer if the program operation didn't fail. */
        if(failed == false)
        {
            update_current_address();
        }
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

/*************************************************************
 * FUNCTION: update_next_address()
 * -----------------------------------------------------------
 * This function goes to the next address that should be 
 * written to or read from. If the block currently being
 * updated is at or past the final allowed value based on 
 * NUM_BLOCKS, then an error message is thrown.  
 *
 * Parameters: none
 *
 * Returns:
 *      address	: Updated address.
 *************************************************************/
uint32_t update_next_address() {
	/* Check if block out of main array. */
    if(seal_calculate_block_offset(flash_address.currentAddress) >= NUM_BLOCKS)
    {
        /* Make sure there is still more space on the device. */
        if(flash_address.currentChipInUse < (MAX_NUM_CHIPS - 1))
        {
            /* Switch chip and set address pointer. */
            switch_flash_chips();
        }
        else
        {
            /* Device out of space! Set global flag. */
            FLASH_IS_FULL = true;
        }
    } 
    else
    {
        flash_address.nextAddress++;
    }

    return (flash_address.nextAddress);
}

/*************************************************************
 * FUNCTION: update_current_address()
 * -----------------------------------------------------------
 * This function goes to the next address that should be
 * written to or read from. If the block currently being
 * updated is at or past the final allowed value based on
 * NUM_BLOCKS, then an error message is thrown. 
 *
 * Parameters: none
 *
 * Returns:
 *      address	: Updated address.
 *************************************************************/
uint32_t update_current_address() {
    /* Check if block out of main array. */
    if(seal_calculate_block_offset(flash_address.currentAddress) >= NUM_BLOCKS)
    {
        /* Make sure there is still more space on the device. */
        if(flash_address.currentChipInUse < (MAX_NUM_CHIPS - 1))
        {
            /* Switch chip and set address pointer. */
            switch_flash_chips();
        }
        else
        {
            /* Device out of space! Set global flag. */
            FLASH_IS_FULL = true;
        }
    }
    else
    {
        flash_address.currentAddress++;
    }

    return (flash_address.currentAddress);
}

/*************************************************************
 * FUNCTION: get_current_address()
 * -----------------------------------------------------------
 * This function returns the value currently stored in the 
 * address descriptor's current address value. 
 *
 * Parameters: none
 *
 * Returns:
 *      addressInfo.currentAddress : Current address value. 
 *************************************************************/
uint32_t get_current_address() {
	return flash_address.currentAddress;
}

/*************************************************************
 * FUNCTION: get_next_address()
 * -----------------------------------------------------------
 * This function returns the value currently stored in the 
 * address descriptor's current address value. 
 *
 * Parameters: none
 *
 * Returns:
 *      addressInfo.currentAddress : Current address value. 
 *************************************************************/
uint32_t get_next_address() {
	return flash_address.nextAddress;
}

/*************************************************************
 * FUNCTION: reset_address_info()
 * -----------------------------------------------------------
 * This function reinitializes the address pointer back to the 
 * beginning of the device. It points back to block one of the 
 * device. Block zero is the superblock and is not addressable
 * by the user. 
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void reset_address_info()
{
    /* Initialize the address descriptor. Initialize block address to block 1 (after the superblock). */
    flash_address.currentAddress   = 0x40;
    flash_address.nextAddress      = 0x40;
    flash_address.currentChipInUse = 0x00;
    
    seal_set_active_chip(0);
}

/*************************************************************
 * FUNCTION: switch_flash_chips()
 * -----------------------------------------------------------
 * This function switches to the next available flash chip and
 * sets the address pointer to the first user-addressable 
 * space.
 *
 * Parameters: none
 *
 * Returns: void
 *************************************************************/
void switch_flash_chips()
{
    /* Switch to next flash chip. */
    flash_address.currentChipInUse++;
    seal_set_active_chip(flash_address.currentChipInUse);
    
    /* Set address pointer to beginning of new flash chip. */
    flash_address.currentAddress   = 0x40;
    flash_address.nextAddress      = 0x40;
}