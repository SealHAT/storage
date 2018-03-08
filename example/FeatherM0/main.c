#include <atmel_start.h>
#include "NAND_Flash.h"

int main(void)
{
    /* CONSTANT DECLARATIONS */ 
    uint8_t  USER_DATA[4]     = {0xDE, 0xAD, 0xBE, 0xEF};     //Because why wouldn't this be the test data
    uint32_t colAddress       = 0x0000;                       //Column address for in-page offset
    uint32_t pageBlockAddress = 0x002000;                     //Block and page address. Left 18 bits block address, right 6 bits page address
   
    /* VARIABLE DECLARATIONS */
    volatile uint8_t  status;
    //volatile uint32_t badBlockCount;
    uint8_t pageData[PAGE_SIZE_EXTRA];
     
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    flash_initSPI();
    flash_init_buffers();
    
    /* Allow time for memory device initial setup (minimum power-up time)
     * before sending reset signal. Minimum time is 250us. */
    delay_ms(1);
    
    /* Reset slave device (memory chip). */ 
    status = flash_reset();     //status should be 0x01 if busy and 0x00 when done
    
    /* Minimum 1.25ms delay after reset command before any
     * other commands can be issued. Rounded up to 2ms. */
    delay_ms(200);
    
    /* Build a bad block table. (Just count bad blocks for now) 
     * Very slow with debugger. */
    //badBlockCount = BuildBadBlockTable();

    /* Unlock all blocks (locked by default at power up). Returns status of the
     * block lock register. If the blocks are already unlocked, the unlock 
     * command will not be sent. */
    status = flash_block_lock_status();
    
    if(status > 0)
    {
        status = flash_unlock_all_blocks();
    }        

    /* Read a page from memory. */
    status = flash_read_page((uint8_t *) &pageBlockAddress, (uint8_t *) &colAddress, pageData);
    
    /* Erase a block. (Don't do this yet, wait until bad block table code is established) */
    //status = flash_BlockErase(pageBlockAddress);
    
    /* Check status register. */
    status = flash_status();    //status should be zero if flash is not busy

    /* Set Write Enable bit. */ 
    status = flash_set_WEL();    //status should be 0x02 if WEL was set

    /* Write a page to memory. */ 
    status = flash_write_page(USER_DATA, 4, (uint8_t *) &colAddress, (uint8_t *) &pageBlockAddress); 
    
     /* Next, check the status register for failures. */
    delay_ms(5);
    status = flash_status();

    /* Read a page from memory. */
    status = flash_read_page((uint8_t *) &pageBlockAddress, (uint8_t *) &colAddress, pageData);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(200);
    }
}