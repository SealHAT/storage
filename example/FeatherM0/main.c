#include <atmel_start.h>
#include "NAND_Flash.h"

int main(void)
{
    /* CONSTANT DECLARATIONS */ 
    uint8_t  USER_DATA[4]     = {0xDE, 0xAD, 0xBE, 0xEF};     //Because why wouldn't this be the test data
    uint32_t colAddress       = 0x0000;                       //Column address for in-page offset
    uint32_t pageBlockAddress = 0x002000;                     //Block and page address. Left 18 bits block address, right 6 bits page address
    uint32_t testBlockAddress = 0x002000;
    uint32_t firstPage = 0x000000;
   
    /* VARIABLE DECLARATIONS */
    volatile uint8_t  status;
    volatile uint32_t badBlockCount;
    uint8_t pageData[PAGE_SIZE_EXTRA];
       
     
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    flash_init();

    /* Read a page from memory. */
    status = flash_read_page((uint8_t *) &firstPage, (uint8_t *) &colAddress, pageData);
    
    char message[] = "Before bad block count\n";
    delay_ms(5000);
    usb_send_buffer((uint8_t *) message, 23);
    
    int j;
    char newLine = '\n';
    
    for(j = 0; j < MAX_BAD_BLOCKS; j++)
    {
        usb_send_buffer((uint8_t *) &badBlockTable[j], 4);
        usb_put(newLine);
    }
    
    /* Build a bad block table. (Just count bad blocks for now) 
     * Very slow with debugger. */
    badBlockCount = build_bad_block_table();

    usb_put(badBlockCount);
    usb_put(newLine);

    for(j = 0; j < MAX_BAD_BLOCKS; j++)
    {
        usb_send_buffer((uint8_t *) &badBlockTable[j], 4);
        usb_put(newLine);
    }

    /* Read a page from memory. */
    status = flash_read_page((uint8_t *) &pageBlockAddress, (uint8_t *) &colAddress, pageData);
    
    /* Erase a block. (Don't do this yet, wait until bad block table code is established) */
    //status = flash_block_erase(pageBlockAddress);
    
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