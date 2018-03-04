#include <atmel_start.h>
#include "NAND_Flash.h"

int main(void)
{
    /* CONSTANT DECLARATIONS */ 
    const uint8_t USER_DATA[4]        = {0xDE, 0xAD, 0xBE, 0xEF};   //Because why wouldn't this be the test data
    const uint8_t colAddress[2]       = {0x00, 0x00};               //Column address for in-page offset
    const uint8_t pageBlockAddress[3] = {0x00, 0x10, 0x00};         //Block and page address. Left 18 bits block address, right 6 bits page address

    /* VARIABLE DECLARATIONS */
    volatile uint8_t status;
    uint8_t pageData[PAGE_SIZE];
     
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    flash_InitSPI();
    flash_InitBuffers();
    
    /* Allow time for memory device initial setup (minimum power-up time)
     * before sending reset signal. Minimum time is 250us. */
    delay_ms(1);
    
    /* Reset slave device (memory chip). */ 
    status = flash_Reset();     //status should be 0x01 if busy and 0x00 when done
    
    /* Minimum 1.25ms delay after reset command before any
     * other commands can be issued. Rounded up to 2ms. */
    delay_ms(200);
    
    /* Unlock all blocks (locked by default at power up). Returns status of the
     * block lock register. */
    status = flash_UnlockAllBlocks();
    
    status = flash_BlockLockStatus();
    
    /* Read a page from memory. */
    status = flash_ReadPage(pageBlockAddress, colAddress, pageData);
    
    //status = flash_BlockErase(pageBlockAddress);
    
    /* Check status register. */
    status = flash_Status();    //status should be zero if flash is not busy

    /* Set Write Enable bit. */ 
    status = flash_SetWEL();    //status should be 0x02 if WEL was set

    /* Check status register. */
    status = flash_Status();    //status should be 0x02 if WEL was set

    /* Write a page to memory. */ 
    status = flash_WritePage(USER_DATA, 4, colAddress, pageBlockAddress); 
    
     /* Next, check the status register for failures. */
    delay_ms(500);
    status = flash_Status();

    /* Read a page from memory. */
    status = flash_ReadPage(pageBlockAddress, colAddress, pageData);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED0);
        delay_ms(200);
    }
}