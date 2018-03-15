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
    char welcome[] = "About to initialize device...\n";
    char message[] = "Before bad block count\n";
    char goodbye[] = "Goodbye!\n";
    char newLine = '\n';
   
    /* VARIABLE DECLARATIONS */
    volatile uint8_t  status;
    volatile uint32_t badBlockCount;
    uint8_t pageData[PAGE_SIZE_EXTRA];
    int retVal;
    int j;
    volatile SUPERBLOCK_t superblock;
   
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    
    /* Wait for USB connection to be made with computer. */
    do { /* NOTHING */ } while (!usb_dtr());
        
    /* Write welcome message to PC console. */
    retVal = usb_write((uint8_t *) welcome, sizeof(welcome));
        
    flash_init();

    /* Read a page from memory. */
    status = flash_read_page((uint8_t *) &firstPage, (uint8_t *) &colAddress, pageData);
    

    /* Write message to computer. */
    do
    {
         retVal = usb_write((uint8_t *) message, 23);
    } while(retVal < 0);         

    /* Iterate over bad block table and print to the computer. */ 
    superblock = *flash_get_superblock();
    
     do
     {
         retVal =usb_put(badBlockCount);
     } while(retVal < 0);
    
    do
    {
        retVal = usb_put(newLine);
    } while(retVal < 0);

    for(j = 0; j < MAX_BAD_BLOCKS; j++)
    {
        do
        {
            retVal = usb_write((uint8_t *) &badBlockTable[j], 4);
        } while(retVal < 0);
            
        do
        {
            retVal = usb_put(newLine);
        } while(retVal < 0);   
    }
    
    /* Write message to computer. */
    do
    {
        retVal = usb_write((uint8_t *) goodbye, sizeof(goodbye));
    } while(retVal < 0);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(200);
    }
}