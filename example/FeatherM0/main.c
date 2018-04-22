#include <atmel_start.h>
//#include "Small_NAND_Test.h"
#include "flash_io.h"

extern uint8_t TEST_DATA[TEST_DATA_SIZE];

int main(void)
{   
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    
    /* Initialize the flash interface. */
    flash_init();
    
    /* Declare and initialize flash descriptor. */
    FLASH_DESCRIPTOR flash_descriptor;
    
    /* Variables */
    uint8_t status;
    
    /* Start by erasing the entire device (except the superblock). */
    status = flash_erase_device();
    
    /* Wait until device is done erasing. */
    flash_wait_until_not_busy();
    
    /* Initialize descriptor. */
    flash_io_init(flash_descriptor, PAGE_SIZE_LESS);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(1000);
    }
}