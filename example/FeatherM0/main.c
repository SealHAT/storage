#include <atmel_start.h>
//#include "Small_NAND_Test.h"
#include "flash_io.h"

extern uint8_t TEST_DATA[TEST_DATA_SIZE];

int main(void)
{   
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    
    /* Declare and initialize flash descriptor. */
    FLASH_DESCRIPTOR flash_descriptor;
    
    /* Initialize descriptor. */
    flash_io_init(flash_descriptor, PAGE_SIZE_LESS);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(1000);
    }
}