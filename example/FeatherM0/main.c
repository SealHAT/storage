#include <atmel_start.h>
#include "Small_NAND_Test.h"

int main(void)
{   
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    
    /* Run flash test. */
    small_nand_test_driver();
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(1000);
    }
}