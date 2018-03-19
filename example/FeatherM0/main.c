#include <atmel_start.h>
#include "NAND_Test.h"

int main(void)
{   
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    
    /* Run flash test. */
    nand_test_driver();
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(200);
    }
}