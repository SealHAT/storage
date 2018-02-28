#include <atmel_start.h>
#include "NAND_Flash.h"

int main(void)
{
    /* CONSTANT DECLARATIONS */ 
    const uint8_t USER_DATA[4]        = {0xDE, 0xAD, 0xBE, 0xEF};   //Because why wouldn't this be the test data
    const uint8_t colAddress[2]       = {0x00, 0x00};               //Column address for in-page offset
    const uint8_t pageBlockAddress[3] = {0x00, 0x00, 0x80};         //Block and page address. Left 18 bits block address, right 6 bits page address

    /* VARIABLE DECLARATIONS */
    volatile uint8_t status;
    
    /*******************
     * INITIALIZATIONS *
     *******************/ 
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    flash_InitSPI();
    flash_InitBuffers();
    
    /* Allow time for memory device initial setup (minimum power-up time)
     * before sending reset signal. Minimum time is 250us. */
    delay_ms(1);
    
    /**********************
     * RESET SLAVE DEVICE *
     **********************/ 
    status = flash_Reset();     //status should be 0x01 if busy and 0x00 when done
    
    /* Minimum 1.25ms delay after reset command before any
     * other commands can be issued. Rounded up to 2ms. */
    delay_ms(200);
    
    /* Check status register. */
    status = flash_Status();    //status should be zero if flash is not busy

    /********************
     * SET WRITE ENABLE *
     ********************/ 
    status = flash_SetWEL();    //status should be 0x02 if WEL was set

    /* Check status register. */
    status = flash_Status();    //status should be 0x02 if WEL was set

    /*******************
     * WRITE TO MEMORY *
     *******************/ 
    status = flash_WritePage(USER_DATA, 4, colAddress, pageBlockAddress); 
    
     /* Next, check the status register for failures. */
    delay_ms(500);
    status = flash_Status();

    /********************
     * READ FROM MEMORY *
     ********************/ 
    /* First, read a page from the main  memory array into the 
     * data cache. */

    MOSI[0] = PAGE_READ[0];
    MOSI[1] = PAGE_READ[1];
    MOSI[2] = PAGE_READ[2];
    MOSI[3] = PAGE_READ[3];
    
    spi_buff.size = BUFFER_SIZE;
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS), true);
    
    /* Check status to make sure device is not still busy */
     delay_ms(500);
    status = flash_Status();

    /* Read the data from the cache register to the SPI
     * MISO line. */
    MOSI[0] = READ_CACHE[0];
    MOSI[1] = READ_CACHE[1];
    MOSI[2] = READ_CACHE[2];
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED0);
        delay_ms(200);
    }
}