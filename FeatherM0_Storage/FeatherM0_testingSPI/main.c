#include <atmel_start.h>
#include "NAND_Flash.h"

int main(void)
{
    /* CONSTANT DECLARATIONS */ 
    const uint8_t USER_DATA[4]  = {0xDE, 0xAD, 0xBE, 0xEF}; //Because why wouldn't this be the test data

    /* VARIABLE DECLARATIONS */
    volatile uint8_t status;
    
    /*******************
     * INITIALIZATIONS *
     *******************/ 
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();    

    /* Initialize SPI device and sets up buffers */
    flash_InitSPI();

    /* Initialize input and output buffers */
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
    
    /*************************
     * CHECK STATUS REGISTER *
     *************************/ 
    status = flash_Status();    //status should be zero if flash is not busy

    /********************
     * SET WRITE ENABLE *
     ********************/ 
    status = flash_SetWEL();    //status should be 0x02 if WEL was set

    /*************************
     * CHECK STATUS REGISTER *
     *************************/ 
    status = flash_Status();    //status should be 0x02 if WEL was set

    /*******************
     * WRITE TO MEMORY *
     *******************/ 
    /* First, load the data into the cache register. Pull chip 
     * select high as soon as the last user data byte has been
     * transmitted. Transmitting full page amount of data.*/
    MOSI[0] = PROG_LOAD[0];
    MOSI[1] = PROG_LOAD[1];
    MOSI[2] = PROG_LOAD[2];
    MOSI[3] = USER_DATA[0];
    MOSI[4] = USER_DATA[1];
    MOSI[5] = USER_DATA[2];
    MOSI[6] = USER_DATA[3];
    
    spi_buff.size = BUFFER_SIZE;
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
     /* Next, check the status register for failures. */
    delay_ms(500);
    MOSI[0] = GET_FEAT[0];
    MOSI[1] = GET_FEAT[1];
    MOSI[2] = 0;
    MOSI[3] = 0;
    MOSI[4] = 0;
    MOSI[5] = 0;
    MOSI[6] = 0;
    
    spi_buff.size = 50;

    /* Next, write the contents of the cache register into the main
     * memory array. Pull chip select high as soon as address is done
     * transmitting. */
    MOSI[0] = PEXEC[0];
    MOSI[1] = PEXEC[1];
    MOSI[2] = PEXEC[2];
    MOSI[3] = PEXEC[3];
    
    spi_buff.size = 4;
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
        
    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
        
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Check status register again. */
    MOSI[0] = GET_FEAT[0];
    MOSI[1] = GET_FEAT[1];
    MOSI[2] = 0;
    MOSI[3] = 0;
     
    spi_buff.size = 3;
     
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
     
    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
     
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);

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
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS), true);
    
    /* Check status to make sure device is not still busy */
     delay_ms(500);
     MOSI[0] = GET_FEAT[0];
     MOSI[1] = GET_FEAT[1];
     MOSI[2] = 0;
     MOSI[3] = 0;
     
     /* 50 is arbitrary. Minimum size to see result would be 3 */
     spi_buff.size = 50;

     /* Set slave select (CS) active low to communicate. */
     gpio_set_pin_level(GPIO_PIN(CS),false);
     
     /* Read/write over SPI */
     spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
     
     /* Read/write over SPI */
     gpio_set_pin_level(GPIO_PIN(CS),true);

    /* Read the data from the cache register to the SPI
     * MISO line. */
    MOSI[0] = READ_CACHE[0];
    MOSI[1] = READ_CACHE[1];
    MOSI[2] = READ_CACHE[2];
    
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Toggle LED on/off forever. */
    while(1) 
    {
        gpio_toggle_pin_level(LED0);
        delay_ms(200);
    }
}