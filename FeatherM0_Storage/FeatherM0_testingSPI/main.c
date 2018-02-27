#include <atmel_start.h>

#define BUFFER_SIZE (1024)                                 //maximum input/output buffer transfer size for SPI (I love parentheses)

int main(void)
{
    /* CONSTANT DECLARATIONS */ 
    const uint8_t RESET[1]      = {0xFF};                  //Command to reset the memory device
    const uint8_t GET_FEAT[2]   = {0x0F, 0xC0};            //Command to get the current contents of the status register
    const uint8_t SET_WEL[1]    = {0x06};                  //Command to set the write enable bit in in the status register
    const uint8_t PROG_LOAD[3]  = {0x02, 0x00, 0x00};      //Command to load data from micro into cache of memory device. Last 2 bytes page column address
    const uint8_t PEXEC[4]      = {0x10, 0x00, 0x00, 0x00};//Command to program the main memory array from the memory devices cache register
        
    const uint8_t PAGE_READ[4]  = {0x13, 0x00, 0x00, 0x10};//Command to read data from main array into data cache
    const uint8_t READ_CACHE[3] = {0x03, 0x00, 0x00};      //Command to read data from memory device cache to SPI buffer
        
    const uint8_t USER_DATA[4]  = {0xDE, 0xAD, 0xBE, 0xEF};//Because why wouldn't this be the test data
    
    /* VARIABLE DECLARATIONS */
    const    uint8_t BUSY_MASK   = 0x01;                   //mask for checking if the flash is busy
    const    uint8_t PROG_FAIL   = 0b00001000;             //mask for checking if the memory was programmed successfully
    const    uint8_t WEL_BIT     = 0x02;                   //mask for checking if write enable is high
    volatile uint32_t spi_result = 0;                      //how many bytes were transfered during SPI communication. -1 if error. Just used while debugging
    uint8_t  MOSI[BUFFER_SIZE];                            //master's output buffer 
    uint8_t  MISO[BUFFER_SIZE];                            //master's input buffer
    struct   spi_xfer spi_buff;                            //SPI transfer descriptor
    int      i;                                            //loop control variable
    
    /*******************
     * INITIALIZATIONS *
     *******************/ 
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();    
    
    /* Initialize input and output buffers */
    for(i = 0; i < BUFFER_SIZE; i++)
    { 
        MISO[i] = 0xFF;       
        MOSI[i] = 0x00;  
    }

    /* Associate flash buffers with SPI device and set
     * buffer size. */
    spi_buff.size  = BUFFER_SIZE;
    spi_buff.rxbuf = MISO;
    spi_buff.txbuf = MOSI;

    /* Setup SPI IO */
	/* Set clock mode and enable SPI */
	spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
	spi_m_sync_enable(&SPI_0);
    
    /* Allow time for initial device setup before sending
     * reset signal. Minimum time is 250us. */
    delay_ms(1);
    
    /**********************
     * RESET SLAVE DEVICE *
     **********************/ 
    /* Issue the RESET command: 0xFF. Device reset on startup
     * is not mandatory for this part, but is recommended. This
     * also ensures previous run settings are reset within the 
     * device. */
    MOSI[0] = RESET[0];

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);

    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Minimum 1.25ms delay after reset command before any
     * other commands can be issued. Rounded up to 2ms. */
    delay_ms(200);
    
    /*************************
     * CHECK STATUS REGISTER *
     *************************/ 
    /* Issue the GET FEATURES command: 0x0F 
     * Send address of FEATURES register: 0xC0
     * Put command in MOSI buffer.  */
    MOSI[0] = GET_FEAT[0];
    MOSI[1] = GET_FEAT[1];

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);

    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);

    /********************
     * SET WRITE ENABLE *
     ********************/ 
    /* Issue the SET WEL BIT command to enable writing: 0x06
     * Issue GET FEATURES command and register address: 0x0FC0 */
    MOSI[0] = SET_WEL[0];
    MOSI[1] = 0;
	
    /* In order for the chip to recognize the SET WEL BIT command, 
     * ONLY one byte of data can be sent over the SPI connection
     * containing the command. The status is checked immediately
     * after in a different SPI transaction. */
    delay_ms(500);
	spi_buff.size = 1;
	
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);

    /*************************
     * CHECK STATUS REGISTER *
     *************************/ 
    /* Send GET FEATURES command to check the status of the 
     * WEL bit. Also, reset the SPI buffer to its initial
     * size. */
    delay_ms(500);
    MOSI[0] = GET_FEAT[0];
    MOSI[1] = GET_FEAT[1];
	spi_buff.size  = BUFFER_SIZE;

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
        
    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
        
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);

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
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
    
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