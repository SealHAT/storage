#include <atmel_start.h>

#define BUFFER_SIZE (50)                          //maximum input/output buffer transfer size for SPI

int main(void)
{
    /* VARIABLE DECLARATIONS */
    const    uint8_t BUSY_MASK      = 0x01;         //mask for checking if the flash is busy
    const    uint8_t PROG_FAIL      = 0b00001000;   //mask for checking if the memory was programmed successfully
    const    uint8_t WEL_BIT        = 0x02;         //mask for checking if write enable is high
    volatile uint32_t spi_result    = 0;            //how many bytes were transfered during SPI communication. -1 if error
    volatile uint8_t  MOSI[BUFFER_SIZE];            //master's output buffer 
    volatile uint8_t  MISO[BUFFER_SIZE];            //master's input buffer
    struct   spi_xfer spi_buff;                     //SPI transfer descriptor
    int      i;                                     //loop control variable
    
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

    /* Associate flash buffers with SPI device */
    spi_buff.size  = BUFFER_SIZE;
    spi_buff.rxbuf = MISO;
    spi_buff.txbuf = MOSI;

    /* Setup SPI IO */
	/* Set clock mode and enable SPI */
	spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
	spi_m_sync_enable(&SPI_0);
    
    /* Allow time for initial device setup before sending
     * reset signal. Minimum time is 250us*/
    delay_ms(1);

    /* Issue the RESET command: 0xFF. 
     * Put command in MOSI buffer.  */
    uint8_t RESET[1] = {0xFF};
    MOSI[0] = RESET[0];

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);

    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Minimum 1.25ms delay after reset command before any
     * other commands can be issued. Rounded up to 2ms. */
    //delay_ms(200);
    
    /* Issue the GET FEATURES command: 0x0F 
     * Send address of FEATURES register: 0xC0
     * Put command in MOSI buffer.  */
    uint8_t GET_FEAT[2] = {0x0F, 0xC0};
    MOSI[0] = GET_FEAT[0];
    MOSI[1] = GET_FEAT[1];

    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);

    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Waiting to see if this helps before issuing next command... 
     * Sending GET FEATURES message again. */
    delay_ms(500);
     
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);

    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);

    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);
    
    /* Issue the SET WEL BIT command to enable writing: 0x06
     * Issue GET FEATURES command and register address: 0x0FC0 */
    uint8_t SET_WEL_GET_FEAT[3] = {0x06, 0x0F, 0xC0};
    MOSI[0] = SET_WEL_GET_FEAT[0];
    MOSI[1] = 0;
	
	spi_buff.size  = 1;
	
	/* Initialize input and output buffers */
    for(i = 0; i < BUFFER_SIZE; i++)
    { 
        MISO[i] = 0xFF;
    }
    
	delay_ms(500);
		
    /* Set slave select (CS) active low to communicate. */
    gpio_set_pin_level(GPIO_PIN(CS),false);
    
    /* Read/write over SPI */
    spi_result = spi_m_sync_transfer(&SPI_0, &spi_buff);
    
    /* Read/write over SPI */
    gpio_set_pin_level(GPIO_PIN(CS),true);

    /* Send GET FEATURES command again without setting the WEL bit */
    MOSI[0] = 0;
	
	delay_ms(500);
	MOSI[0] = SET_WEL_GET_FEAT[1];
	MOSI[1] = SET_WEL_GET_FEAT[2];
	spi_buff.size  = BUFFER_SIZE;
	
	/* Initialize input and output buffers */
	//for(i = 0; i < BUFFER_SIZE; i++)
	//{
	//	MISO[i] = 0xFF;
	//}

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