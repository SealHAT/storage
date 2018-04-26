#include <atmel_start.h>
//#include "Small_NAND_Test.h"
#include "flash_io.h"

#define TEST_DATA_SIZE      (10240)  /* 5 pages (2048 * 5) */
extern uint8_t TEST_DATA[TEST_DATA_SIZE];
static char charBuffer[5];
static char statBuffer[100];

char WELCOME[31]     = "About to initialize device...\n";
char GOODBYE[10]     = "Goodbye!\n";
char START_WRITE[22] = "Device writing begin!\n";
char DONE_WRITE[25]  = "Device writing complete.\n";
char START_READ[22]  = "Begin device reading.\n";
char DONE_READ[25]   = "Device reading complete!\n";
char NEW_LINE        = '\n';

int main(void)
{
    /* Variables */
    uint8_t status;
    uint8_t *dataAr;//[PAGE_SIZE_EXTRA];
    int     i;
    int     retVal;
    int     tempRetVal;
    
    /* Initializes MCU, SPI device, and SPI Flash buffers. */
    atmel_start_init();
    
    /* Set up USB connection */
    /* Wait for USB connection to be made with computer. */
    do { /* NOTHING */ } while (!usb_dtr());
    
    /* Write welcome message to PC console. */
    retVal = usb_write((uint8_t *) WELCOME, sizeof(WELCOME));
    
    /* Initialize the flash interface. */
    flash_init();
    
    do {
        retVal = usb_write((uint8_t *) WELCOME, sizeof(WELCOME));
    } while(retVal != USB_OK );
    
    /* Declare and initialize flash descriptor. */
    static FLASH_DESCRIPTOR flash_descriptor;

    /* Start by erasing the entire device (except the superblock). */
    status = flash_erase_device();
    
    /* Wait until device is done erasing. */
    flash_wait_until_not_busy();
    
    /* Initialize descriptor. */
    flash_io_init(&flash_descriptor, PAGE_SIZE_LESS);
    
    do {
        retVal = usb_write((uint8_t *) START_WRITE, sizeof(START_WRITE));
    } while(retVal != USB_OK );
    
    flash_io_write(&flash_descriptor, TEST_DATA, PAGE_SIZE_LESS);
    
    do {
        retVal = usb_write((uint8_t *) DONE_WRITE, sizeof(DONE_WRITE));
    } while(retVal != USB_OK);
    
    do {
        retVal = usb_write((uint8_t *) START_READ, sizeof(START_READ));
    } while(retVal != USB_OK);
    
    flash_io_flush();
    flash_io_reset_addr();
    
    flash_io_read(&flash_descriptor, dataAr, PAGE_SIZE_LESS);
    
    for(i = 0; i < PAGE_SIZE_LESS; i++)
    {
        /* Wait for USB to not be busy. This prevents charBuffer from being overwritten. */
        while(usb_isInBusy() == true) {/* WAIT */}
        
        /* Place new data in the buffer. */
        snprintf(charBuffer, 5,"%3d\n", dataAr[i]);
        
        /* Try to write the data. If an error occurs, the error value will be printed as well. */
        do {
            retVal = usb_write(charBuffer, sizeof(charBuffer));
            
            /* If there was an error, print the error value. */
            if(retVal != USB_OK) {
                sprintf(statBuffer, "ERROR: %d   DATA: %d", retVal);
                
                do {
                    tempRetVal = usb_write(statBuffer, sizeof(statBuffer));
                } while(tempRetVal != USB_OK);
                
                /* Wait until USB is not busy since the string that was just sent is large. */
                while(usb_isInBusy() == true) {/* WAIT */}
            }
        } while(retVal != USB_OK);
    }
    
    /* Print done message. */
    do {
        retVal = usb_write((uint8_t *) DONE_READ, sizeof(DONE_READ));
    } while(retVal != USB_OK);
    
    /* Toggle LED on/off forever. */
    while(1)
    {
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(1000);
    }
}