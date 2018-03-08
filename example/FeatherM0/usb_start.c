/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#include "atmel_start.h"
#include "usb_start.h"
#include "ringbuff/ringbuffer.h"

#define USB_BUFFER_SIZE CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ		/* Define buffer size as endpoint size */
static uint8_t single_desc_bytes[] = { CDCD_ACM_DESCES_LS_FS };		/* Device descriptors and Configuration descriptors list. */
static struct usbd_descriptors single_desc[] = { {single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes) } };	/* Make a struct of the needed descriptors */

static uint8_t ctrl_buffer[64];					/* Ctrl endpoint buffer */
static uint8_t tx_buff[USB_BUFFER_SIZE];		/* The TX buffer for sending bytes */
volatile static uint8_t tx_idx = 0;				/* index to track the position in the TX queue*/
static ring_buffer_t rx_buff;					/* Ring buffer queue */

volatile static bool inComplete  = false;		/* flag to indicate an IN transfer has occurred */
volatile static bool outComplete = false;		/* flag to indicate an OUT transfer has occurred */
volatile static bool dtr_flag    = false;		/* Flag to indicate status of DTR - Data Terminal Ready */
volatile static bool rts_flag    = false;		/* Flag to indicate status of RTS - Request to Send */

/**
 * \brief Callback for USB to simply set a flag that data has been received.
 */
static bool usb_out_complete(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
//	static uint8_t rx_temp_buf[USB_BUFFER_SIZE];
//	
//	if (count <= USB_BUFFER_SIZE && (RING_BUFFER_SIZE - ring_buffer_num_items(&rx_buff) - 1 == 0) ) {	
//		cdcdf_acm_read(rx_temp_buf, count);
//		ring_buffer_queue_arr(&rx_buff, rx_temp_buf, count);
//		
//		return false;
//	}
//	return true;
	
	outComplete = 1;
	/* No error returns false ... probably because false is 0? */
	return false;	
}

/**
 * \brief Callback for USB to simply set a flag that data has been sent successfully.
 */
static bool usb_in_complete(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	inComplete = 1;
	/* No error returns false ... probably because false is 0? */
	return false;
}

/**
 * \brief Callback invoked when Line State Change
 *
 * this function is called when there is a change to the RTS and DTS control
 * lines on the serial connection. 
 *
 */
static bool usb_line_state_changed(usb_cdc_control_signal_t newState)
{
	static bool callbacks_registered = false;
	
	dtr_flag = newState.rs232.DTR;
	rts_flag = newState.rs232.RTS;
			
	if (cdcdf_acm_is_enabled() && !callbacks_registered) {
		callbacks_registered = true;
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_out_complete);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_in_complete);
	}

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
static void cdc_device_acm_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

void usb_init(void)
{
	cdc_device_acm_init();
	ring_buffer_init(&rx_buff);
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_line_state_changed);
}

// TODO - make this non-blocking
void usb_flush(void)
{
	if(tx_idx > 0)
	{
		inComplete = 0;
		cdcdf_acm_write(tx_buff, tx_idx);
		
		/* Block until data is sent and then reset index */
		while(inComplete == 0);
		tx_idx = 0;
	}
}

void usb_put(uint8_t outChar)
{
	tx_buff[tx_idx++] = outChar;
	
	/* If the buffer is full, flush it */
	if(tx_idx == USB_BUFFER_SIZE) {
		usb_flush();
	}
}

/***************************************************************************
 * FUNCTION: usb_send_buffer()
 * -------------------------------------------------------------------------
 * Send a buffer of data over USB. The data buffer can be of any length.
 * If user's output buffer size is larger than USB buffer, only the max USB
 * buffer size amount of data will be written into the buffer, flushed, then
 * the user's buffer data will begin filling the USB out buffer again.
 * 
 * INPUTS:
 *      outData         :   buffer of data to output over USB
 *      BUFFER_SIZE     :   size of buffer to transfer
 *
 * OUTPUTS:
 *      void
 ***************************************************************************/
// TODO - make non-blocking
void usb_send_buffer(uint8_t outData[], int BUFFER_SIZE)
{
    int i = 0;  // Output data buffer index
    
    while(i < BUFFER_SIZE)
    {
        while((i < BUFFER_SIZE) && (tx_idx != USB_BUFFER_SIZE))
        {
            usb_put(outData[i]);
            i++;
        }

		usb_flush();
    }        
    
}

// TODO make this non-blocking
int usb_get(void)
{
//	char retval;
//	if(ring_buffer_dequeue(&rx_buff, &retval)) {
//		return retval;
//	}
//	return -1;
	
	int retval;
	uint8_t inChar;
	outComplete = 0;
	retval = cdcdf_acm_read(&inChar, 1);
	while(outComplete == 0); // block until data read
	return (retval < 0 ? retval : inChar);
}

/***************************************************************************
 * FUNCTION: usb_receive_buffer()
 * -------------------------------------------------------------------------
 * Receive a buffer of data over USB. Input data buffer can be of any size. 
 * If user's input buffer size is larger than USB buffer, only the max USB
 * buffer size amount of data will be written into the buffer.
 * 
 * INPUTS:
 *      receiveBuffer   :   buffer to store incoming USB data
 *      BUFFER_SIZE     :   size of receiving buffer
 *
 * OUTPUTS:
 *      retval          :   returns the number of bytes received
 ***************************************************************************/
// TODO - make non-blocking
int usb_receive_buffer(uint8_t receiveBuffer[], int BUFFER_SIZE)
{
    int bufferSize;     //adjusted buffer size
	int letter;         //value received from USB buffer
    int i;              //LCV and # of bytes received
	
	/* Users input buffer size doesn't matter. If size is bigger, only indices 
	 * 0 through USB_BUFFER_SIZE - 1 will be filled. If the input array is 
	 * smaller, only up until BUFFER_SIZE will be filled. */
	if(BUFFER_SIZE < USB_BUFFER_SIZE)
	{
		bufferSize = BUFFER_SIZE;
	}
	else
	{
		bufferSize = USB_BUFFER_SIZE;
	}
	
	i = 0;
	
    /* Get data from USB buffer. Only a maximum of USB_BUFFER_SIZE values will
     * be read into the buffer. */
	while(i < bufferSize)
	{
        letter = usb_get();
        
        if(letter > 0) 
        {
            receiveBuffer[i] = letter;
            i++;
        }
    }        

    /* Make sure all data is read before returning */
	while(outComplete == 0); // block until data read
    
	/* If there was an error, return error val. Else, return 0. */
	return i;
}

bool usb_configured(void) {
	return usbdc_get_state() == USBD_S_CONFIG;
}

bool usb_dtr(void) {
	return dtr_flag;
}

bool usb_rts(void) {
	return rts_flag;
}