/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#ifndef USB_DEVICE_MAIN_H
#define USB_DEVICE_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"

/**
 * \brief Initialize USB
 */
void usb_init(void);

/**
 * \brief Returns true if the USB is configured (done with enumeration and not asleep)
 */
bool usb_configured(void);

/**
 * \brief Returns true if DTR signal is high
 */
bool usb_dtr(void);

/**
 * \brief Returns true if RTS signal is high
 */
bool usb_rts(void);

/**
 * \brief flush the TX buffer and send it's contents to the USB host. This functions blocks.
 */
void usb_flush(void);

/**
 * \brief Initialize USB
 */
void usb_put(uint8_t outChar);

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
void usb_send_buffer(uint8_t outData[], int BUFFER_SIZE);

/**
 * \brief Initialize USB
 *
 * \return less than 0 is an error return from usb_d_ep_transfer()
 */
int usb_get(void);

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
int usb_receive_buffer(uint8_t receiveBuffer[], int BUFFER_SIZE);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // USB_DEVICE_MAIN_H
