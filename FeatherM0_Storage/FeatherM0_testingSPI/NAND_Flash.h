/*
 * NAND_Flash.h
 *
 * Created: 2/23/2018 6:34:30 PM
 *  Author: kmcarrin
 */ 


#ifndef NAND_FLASH_H_
#define NAND_FLASH_H_

#include <hal_spi_m_sync.h>
#include <hal_delay.h>

/* DEFINES */
#define BUFFER_SIZE (50)                            //maximum input/output buffer transfer size for SPI

/* CONSTANT DECLARATIONS */
const uint8_t RESET[1]      = {0xFF};               //Command to reset the memory device
const uint8_t GET_FEAT[2]   = {0x0F, 0xC0};         //Command to get the current contents of the status register
const uint8_t SET_WEL[1]    = {0x06};               //Command to set the write enable bit in in the status register
const uint8_t BUSY_MASK     = 0x01;                 //Mask for checking if the flash is busy
const uint8_t PROG_FAIL     = 0b00001000;           //Mask for checking if the memory was programmed successfully
const uint8_t WEL_BIT       = 0x02;                 //Mask for checking if write enable is high


uint8_t  MOSI[BUFFER_SIZE];                     //master's output buffer
uint8_t  MISO[BUFFER_SIZE];                     //master's input buffer
struct   spi_xfer spi_buff;                     //SPI transfer descriptor



#endif /* NAND_FLASH_H_ */