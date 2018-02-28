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

#include "driver_init.h"

/* DEFINES */
#define PAGE_SIZE   (2176)          //maximum NAND Flash page size (including extra space)
#define BUFFER_SIZE (2180)          //max SPI buffer size (page size plus room for commands)

/* CONSTANT DECLARATIONS */
extern const uint8_t RESET[1];      //Command to reset the memory device
extern const uint8_t GET_FEAT[2];   //Command to get the current contents of the status register
extern const uint8_t SET_WEL[1];    //Command to set the write enable bit in in the status register
extern const uint8_t PROG_LOAD[3];  //Command to load data from micro into cache of memory device. Last 2 bytes page column address
extern const uint8_t PEXEC[4];      //Command to program the main memory array from the memory devices cache register
extern const uint8_t PAGE_READ[4];  //Command to read data from main array into data cache
extern const uint8_t READ_CACHE[3]; //Command to read data from memory device cache to SPI buffer

/* MASKS */
extern const uint8_t BUSY_MASK;     //Mask for checking if the flash is busy
extern const uint8_t PROG_FAIL;     //Mask for checking if the memory was programmed successfully
extern const uint8_t WEL_BIT;       //Mask for checking if write enable is high

/* SPI COMMUNICATION BUFFERS */
uint8_t  MOSI[BUFFER_SIZE];         //master's output buffer
uint8_t  MISO[BUFFER_SIZE];         //master's input buffer
struct   spi_xfer spi_buff;         //SPI transfer descriptor

/* FUNCTION DECLARATIONS */

void flash_InitSPI();
void flash_InitBuffers();
void flash_SetSPI_BufferSize(int newSize);
uint8_t flash_Reset(); // returns status register value
uint8_t flash_Status(); //returns status register value
uint8_t flash_SetWEL(); //returns status register value
uint8_t flash_WritePage(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[]); //returns status register value. col addr must be 2 bytes, pageblock 3 bytes
void flash_WaitUntilNotBusy();

/* INTERNAL FUNCTIONS - NOT A PART OF API */
uint8_t ProgramCache(uint8_t data[], int dataSize, uint8_t colAddress[]);
uint8_t ExecuteProgram(uint8_t pageBlockAddress[]);
void ReinitializeOutBuff();

#endif /* NAND_FLASH_H_ */