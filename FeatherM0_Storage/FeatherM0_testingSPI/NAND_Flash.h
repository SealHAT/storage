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
#define PAGE_SIZE   (2176)              //maximum NAND Flash page size (including extra space)
#define BUFFER_SIZE (2180)              //max SPI buffer size (page size plus room for commands)

/* CONSTANT DECLARATIONS */
extern const uint8_t RESET[1];          //Command to reset the memory device
extern const uint8_t GET_FEAT[2];       //Command to get the current contents of the status register
extern const uint8_t SET_WEL[1];        //Command to set the write enable bit in in the status register
extern const uint8_t PROG_LOAD[3];      //Command to load data from micro into cache of memory device. Last 2 bytes page column address
extern const uint8_t PEXEC[4];          //Command to program the main memory array from the memory devices cache register
extern const uint8_t PAGE_READ[4];      //Command to read data from main array into data cache
extern const uint8_t READ_CACHE[3];     //Command to read data from memory device cache to SPI buffer
extern const uint8_t ERASE[1];          //Command to erase a block of data
extern const uint8_t GET_BLOCK_LOCK[2]; //Command to check the block lock status
extern const uint8_t UNLOCK_BLOCKS[3];  //Command to check the block lock status

/* MASKS */
extern const uint8_t BUSY_MASK;         //Mask for checking if the flash is busy
extern const uint8_t PROG_FAIL;         //Mask for checking if the memory was programmed successfully
extern const uint8_t WEL_MASK;          //Mask for checking if write enable is high

/* SPI COMMUNICATION BUFFERS */
uint8_t  MOSI[BUFFER_SIZE];             //master's output buffer
uint8_t  MISO[BUFFER_SIZE];             //master's input buffer
struct   spi_xfer spi_buff;             //SPI transfer descriptor

/* FUNCTION DECLARATIONS */
/*************************************************************
 * FUNCTION: flash_InitSPI()
 * -----------------------------------------------------------
 * This function initializes the SPI communication that will
 * be used to communicate with the memory device. It sets the
 * input and output buffers as well as the SPI buffer size. 
 * In addition, it sets the SPI mode to Mode 0 and enables
 * the SPI device. 
 *************************************************************/
void flash_InitSPI();

/*************************************************************
 * FUNCTION: flash_InitBuffers()
 * -----------------------------------------------------------
 * This function initializes the buffers that will be used for
 * SPI communication. The input buffer is initialized to all
 * 0xFF values. The output buffer, on the other hand, is 
 * initialized to all zeros. 
 *************************************************************/
void flash_InitBuffers();

/*************************************************************
 * FUNCTION: flash_BufferSize()
 * -----------------------------------------------------------
 * This function is a utility function used to set the size
 * of the SPI buffer.
 *************************************************************/
void flash_SetSPI_BufferSize(int newSize);

/*************************************************************
 * FUNCTION: flash_Reset()
 * -----------------------------------------------------------
 * This function issues the RESET command: 0xFF. The command 
 * is placed in the output buffer and then sent over the SPI
 * connection only if the device is not currently busy.
 *************************************************************/
uint8_t flash_Reset();

/*************************************************************
 * FUNCTION: flash_Status()
 * -----------------------------------------------------------
 * Issue the GET FEATURES command: 0x0F 
 * Send address of FEATURES register: 0xC0
 * Put command in MOSI buffer. 
 *************************************************************/
uint8_t flash_Status();

/*************************************************************
 * FUNCTION: flash_SetWEL()
 * -----------------------------------------------------------
 * This function issues the Set Write Enable Latch command to
 * the memory device. The command is placed into the output 
 * buffer and sent to the device. 
 *************************************************************/
uint8_t flash_SetWEL();

/*************************************************************
 * FUNCTION: flash_WritePage()
 * -----------------------------------------------------------
 * This function writes a page of data to the Flash device.
 * First, the write enable flag (WEL bit) is set. Next, the
 * device's cache is loaded with the given data. Once that
 * Operation is complete, the data is written from the
 * device's cache into its main memory array.
 *************************************************************/
uint8_t flash_WritePage(uint8_t data[], int dataSize, uint8_t colAddress[], uint8_t pageBlockAddress[]);

/*************************************************************
 * FUNCTION: flash_WaitUntilNotBusy()
 * -----------------------------------------------------------
 * This function continuously checks the status register. The
 * function returns once the status register shows that the 
 * device is no longer busy.
 *************************************************************/
void flash_WaitUntilNotBusy();

/*************************************************************
 * FUNCTION: flash_ReadPage()
 * -----------------------------------------------------------
 * This function reads a page of data from the Flash device.
 * First, a page of data is read from the main memory array of
 * the device into the data cache. Then, the data is
 * transfered from the cache to the input buffer of the micro.
 *************************************************************/
uint8_t flash_ReadPage(uint8_t blockPageAddress[], uint8_t columnAddress[], uint8_t pageData[]);

/*************************************************************
 * FUNCTION: flash_ReadPage()
 * -----------------------------------------------------------
 * This function checks the busy flag in the status register. 
 * If the device is NOT busy, then the status will become
 * false, signifying that the device is not busy.
 *************************************************************/
bool flash_IsBusy();

uint8_t flash_BlockErase(uint8_t blockAddress[]);
uint8_t flash_BlockLockStatus();
uint8_t flash_UnlockBlocks();

/* INTERNAL FUNCTIONS - NOT A PART OF API */
uint8_t ProgramCache(uint8_t data[], int dataSize, uint8_t colAddress[]);
uint8_t ExecuteProgram(uint8_t pageBlockAddress[]);
void ReinitializeOutBuff();
uint8_t PageRead(uint8_t blockPageAddress[]); //3 byte address. upper 7 bits low, next 11 bits block address, last 6 bits page address
uint8_t ReadFromCache(uint8_t columnAddress[], uint8_t pageData[]); //col address most likely zeros. offset where to start reading

#endif /* NAND_FLASH_H_ */