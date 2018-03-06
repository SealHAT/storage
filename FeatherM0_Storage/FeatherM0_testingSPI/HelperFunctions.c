/*
 * HelperFunctions.c
 *
 * Created: 3/5/2018 1:34:18 PM
 *  Author: kmcarrin
 */ 

#include "HelperFunctions.h"

/**************************************************************
 * FUNCTION: LitToBigEndian
 * ------------------------------------------------------------
 * Changes little endian unsigned integers into big endian 
 * unsigned integers. The Feather M0 is a little endian 
 * processor, but the NAND Flash expects big endian addresses
 * and data. 
 * 
 *  Parameters:
 *      x   :   Little endian unsigned integer.
 *
 *  Returns:
 *      x   :   Big endian unsigned integer.
 **************************************************************/
unsigned int LitToBigEndian(unsigned int x)
{
    return (((x>>24) & 0x000000ff) | ((x>>8) & 0x0000ff00) | ((x<<8) & 0x00ff0000) | ((x<<24) & 0xff000000));
}