/*
 * HelperFunctions.h
 *
 * Created: 3/5/2018 3:38:16 PM
 *  Author: kmcarrin
 */ 


#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_


/* 
 * Changes little endian unsigned integers into big endian 
 * unsigned integers. The Feather M0 is a little endian 
 * processor, but the NAND Flash expects big endian addresses
 * and data. 
 */
unsigned int LitToBigEndian(unsigned int x);



#endif /* HELPERFUNCTIONS_H_ */