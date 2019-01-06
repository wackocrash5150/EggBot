/* 
 * File:   spi.h
 * Author: Brian Schmalz
 *
 * Created on November 25, 2018, 2:35 PM
 */

#ifndef SPI_H
#define	SPI_H

void spi_init(void);
void spi_send(unsigned char command, unsigned long int sendData); 

#endif	/* SPI_H */