/* 
 * File:   spi.h
 * Author: Brian Schmalz
 *
 * Created on November 25, 2018, 2:35 PM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif

void spi_init(void);
void spi_send(unsigned char command, unsigned long int sendData); 
    



#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

