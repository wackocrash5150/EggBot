/* 
 * File:   spi.h
 * Author: Brian Schmalz
 *
 * Created on November 25, 2018, 2:35 PM
 */

#ifndef SPI_H
#define	SPI_H

void spi_init(void);
UINT32 spi_send_receive(UINT8 * rec_command, UINT8 send_command, UINT32 sendData); 
void parse_TS_packet(void);

#endif	/* SPI_H */