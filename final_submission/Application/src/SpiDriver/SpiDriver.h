/*
 * SpiDriver.h
 *
 * Created: 12/13/2023 12:35:50 AM
 *  Author: praty
 */ 


#ifndef SPIDRIVER_H_
#define SPIDRIVER_H_


#include "conf_spi.h"
#include "spi.h"

extern struct spi_module spi_master_instance;

void configure_spi_master(void);

#endif /* SPIDRIVER_H_ */