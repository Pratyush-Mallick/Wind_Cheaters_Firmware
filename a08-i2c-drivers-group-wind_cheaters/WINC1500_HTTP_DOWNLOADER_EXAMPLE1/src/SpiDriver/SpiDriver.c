/*
 * SpiDriver.c
 *
 * Created: 12/13/2023 12:35:29 AM
 *  Author: praty
 */ 

#include <asf.h>
#include "conf_spi.h"
#include "spi.h"

struct spi_module spi_master_instance;
struct spi_slave_inst slave;

/**************************************************************************//**
* @fn		void configure_spi_master(void)
* @brief	Configures the SERCOM 5 as SPI master. 
                				
* @param[in]	
* @param[out]	
* @return		
* @note         
*****************************************************************************/
void configure_spi_master(void)
{
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;

	/* Configure and initialize software device instance of peripheral slave */
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = SLAVE_SELECT_PIN;
	spi_attach_slave(&slave, &slave_dev_config);
	
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = CONF_MASTER_MUX_SETTING;
	config_spi_master.pinmux_pad0 = CONF_MASTER_PINMUX_PAD0;
	config_spi_master.pinmux_pad1 = CONF_MASTER_PINMUX_PAD1;
	config_spi_master.pinmux_pad2 = CONF_MASTER_PINMUX_PAD2;
	config_spi_master.pinmux_pad3 = CONF_MASTER_PINMUX_PAD3;
	
	/* Works upto 1 MHZ with small jumpers. Doesn't work with long jumper wires. 	*/
	config_spi_master.mode_specific.master.baudrate = 1000000;  // 1 MHZ clock.
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_1;  // works with both mode 0 and mode 1
	config_spi_master.select_slave_low_detect_enable = false;

	spi_init(&spi_master_instance, CONF_MASTER_SPI_MODULE, &config_spi_master);
	spi_enable(&spi_master_instance);
}