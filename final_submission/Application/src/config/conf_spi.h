/**
 * \file
 *
 * \brief common SPI configuration
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


#ifndef CONF_SPI_H_INCLUDED
#  define CONF_SPI_H_INCLUDED

#  define CONF_SPI_MASTER_ENABLE     true
#  define CONF_SPI_SLAVE_ENABLE      false

/* Changes Required for SPI Communication with sensor */
/* Even though we don't use it in boot loader, still added here for consistency */
#define SLAVE_SELECT_PIN        CONF_MASTER_SS_PIN
#define CONF_MASTER_SPI_MODULE  SERCOM5
#define CONF_MASTER_SS_PIN      PIN_PA17                     // PA_17
#define CONF_MASTER_MUX_SETTING SPI_SIGNAL_MUX_SETTING_N     // Best setting for our pin configuration
#define CONF_MASTER_PINMUX_PAD0 PINMUX_PB02D_SERCOM5_PAD0    // MOSI
#define CONF_MASTER_PINMUX_PAD1 PINMUX_PB03D_SERCOM5_PAD1    // MISO can be on 1 and 3 only
#define CONF_MASTER_PINMUX_PAD2 PINMUX_UNUSED                // Unused
#define CONF_MASTER_PINMUX_PAD3 PINMUX_PA21C_SERCOM5_PAD3    // SCLK

#endif /* CONF_SPI_H_INCLUDED */

