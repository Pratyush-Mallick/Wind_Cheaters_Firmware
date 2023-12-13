/**
 * \file
 *
 * \brief Autogenerated API include file for the Atmel Software Framework (ASF)
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ASF_H
#define ASF_H

/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: CRC-32 calculation
#include <crc32.h>

// From module: CRC32 - 32-bit cyclic redundancy check
#include <crc32.h>

// From module: Common SAM0 compiler driver
#include <compiler.h>
#include <status_codes.h>

// From module: Debug Print (FreeRTOS)
#include <dbg_print.h>
#include <quick_start_basic/qs_dbg_print_basic.h>

// From module: Delay routines
#include <delay.h>

// From module: EXTINT - External Interrupt (Callback APIs)
#include <extint.h>
#include <extint_callback.h>

// From module: FatFS file system
#include <diskio.h>
#include <ff.h>
#include <ffconf.h>
#include <integer.h>

// From module: FreeRTOS - kernel 10.0.0
#include <FreeRTOS.h>
#include <StackMacros.h>
#include <croutine.h>
#include <deprecated_definitions.h>
#include <event_groups.h>
#include <list.h>
#include <message_buffer.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <stack_macros.h>
#include <stream_buffer.h>
#include <task.h>
#include <timers.h>

// From module: Generic board support
#include <board.h>

// From module: Interrupt management - SAM implementation
#include <interrupt.h>

// From module: Memory Control Access Interface
#include <ctrl_access.h>

// From module: NVM - Non-Volatile Memory
#include <nvm.h>

// From module: PAC - Peripheral Access Controller
#include <pac.h>

// From module: PORT - GPIO Pin Control
#include <port.h>

// From module: Part identification macros
#include <parts.h>

// From module: RTC - Real Time Counter in Calendar Mode (Callback APIs)
#include <rtc_calendar.h>
#include <rtc_calendar_interrupt.h>
#include <rtc_tamper.h>

// From module: SD/MMC Memory Control Access - Enable
#include <sd_mmc_mem.h>

// From module: SD/MMC stack on SPI interface
#include <sd_mmc.h>

// From module: SERCOM Callback API
#include <sercom.h>
#include <sercom_interrupt.h>

// From module: SERCOM I2C - Master Mode I2C (Callback APIs)
#include <i2c_common.h>
#include <i2c_master.h>
#include <i2c_master_interrupt.h>

// From module: SERCOM SPI - Serial Peripheral Interface (Callback APIs)
#include <spi.h>
#include <spi_interrupt.h>

// From module: SERCOM USART - Serial Communications (Callback APIs)
#include <usart.h>
#include <usart_interrupt.h>

// From module: SYSTEM - Clock Management for SAMD21/R21/DA/HA
#include <clock.h>
#include <gclk.h>

// From module: SYSTEM - Core System Driver
#include <system.h>

// From module: SYSTEM - I/O Pin Multiplexer
#include <pinmux.h>

// From module: SYSTEM - Interrupt Driver
#include <system_interrupt.h>

// From module: SYSTEM - Power Management for SAM D20/D21/R21/D09/D10/D11/DA/HA
#include <power.h>

// From module: SYSTEM - Reset Management for SAM D20/D21/R21/D09/D10/D11/DA/HA
#include <reset.h>

// From module: Sleep manager - SAMD implementation
#include <samd/sleepmgr.h>
#include <sleepmgr.h>

// From module: Standard serial I/O (stdio)
#include <stdio_serial.h>

// From module: Supported ports of FatFS
#include <diskio.h>

// From module: TC - Timer Counter (Callback APIs)
#include <tc.h>
#include <tc_interrupt.h>

// From module: TCC - Timer Counter for Control Applications (Callback APIs)
#include <tcc.h>
#include <tcc_callback.h>

// From module: USART - Serial interface- SAM implementation for devices with only USART
#include <serial.h>

#endif // ASF_H
