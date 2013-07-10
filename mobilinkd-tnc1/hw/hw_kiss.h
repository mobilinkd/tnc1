/**
 * \file
 * <!--
 * This file is part of BeRTOS.
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright 2012 Robin Gilks
 * All Rights Reserved.
 * -->
 *
 * \brief EEPROM load/save functions for KISS protocol command parameters
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 */

#ifndef HW_KISS_H
#define HW_KISS_H

#include "cfg/cfg_arch.h"

#include <avr/eeprom.h>


// move this eeprom definition to a kblock device eventually
Params EEMEM eeparams;


#define KISS_EEPROM_SAVE() do { eeprom_write_block ((const void *) &k->params, (void *) &eeparams, sizeof (eeparams)); } while (0)

#define KISS_EEPROM_LOAD() do { eeprom_read_block ((void *) &k->params, (const void *) &eeparams, sizeof (eeparams)); } while (0)


#endif



