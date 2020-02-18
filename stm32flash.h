/*
  stm32flash - Open Source ST STM32 flash program for Arduino
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#ifndef _STM32FLASH_H
#define _STM32FLASH_H

#include "Arduino.h"

#define STM32_MAX_RX_FRAME  256 /* cmd read memory */
#define STM32_MAX_TX_FRAME  (1 + 256 + 1) /* cmd write memory */

#define STM32_MAX_PAGES   0x0000ffff
#define STM32_MASS_ERASE  0x00100000 /* > 2 x max_pages */

typedef enum {
  STM32_ERR_OK = 0,
  STM32_ERR_UNKNOWN,  /* Generic error */
  STM32_ERR_NACK,
  STM32_ERR_NO_CMD, /* Command not available in bootloader */
} stm32_err_t;

typedef struct stm32_cmd  stm32_cmd_t;

class stm32flash {
public:

  stm32flash(Stream& serial);
  ~stm32flash(void);

  bool init(bool send_init);
  stm32_err_t write_memory(uint32_t address,
               const uint8_t data[], unsigned int len);
  stm32_err_t go(uint32_t address);

protected:
  
  stm32_err_t _send_init_seq(void);
  stm32_err_t _guess_len_cmd(uint8_t cmd,
             uint8_t *data, unsigned int len);
  stm32_err_t _send_command(const uint8_t cmd);
  stm32_err_t _send_command_timeout(const uint8_t cmd,
              int timeout);
  stm32_err_t _get_ack(void);
  stm32_err_t _get_ack_timeout(int timeout);
  bool _check_read_timeout(size_t len, int timeout);

private:

  Stream&     _serial;
  uint8_t     bl_version;
  uint8_t     version;
  uint8_t     option1, option2;
  uint16_t    pid;
  stm32_cmd_t *cmd;
};

#endif
