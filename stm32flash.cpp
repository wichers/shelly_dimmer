/*
  stm32flash - Open Source ST STM32 flash program for Arduino
  Copyright 2010 Geoffrey McRae <geoff@spacevs.com>
  Copyright 2012-2014 Tormod Volden <debian.tormod@gmail.com>

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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "stm32flash.h"
#include "debug.h"

static const char *TAG = "stm32flash";

#define STM32_ACK 0x79
#define STM32_NACK  0x1F
#define STM32_BUSY  0x76

#define STM32_CMD_INIT  0x7F
#define STM32_CMD_GET 0x00  /* get the version and command supported */
#define STM32_CMD_GVR 0x01  /* get version and read protection status */
#define STM32_CMD_GID 0x02  /* get ID */
#define STM32_CMD_RM  0x11  /* read memory */
#define STM32_CMD_GO  0x21  /* go */
#define STM32_CMD_WM  0x31  /* write memory */
#define STM32_CMD_WM_NS 0x32  /* no-stretch write memory */
#define STM32_CMD_ER  0x43  /* erase */
#define STM32_CMD_EE  0x44  /* extended erase */
#define STM32_CMD_EE_NS 0x45  /* extended erase no-stretch */
#define STM32_CMD_WP  0x63  /* write protect */
#define STM32_CMD_WP_NS 0x64  /* write protect no-stretch */
#define STM32_CMD_UW  0x73  /* write unprotect */
#define STM32_CMD_UW_NS 0x74  /* write unprotect no-stretch */
#define STM32_CMD_RP  0x82  /* readout protect */
#define STM32_CMD_RP_NS 0x83  /* readout protect no-stretch */
#define STM32_CMD_UR  0x92  /* readout unprotect */
#define STM32_CMD_UR_NS 0x93  /* readout unprotect no-stretch */
#define STM32_CMD_CRC 0xA1  /* compute CRC */
#define STM32_CMD_ERR 0xFF  /* not a valid command */

#define STM32_MASSERASE_TIMEOUT 35  /* seconds */
#define STM32_PAGEERASE_TIMEOUT 5 /* seconds */
#define STM32_BLKWRITE_TIMEOUT  1 /* seconds */
#define STM32_WUNPROT_TIMEOUT 1 /* seconds */
#define STM32_WPROT_TIMEOUT 1 /* seconds */
#define STM32_RPROT_TIMEOUT 1 /* seconds */

#define STM32_CMD_GET_LENGTH  17  /* bytes in the reply */

struct stm32_cmd {
  uint8_t get;
  uint8_t gvr;
  uint8_t gid;
  uint8_t rm;
  uint8_t go;
  uint8_t wm;
  uint8_t er; /* this may be extended erase */
  uint8_t wp;
  uint8_t uw;
  uint8_t rp;
  uint8_t ur;
  uint8_t crc;
};

stm32flash::stm32flash(Stream& serial) 
  : _serial(serial) {
  this->cmd = new stm32_cmd_t();
}

stm32flash::~stm32flash(void)
{
  delete this->cmd;
}

bool stm32flash::_check_read_timeout(size_t len, int timeout) 
{
  if (_serial.available() >= len)
    return true;

  uint32_t start_time = millis();
  while (_serial.available() < len) {
    if (millis() - start_time > 1000  * timeout) {
      return false;
    }
  }
  return true;
}

stm32_err_t stm32flash::_get_ack_timeout(int timeout)
{
  uint8_t byte;

  if (!this->_check_read_timeout(1, timeout)) {
    return STM32_ERR_UNKNOWN;   
  }

  _serial.readBytes(&byte, 1);
  if (byte == STM32_ACK)
    return STM32_ERR_OK;
  if (byte == STM32_NACK)
    return STM32_ERR_NACK;
  if (byte != STM32_BUSY) {
    // Got wrong byte instead of ACK
    return STM32_ERR_UNKNOWN;
  }
}

stm32_err_t stm32flash::_get_ack(void)
{
  return this->_get_ack_timeout(0);
}

stm32_err_t stm32flash::_send_command_timeout(const uint8_t cmd,
                int timeout)
{
  stm32_err_t s_err;
  uint8_t buf[2];

  buf[0] = cmd;
  buf[1] = cmd ^ 0xFF;
  _serial.write(buf, 2);
  s_err = this->_get_ack_timeout(timeout);
  if (s_err == STM32_ERR_OK)
    return STM32_ERR_OK;
  return STM32_ERR_UNKNOWN;
}

stm32_err_t stm32flash::_send_command(const uint8_t cmd)
{
  return this->_send_command_timeout(cmd, 0);
}

/*
 * some command receive reply frame with variable length, and length is
 * embedded in reply frame itself.
 * We can guess the length, but if we guess wrong the protocol gets out
 * of sync.
 * Use byte-by-byte read for byte oriented interfaces (e.g. UART).
 *
 * to run safely, data buffer should be allocated for 256+1 bytes
 *
 * len is value of the first byte in the frame.
 */
stm32_err_t stm32flash::_guess_len_cmd(uint8_t cmd,
               uint8_t *data, unsigned int len)
{
  if (this->_send_command(cmd) != STM32_ERR_OK)
    return STM32_ERR_UNKNOWN;

  _serial.readBytes(data, 1);
  len = data[0];
  _serial.readBytes(data + 1, len + 1);
  return STM32_ERR_OK;
}

/*
 * Some interface, e.g. UART, requires a specific init sequence to let STM32
 * autodetect the interface speed.
 * The sequence is only required one time after reset.
 * stm32flash has command line flag "-c" to prevent sending the init sequence
 * in case it was already sent before.
 * User can easily forget adding "-c". In this case the bootloader would
 * interpret the init sequence as part of a command message, then waiting for
 * the rest of the message blocking the interface.
 * This function sends the init sequence and, in case of timeout, recovers
 * the interface.
 */
stm32_err_t stm32flash::_send_init_seq(void)
{
  uint8_t byte, cmd = STM32_CMD_INIT;

  _serial.write(&cmd, 1);
  _serial.readBytes(&byte, 1);
  if (byte == STM32_ACK)
    return STM32_ERR_OK;
  if (byte == STM32_NACK) {
    /* We could get error later, but let's continue, for now. */
    // Warning: the interface was not closed properly.
    return STM32_ERR_OK;
  }

  /*
   * Check if previous STM32_CMD_INIT was taken as first byte
   * of a command. Send a new byte, we should get back a NACK.
   */
  _serial.write(&cmd, 1);
  _serial.readBytes(&byte, 1);
  if (byte == STM32_NACK)
    return STM32_ERR_OK;
  // Failed to init device.
  return STM32_ERR_UNKNOWN;
}

/* find newer command by higher code */
#define newer(prev, a) (((prev) == STM32_CMD_ERR) \
      ? (a) \
      : (((prev) > (a)) ? (prev) : (a)))

bool stm32flash::init(bool send_init)
{
  uint8_t len, val, buf[257];
  int i;

  memset(this->cmd, STM32_CMD_ERR, sizeof(stm32_cmd_t));

  if (send_init && this->_send_init_seq() != STM32_ERR_OK)
  {
  DEBUG_MSG(TAG, "This fails");

    return false;
    }

  /* get the version and read protection status  */
  if (this->_send_command(STM32_CMD_GVR) != STM32_ERR_OK) {
    return false;
  }

  /* only UART bootloader returns 3 bytes */
  _serial.readBytes(buf, 3);
  this->version = buf[0];
  this->option1 = buf[1];
  this->option2 = buf[2];
  if (this->_get_ack() != STM32_ERR_OK) {
    return false;
  }

  /* get the bootloader information */
  if (this->_guess_len_cmd(STM32_CMD_GET, buf, STM32_CMD_GET_LENGTH) != STM32_ERR_OK)
    return false;
  len = buf[0] + 1;
  this->bl_version = buf[1];
  for (i = 1; i < len; i++) {
    val = buf[i + 1];
    switch (val) {
    case STM32_CMD_GET:
      this->cmd->get = val; break;
    case STM32_CMD_GVR:
      this->cmd->gvr = val; break;
    case STM32_CMD_GID:
      this->cmd->gid = val; break;
    case STM32_CMD_RM:
      this->cmd->rm = val; break;
    case STM32_CMD_GO:
      this->cmd->go = val; break;
    case STM32_CMD_WM:
    case STM32_CMD_WM_NS:
      this->cmd->wm = newer(this->cmd->wm, val);
      break;
    case STM32_CMD_ER:
    case STM32_CMD_EE:
    case STM32_CMD_EE_NS:
      this->cmd->er = newer(this->cmd->er, val);
      break;
    case STM32_CMD_WP:
    case STM32_CMD_WP_NS:
      this->cmd->wp = newer(this->cmd->wp, val);
      break;
    case STM32_CMD_UW:
    case STM32_CMD_UW_NS:
      this->cmd->uw = newer(this->cmd->uw, val);
      break;
    case STM32_CMD_RP:
    case STM32_CMD_RP_NS:
      this->cmd->rp = newer(this->cmd->rp, val);
      break;
    case STM32_CMD_UR:
    case STM32_CMD_UR_NS:
      this->cmd->ur = newer(this->cmd->ur, val);
      break;
    case STM32_CMD_CRC:
      this->cmd->crc = newer(this->cmd->crc, val);
      break;
    default:
      break;
    }
  }
  if (this->_get_ack() != STM32_ERR_OK) {
    return false;
  }

  if (this->cmd->get == STM32_CMD_ERR
      || this->cmd->gvr == STM32_CMD_ERR
      || this->cmd->gid == STM32_CMD_ERR) {
    // Error: bootloader did not returned correct information from GET command
    return false;
  }

  /* get the device ID */
  if (this->_guess_len_cmd(this->cmd->gid, buf, 1) != STM32_ERR_OK) {
    return false;
  }
  len = buf[0] + 1;
  if (len < 2) {
    // unknown/unsupported device
    return false;
  }
  this->pid = (buf[1] << 8) | buf[2];
  if (this->_get_ack() != STM32_ERR_OK) {
    return false;
  }

  return true;
}

stm32_err_t stm32flash::go(uint32_t address)
{
  uint8_t buf[5];

  if (this->cmd->go == STM32_CMD_ERR) {
    // GO command not implemented in bootloader.
    return STM32_ERR_NO_CMD;
  }

  if (this->_send_command(this->cmd->go) != STM32_ERR_OK)
    return STM32_ERR_UNKNOWN;

  buf[0] = address >> 24;
  buf[1] = (address >> 16) & 0xFF;
  buf[2] = (address >> 8) & 0xFF;
  buf[3] = address & 0xFF;
  buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
  _serial.write(buf, 5);

  if (this->_get_ack() != STM32_ERR_OK)
    return STM32_ERR_UNKNOWN;
  return STM32_ERR_OK;
}

stm32_err_t stm32flash::write_memory(uint32_t address,
             const uint8_t data[], unsigned int len)
{
  uint8_t cs, buf[256 + 2];
  unsigned int i, aligned_len;
  stm32_err_t s_err;

  if (!len)
    return STM32_ERR_OK;

  if (len > 256) {
    // Error: READ length limit at 256 bytes
    return STM32_ERR_UNKNOWN;
  }

  /* must be 32bit aligned */
  if (address & 0x3) {
    // Error: WRITE address must be 4 byte aligned
    return STM32_ERR_UNKNOWN;
  }

  if (this->cmd->wm == STM32_CMD_ERR) {
    // Error: WRITE command not implemented in bootloader.
    return STM32_ERR_NO_CMD;
  }

  /* send the address and checksum */
  if (this->_send_command(this->cmd->wm) != STM32_ERR_OK)
    return STM32_ERR_UNKNOWN;

  buf[0] = address >> 24;
  buf[1] = (address >> 16) & 0xFF;
  buf[2] = (address >> 8) & 0xFF;
  buf[3] = address & 0xFF;
  buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
  _serial.write(buf, 5);
  if (this->_get_ack() != STM32_ERR_OK)
    return STM32_ERR_UNKNOWN;

  aligned_len = (len + 3) & ~3;
  cs = aligned_len - 1;
  buf[0] = aligned_len - 1;
  for (i = 0; i < len; i++) {
    cs ^= data[i];
    buf[i + 1] = data[i];
  }
  /* padding data */
  for (i = len; i < aligned_len; i++) {
    cs ^= 0xFF;
    buf[i + 1] = 0xFF;
  }
  buf[aligned_len + 1] = cs;
  _serial.write(buf, aligned_len + 2);

  s_err = this->_get_ack_timeout(STM32_BLKWRITE_TIMEOUT);
  if (s_err != STM32_ERR_OK) {
    return STM32_ERR_UNKNOWN;
  }
  return STM32_ERR_OK;
}
