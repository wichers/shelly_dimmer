#ifndef __SHELLY_DIMMER_PWM_H_
#define __SHELLY_DIMMER_PWM_H_

#include "Arduino.h"

class Dimmer {

public:

  Dimmer();
  virtual ~Dimmer();

  void setup();
  void loop();

protected:
  
  void _send_cmd(uint8_t cmd, uint8_t *payload, uint8_t len);
  void _send_switch_cmd(uint16 brightness);
  void _send_on_cmd(void);
  void _send_off_cmd(void);
  void _send_dimming_cmd(uint16_t brightness, uint32_t previous_state);
  void _send_polling_cmd(void);
  void _send_version_cmd(void);
  void _send_fade_rate_cmd(uint8_t fade_rate);
  void _send_set_state_cmd(uint16_t brightness, uint16_t func, uint16_t fade_rate);
  void _send_calibration_cmd(void);
  bool _read_available_cmd(void);
  bool _read_available_cmd_with_timeout(int timeout);
  
  float get_board_temp(void);

private:

  uint16_t _checksum(uint8_t *buf, int len);
  int _check_byte();
  bool _parse_cmd(uint8_t expected_cmd);
  
  uint8_t data_[4 + 72 + 3]; // maximum payload for 0x30 packet is 72
  uint8_t data_index_ {0}; 
  uint8_t counter_ = 1; // packet counter used for packet identification
  uint32_t start_time{0};
  uint8_t percentage = 0;
  uint8_t brightness_ = 0;
};

#endif
