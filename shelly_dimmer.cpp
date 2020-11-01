/*
 * Copyright (c) 2020, Alexander Wichers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the esphome shelly dimmer implementation.
 *
 * Author: Alexander Wichers
 *
 */

#include <algorithm>

#include "esphome/core/log.h"

#include "shelly_dimmer.h"
#include "stm32flash.h"

namespace esphome {
namespace shd {

#ifdef SHELLY_DIMMER_2
#include "shdm-2_v2.48.h"
#else
#include "shdm-1_v2.48.h"
#endif

struct ShdRTCState {
  // Transition Time
  int16_t transition_length; // 0-5000ms

  // Fade rate speed
  uint16_t fade_rate{15};

#ifdef SHELLY_DIMMER_2
  // Anti-flickering debounce
  uint8_t zcross_debounce; // 50-150
#endif

  uint16_t min_brightness;

  // Some light bulbs need more energy to wake up when the brightness is below 10%.
  // You can create brightness and time that will create an initial impulse.
  uint16_t warmup_brightness{0}; // 10-100
  uint16_t warmup_time{0}; // 20-200ms

  uint8_t pulse_type{SHD_TRAILING_EDGE_PULSE};
  uint16_t calibration_table[2][100]{{0}};
};

static const uint32_t SHD_PREFERENCE_VERSION = 3741239533UL;

uint16_t ShdComponent::checksum_(uint8_t *buf, int len) {
  uint16_t chksm = 0;
  for (uint8_t i = 0; i < len; i++) {
    chksm += buf[i];
  }
  return chksm;
}

uint8_t ShdComponent::send_cmd_(uint8_t cmd, uint8_t *payload, uint8_t len) {
  uint8_t header[4];
  uint8_t footer[3];
  uint16_t chksm;
  uint8_t packet_id = this->counter_++;

  header[0] = SHD_START_BYTE;
  header[1] = packet_id;
  header[2] = cmd;
  header[3] = len;
  Serial.write(header, sizeof(header));

  chksm = this->checksum_(header + 1, sizeof(header) - 1);
  if (payload && len) {
    chksm += this->checksum_(payload, len);
    Serial.write(payload, len);
  }

  footer[0] = chksm >> 8;
  footer[1] = chksm & 0xff;
  footer[2] = SHD_END_BYTE;
  Serial.write(footer, sizeof(footer));
  Serial.flush();
  delay(1); // without this 1ms delay we get no response, why?

  //ESP_LOGD(TAG, "tx cmd: %02x", cmd);

#if 0
  //ESP_LOGD(TAG, "tx: (%u)", len);  // NOLINT
  for (uint8_t i = 0; i < 4; i++) {
    //ESP_LOGD(TAG, "  0x%02X", header[i]);
  }
  for (uint8_t i = 0; i < len; i++) {
    //ESP_LOGD(TAG, "  0x%02X", payload[i]);
  }
  for (uint8_t i = 0; i < 3; i++) {
    //ESP_LOGD(TAG, "  0x%02X", footer[i]);
  }
#endif

  return packet_id;
}

uint8_t ShdComponent::send_brightness_cmd_(uint16_t brightness) {
  uint8_t payload[6];
  uint16_t prev_brightness = this->brightness_;

  this->brightness_ = brightness;

  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;

  auto transition_length = std::max<int16_t>(this->transition_length_ - 200, 0);
  if (transition_length > 0) {
    uint16_t delta;

    if (brightness > prev_brightness) {
      delta = (brightness - prev_brightness) * transition_length / 1000;
    } else {
      delta = (prev_brightness - brightness) * transition_length / 1000;
    }

    if (delta > 0) {
      payload[2] = delta & 0xff;
      payload[3] = delta >> 8;

      payload[4] = 0;
      payload[5] = 0;

      return this->send_cmd_(SHD_BRIGHTNESS_FADE_CMD, payload, 6);
    }
  }

  return this->send_cmd_(SHD_BRIGHTNESS_CMD, payload, 2);
}

uint8_t ShdComponent::send_calibration_brightness_cmd_(uint16_t brightness) {
  uint8_t payload[2];

  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;

  return this->send_cmd_(SHD_CALIBRATION_BRIGHTNESS_CMD, payload, sizeof(payload));
}

// set fade rate x1-x5
void ShdComponent::set_fade_rate_cmd_(uint8_t fade_rate) {
  static const uint8_t fade_rate_tab[] = {5, 5, 10, 15, 20, 25, 25, 25};

  this->fade_rate_ = fade_rate_tab[fade_rate & 7];
  this->save_rtc_state_();
  this->send_settings_();
}

void ShdComponent::set_pulse_type_to_leading_edge(void) {
  this->pulse_type_ = SHD_LEADING_EDGE_PULSE;
  memset(this->calibration_table_, 0, sizeof(this->calibration_table_));
  this->save_rtc_state_();
  this->send_calibration_();
}

void ShdComponent::set_pulse_type_to_trailing_edge(void) {
  this->pulse_type_ = SHD_TRAILING_EDGE_PULSE;
  memset(this->calibration_table_, 0, sizeof(this->calibration_table_));
  this->save_rtc_state_();
  this->send_calibration_();
}

void ShdComponent::set_transition_length(int16_t transition_length) {
  if (transition_length >= 0 && transition_length <= 5000) {
    this->transition_length_ = transition_length;
    this->save_rtc_state_();
  }
}

#ifdef SHELLY_DIMMER_2
void ShdComponent::set_zcross_debounce(uint16_t zcross_debounce) {
  if (zcross_debounce >= 50 && zcross_debounce <= 150) {
    this->zcross_debounce_ = zcross_debounce;
    this->save_rtc_state_();
    this->send_settings_();
  }
}
#endif

void ShdComponent::set_warmup(uint16_t warmup_brightness, uint16_t warmup_time) {
  if (warmup_brightness >= 10 && warmup_brightness <= 100
   && warmup_time >= 20 && warmup_time <= 200) {
    this->warmup_brightness_ = warmup_brightness * 10;
    this->warmup_time_ = warmup_time;
    this->save_rtc_state_();
    this->send_warmup_(warmup_brightness, warmup_time);
  }
}

float ShdComponent::get_current_wattage(void) {
  return this->wattage_;
}

uint8_t ShdComponent::send_settings_() {
  uint8_t payload[12], pos = 0;

  payload[pos++] = this->brightness_ & 0xff;
  payload[pos++] = this->brightness_ >> 8;

  payload[pos++] = this->pulse_type_ & 0xff;
  payload[pos++] = this->pulse_type_ >> 8;

  payload[pos++] = this->fade_rate_ & 0xff;
  payload[pos++] = this->fade_rate_ >> 8;

#ifdef SHELLY_DIMMER_2
  payload[pos++] = this->zcross_debounce_ & 0xff;
  payload[pos++] = this->zcross_debounce_ >> 8;
#endif

  payload[pos++] = this->warmup_brightness_ & 0xff;
  payload[pos++] = this->warmup_brightness_ >> 8;

  payload[pos++] = this->warmup_time_ & 0xff;
  payload[pos++] = this->warmup_time_ >> 8;

  return this->send_cmd_(SHD_SET_STATE_CMD, payload, pos);
}

uint8_t ShdComponent::send_warmup_(uint16_t warmup_brightness, uint16_t warmup_time) {
  uint8_t payload[4];

  payload[0] = warmup_brightness & 0xff;
  payload[1] = warmup_brightness >> 8;

  payload[2] = warmup_time & 0xff;
  payload[3] = warmup_time >> 8;

  return this->send_cmd_(SHD_WARMUP_CMD, payload, sizeof(payload));
}

void ShdComponent::send_calibration_(void) {
  this->send_cmd_(SHD_SET_CALIBRATION_PART1_CMD, (uint8_t *) this->calibration_table_, sizeof(this->calibration_table_) / 2);
  this->send_cmd_(SHD_SET_CALIBRATION_PART2_CMD, (uint8_t *) this->calibration_table_ + sizeof(this->calibration_table_) / 2, sizeof(this->calibration_table_) / 2);
}

void ShdComponent::start_calibration() {
  if (this->calibration_ == nullptr) {
    memset(this->calibration_table_, 0, sizeof(this->calibration_table_));
    this->send_calibration_();

    this->calibration_ = new ShdCalibration(this);
  }
}

void ShdComponent::reset_to_app_mode_() {
  //ESP_LOGD(TAG, "request co-processor reset configuration, pin %d to high", GPIO_SHELLY_DIMMER_RST_INV);

  pinMode(GPIO_SHELLY_DIMMER_BOOT0, OUTPUT);
  digitalWrite(GPIO_SHELLY_DIMMER_BOOT0, LOW);

  pinMode(GPIO_SHELLY_DIMMER_RST_INV, OUTPUT);
  digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, LOW);
  delay(50);

  // clear in the receive buffer
  while (Serial.available()) {
    Serial.read();
  }

  digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, HIGH); // pull out of reset
  delay(50); // wait 50ms fot the co-processor to come online
}

void ShdComponent::reset_to_dfu_mode_()
{
  //ESP_LOGD(TAG, "request co-processor reset configuration, pin %d to high", GPIO_SHELLY_DIMMER_RST_INV);

  pinMode(GPIO_SHELLY_DIMMER_BOOT0, OUTPUT);
  digitalWrite(GPIO_SHELLY_DIMMER_BOOT0, HIGH);

  pinMode(GPIO_SHELLY_DIMMER_RST_INV, OUTPUT);
  digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, LOW);
  delay(50);

  // clear in the receive buffer
  while (Serial.available()) {
    Serial.read();
  }

  digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, HIGH); // pull out of reset
  delay(50); // wait 50ms for the co-processor to wake-up
}

void ShdComponent::loop() {

  if (this->calibrate_on_startup) {
    this->start_calibration();
    this->calibrate_on_startup = false;
  }
  
  if (this->calibration_ != nullptr) {
    this->calibration_->loop();
    if (!this->calibration_->is_calibrating()) {
      if (this->calibration_->is_finished()) {
        this->calibration_->fill_calibration_table(this->calibration_table_);
        this->save_rtc_state_();

        this->send_calibration_();
        this->send_brightness_cmd_(this->brightness_);
      }

      delete this->calibration_;
      this->calibration_ = nullptr;
    }
  } else {
    this->process_serial_input_(0, 0);

    if (this->last_poll_ == 0 || millis() - this->last_poll_ > 2000) {
      this->last_poll_ = millis();
      this->send_cmd_(SHD_GET_STATE_CMD, 0, 0);
    }

    if (this->last_temp_ == 0 || millis() - this->last_temp_ > 5000) {
      this->last_temp_ = millis();
      this->board_temp_ = this->get_board_temp_();
      //ESP_LOGD(TAG, "temperature: %f", this->board_temp_);
      if (this->board_temp_ > 75.0f) {
        ESP_LOGW(TAG, "board temperature: %f too hot, turning off", this->board_temp_);
        this->send_brightness_cmd_(0);
      }
    }

  }

}

void ShdComponent::save_rtc_state_() {
  ShdRTCState saved;
  saved.transition_length = this->transition_length_;
  saved.fade_rate = this->fade_rate_;
#ifdef SHELLY_DIMMER_2
  saved.zcross_debounce = this->zcross_debounce_;
#endif
  saved.warmup_brightness = this->warmup_brightness_;
  saved.warmup_time = this->warmup_time_;
  saved.pulse_type = this->pulse_type_;
  memcpy(saved.calibration_table, this->calibration_table_, sizeof(this->calibration_table_));
  this->rtc_.save(&saved);
}

bool ShdComponent::send_version_() {
  //ESP_LOGD(TAG, "sending version command");
  uint8_t id = this->send_cmd_(SHD_VERSION_CMD, 0, 0);
  for (int i = 0; i < 1000; i++) {
    if (ShdComponent::process_serial_input_(SHD_VERSION_CMD, id)) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool ShdComponent::update_firmware_(const uint8_t data[], unsigned int size) {
  bool ret = true;
  stm32_t *stm = stm32_init(&Serial, STREAM_SERIAL, 1);
  if (stm) {
    off_t offset = 0;
    uint8_t buffer[256];
    uint32_t len;
    const uint8_t *p_st = data;
    uint32_t  addr, start, end;
    stm32_err_t s_err;

    stm32_erase_memory(stm, 0, STM32_MASS_ERASE);

    addr = stm->dev->fl_start;
    end = addr + size;
    while(addr < end && offset < size) {
      uint32_t left = end - addr;
      len = sizeof(buffer) > left ? left : sizeof(buffer);
      len = len > size - offset ? size - offset : len;

      if (len == 0) {
        break;
      }

      memcpy(buffer, p_st, len);
      p_st += len;

      s_err = stm32_write_memory(stm, addr, buffer, len);
      if (s_err != STM32_ERR_OK) {
        ret = false;
        break;
      }

      addr  += len;
      offset  += len;
    }
    stm32_close(stm);
  }
  return ret;
}

void ShdComponent::setup() {
  //ESP_LOGD(TAG, "shelly dimmer driver v%d.%d", SHD_DRIVER_MAJOR_VERSION, SHD_DRIVER_MINOR_VERSION);

  Serial.begin(115200, SERIAL_8N1);
  Serial.flush();

  this->reset_to_app_mode_();
  bool got_version = this->send_version_();
  if (!got_version || (got_version &&
    (this->version_minor_ != SHD_FIRMWARE_MINOR_VERSION ||
    this->version_major_ != SHD_FIRMWARE_MAJOR_VERSION))) {
    ESP_LOGI(TAG, "we need to update the firmware to %u.%u", SHD_FIRMWARE_MAJOR_VERSION, SHD_FIRMWARE_MINOR_VERSION);

    Serial.end();
    Serial.begin(115200, SERIAL_8E1);
    this->reset_to_dfu_mode_();
    this->update_firmware_(stm_firmware, sizeof(stm_firmware));
    Serial.end();

    this->reset_to_app_mode_();
    Serial.begin(115200, SERIAL_8N1);

    this->send_version_();
  }

  this->rtc_ = global_preferences.make_preference<ShdRTCState>(SHD_PREFERENCE_VERSION);
  ShdRTCState recovered{};
  if (!this->rtc_.load(&recovered))
    return;

  this->transition_length_ = recovered.transition_length;
  this->fade_rate_ = recovered.fade_rate;
#ifdef SHELLY_DIMMER_2
  this->zcross_debounce_ = recovered.zcross_debounce;
#endif
  this->warmup_brightness_ = recovered.warmup_brightness;
  this->warmup_time_ = recovered.warmup_time;
  this->pulse_type_ = recovered.pulse_type;
  memcpy(this->calibration_table_, recovered.calibration_table, sizeof(this->calibration_table_));

  delay(50);

  this->send_settings_();
  this->send_calibration_();
  this->send_settings_();
}

void ShdComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "shelly dimmer stm32:");
  //ESP_LOGD(TAG, "stm32 version major %u", this->version_major_);
  //ESP_LOGD(TAG, "stm32 version minor %u", this->version_minor_);

  if (this->pulse_type_ == SHD_LEADING_EDGE_PULSE)
    ESP_LOGCONFIG(TAG, "   Method: leading");
  else
    ESP_LOGCONFIG(TAG, "   Method: trailing");
}

bool ShdComponent::process_serial_input_(uint8_t match_cmd, uint8_t match_id) {
  while (Serial.available()) {
    uint8_t serial_in_byte = Serial.read();
    this->buffer_[this->byte_counter_] = serial_in_byte;
    int check = this->check_byte_();
    if (check > 1) {
      // finished
      this->byte_counter_ = 0;
      return this->handle_command_(match_cmd, match_id);
    } else if (check == 0) {
      // wrong data
      //ESP_LOGD(TAG, "byte %i of received data frame is invalid", this->byte_counter_);
      this->byte_counter_ = 0;
    } else {
      this->byte_counter_++;
    }

  }
  return false;
}

int ShdComponent::check_byte_() {
  uint8_t index = this->byte_counter_;
  uint8_t byte = this->buffer_[index];

  if (index == 0)
    return byte == SHD_START_BYTE;

  if (index < 4)
    return 1;

  uint8_t data_length = this->buffer_[3];
  if ((4 + data_length + 3) > SHD_BUFFER_SIZE) {
    //ESP_LOGD(TAG, "data_length %i too large", data_length);
    return 0;
  }

  if (index < 4 + data_length + 1)
    return 1;

  if (index == 4 + data_length + 1) {
    uint16_t chksm = (this->buffer_[index - 1] << 8 | this->buffer_[index]);
    if (chksm != this->checksum_(&this->buffer_[1], 3 + data_length)) {
      //ESP_LOGD(TAG, "chksm %i mismatch", chksm);
      return 0;
    }

    return 1;
  }

  if (index == 4 + data_length + 2 && byte == SHD_END_BYTE)
    return index;

  return 0;
}

bool ShdComponent::handle_command_(uint8_t match_cmd, uint8_t match_id) {
  uint8_t pos = 1; // skip SHD_START_BYTE
  uint8_t id, cmd, len;
  bool ret = true;

  id = this->buffer_[pos++];
  cmd = this->buffer_[pos++];
  len = this->buffer_[pos++];

  //ESP_LOGD(TAG, "rx id: %u,%u cmd %02x,%02x", id, match_id, cmd, match_cmd);

  if (match_cmd && id != match_id)
    return false;

  if (match_cmd && cmd != match_cmd)
    return false;

  switch (cmd) {
    case SHD_GET_STATE_CMD: {
        // 1 when returning fade_rate, 0 when returning wattage
        //uint16_t unknown_0 = this->buffer_[pos + 1] << 8 | this->buffer_[pos + 0];

        uint32_t brightness = this->buffer_[pos + 3] << 8 | this->buffer_[pos + 2];
        uint32_t wattage_raw = this->buffer_[pos + 7] << 24 |
            this->buffer_[pos + 6] << 16 |
            this->buffer_[pos + 5] << 8 |
            this->buffer_[pos + 4];
        if (wattage_raw > 0) {
          // 1925000.0f is reversed from Shelly 1 1.7.2 ESP firmware, 1.9845f is a guess, output is a 100% match to original
          this->wattage_ = (1925000.0f / (float) wattage_raw / 1.9845f);
          if (this->wattage_sensor != nullptr) {
            this->wattage_sensor->publish_state(this->wattage_);
          }
        } else {
          this->wattage_ = 0.0f;
        }

        //ESP_LOGD(TAG, "brightness: %d power: %d wattage: %f", brightness, wattage_raw, this->wattage_);
      }
      break;
    case SHD_VERSION_CMD: {
        // returns a static, is this a version number?
        ret = this->buffer_[pos] == SHD_FIRMWARE_MINOR_VERSION &&
          this->buffer_[pos + 1] == SHD_FIRMWARE_MAJOR_VERSION;

        this->version_minor_ = this->buffer_[pos];
        this->version_major_ = this->buffer_[pos + 1];
      }
      break;
    case SHD_BRIGHTNESS_CMD:
    case SHD_BRIGHTNESS_FADE_CMD:
    case SHD_CALIBRATION_BRIGHTNESS_CMD:
    case SHD_SET_STATE_CMD:
    case SHD_WARMUP_CMD:
    case SHD_SET_CALIBRATION_PART1_CMD:
    case SHD_SET_CALIBRATION_PART2_CMD: {
        ret = (this->buffer_[pos] == 0x01);
      }
    break;
  }

  return ret;
}

light::LightTraits ShdComponent::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supports_brightness(true); // TODO, move to yaml
  return traits;
}

void ShdComponent::setup_state(light::LightState *state) {
  state_ = state;
}

void ShdComponent::write_state(light::LightState *state) {
  float brightness;
  state->current_values_as_brightness(&brightness);

  auto brightness_int = static_cast<uint16_t>(brightness * 1000);
  brightness_int = std::max<uint16_t>(brightness_int, 0);
  this->send_brightness_cmd_(brightness_int);
}

float ShdComponent::taylor_log_(float x)
{
  if (x <= 0.0f) { return NAN; }
  if (x == 1.0f) { return 0.0f; }
  float z = (x + 1.0f) / (x - 1.0f);                              // We start from power -1, to make sure we get the right power in each iteration;
  float step = ((x - 1.0f) * (x - 1.0f)) / ((x + 1.0f) * (x + 1.0f));   // Store step to not have to calculate it each time
  float totalValue = 0.0f;
  float powe = 1.0f;
  for (int count = 0; count < 10; count++) {            // Experimental number of 10 iterations
    z *= step;
    float y = (1.0f / powe) * z;
    totalValue = totalValue + y;
    powe = powe + 2.0f;
  }
  totalValue *= 2.0f;

  return totalValue;
}

float ShdComponent::get_board_temp_(void)
{
  // Range: 387 (cold) to 226 (hot)
  int adc = analogRead(GPIO_NTC_THERMISTOR);

  // Shelly NTC Thermistor
  // 3V3 --- ANALOG_NTC_BRIDGE_RESISTANCE ---v--- NTC --- Gnd
  //                                         |
  //                                        ADC0
  #define ANALOG_NTC_BRIDGE_RESISTANCE 32000.0f            // NTC Voltage bridge resistor
  #define ANALOG_NTC_RESISTANCE 10000.0f                   // NTC Resistance
  #define ANALOG_NTC_B_COEFFICIENT 3350.0f                 // NTC Beta Coefficient
  // Parameters for equation
  #define TO_CELSIUS(x) ((x) - 273.15f)
  #define TO_KELVIN(x) ((x) + 273.15f)
  #define ANALOG_V33                    3.3f              // ESP8266 Analog voltage
  #define ANALOG_T0                     TO_KELVIN(25.0f)  // 25 degrees Celcius in Kelvin (= 298.15)

  // Steinhart-Hart equation for thermistor as temperature sensor
  float Rt = (adc * ANALOG_NTC_BRIDGE_RESISTANCE) / (1024.0f * ANALOG_V33 - (float)adc);
  float BC = (float)ANALOG_NTC_B_COEFFICIENT * 10000.0f / 10000.0f;
  float T = BC / (BC / ANALOG_T0 + this->taylor_log_(Rt / (float)ANALOG_NTC_RESISTANCE));
  float temperature = TO_CELSIUS(T);

  return temperature;
}

}  // namespace shd
}  // namespace esphome
