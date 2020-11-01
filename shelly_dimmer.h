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

#ifndef __SHELLY_DIMMER__H_
#define __SHELLY_DIMMER__H_

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"

#include "stm32flash.h"

namespace esphome {
namespace shd {

static const uint8_t SHD_DRIVER_MAJOR_VERSION = 0x02;
static const uint8_t SHD_DRIVER_MINOR_VERSION = 0x30;

static const uint8_t SHD_VERSION_CMD = 0x01;
static const uint8_t SHD_BRIGHTNESS_CMD = 0x02;
static const uint8_t SHD_BRIGHTNESS_FADE_CMD = 0x03;
static const uint8_t SHD_CALIBRATION_BRIGHTNESS_CMD = 0x05;
static const uint8_t SHD_GET_STATE_CMD = 0x10;
static const uint8_t SHD_SET_STATE_CMD = 0x20;
static const uint8_t SHD_WARMUP_CMD = 0x21;
static const uint8_t SHD_SET_CALIBRATION_PART1_CMD = 0x30;
static const uint8_t SHD_SET_CALIBRATION_PART2_CMD = 0x31;

static const uint8_t SHD_START_BYTE = 0x01;
static const uint8_t SHD_END_BYTE = 0x04;

static const uint8_t GPIO_SHELLY_DIMMER_BOOT0 = 4;
static const uint8_t GPIO_SHELLY_DIMMER_RST_INV = 5;
static const uint8_t GPIO_NTC_THERMISTOR = A0;

static const uint8_t SHD_LEADING_EDGE_PULSE = 1;
static const uint8_t SHD_TRAILING_EDGE_PULSE = 2;

static const char *TAG = "shd";

static const uint8_t SHD_BUFFER_SIZE = 0xff;

class ShdCalibration;

class ShdComponent : public Component, public light::LightOutput
{
public:

  void loop() override;
  void setup() override;
  void dump_config() override;
  light::LightTraits get_traits() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;
  void set_pulse_type_to_leading_edge(void);
  void set_pulse_type_to_trailing_edge(void);
  void set_transition_length(int16_t transistion_time);
  void set_zcross_debounce(uint16_t zcross_debounce);
  void set_warmup(uint16_t warmup_brightness, uint16_t warmup_time);
  void start_calibration();

  float get_current_wattage(void);

  sensor::Sensor *wattage_sensor{nullptr};

  bool calibrate_on_startup = false;

protected:
  friend ShdCalibration;

  bool handle_command_(uint8_t match_cmd, uint8_t match_id);
  bool process_serial_input_(uint8_t match_cmd, uint8_t match_id);
  bool send_version_();
  bool update_firmware_(const uint8_t data[], unsigned int size);
  void save_rtc_state_();
  int check_byte_();
  uint16_t checksum_(uint8_t *buf, int len);
  uint8_t send_brightness_cmd_(uint16_t brightness);
  uint8_t send_calibration_brightness_cmd_(uint16_t brightness);
  uint8_t send_cmd_(uint8_t cmd, uint8_t *payload, uint8_t len);
  uint8_t send_settings_();
  uint8_t send_warmup_(uint16_t warmup_brightness, uint16_t warmup_time);
  uint8_t set_brightness_fade_(uint16_t brightness);
  void debug_state_();
  void reset_to_app_mode_();
  void reset_to_dfu_mode_();
  void send_calibration_(void);
  void set_fade_rate_cmd_(uint8_t fade_rate);
  float taylor_log_(float x);
  float get_board_temp_(void);

private:

  ESPPreferenceObject rtc_;

  // serial receive buffer
  uint8_t buffer_[SHD_BUFFER_SIZE];

  // index in serial receive buffer
  int byte_counter_ = 0;

  // packet counter
  uint8_t counter_ = 1;

  // last poll in ms
  uint32_t last_poll_ = 0;

  // last board temp measured in ms
  uint32_t last_temp_ = 0;

  light::LightState *state_{nullptr};
  ShdCalibration *calibration_{nullptr};

  uint8_t version_major_ = 0;
  uint8_t version_minor_ = 0;

  float wattage_;
  float board_temp_;

  uint16_t brightness_;

  // Transition Time
  int16_t transition_length_; // 0-5000ms

  // Fade rate speed
  uint16_t fade_rate_;

#ifdef SHELLY_DIMMER_2
  // Anti-flickering debounce
  uint8_t zcross_debounce_; // 50-150
#endif

  // Some light bulbs need more energy to wake up when brightness is below 10%.
  // You can create brightness and time that will create an initial impulse.
  uint16_t warmup_brightness_; // 10-100
  uint16_t warmup_time_; // 20-200ms

  uint16_t pulse_type_;

  uint16_t calibration_table_[2][100] {{0}};
};

class TimerPoll {
public:
  void init(int timeout) {
    this->timeout_ = millis() + timeout;
    this->is_busy_ = true;
  }

  bool is_due(void) {
    if (!this->is_busy_)
      return true;

    if (millis() >= this->timeout_) {
      this->is_busy_ = false;
      return true;
    }
    return false;
  }

protected:
  uint32_t timeout_;
  uint8_t is_busy_;
};

class ShdCalibration {
public:

  ShdCalibration(ShdComponent *parent) : current_state_(CALIBRATION_INIT) {
    this->parent_ = parent;
  }

  bool is_calibrating(void) { return is_calibrating_;  }
  bool is_finished(void) { return this->current_state_ == CALIBRATION_FINISHED; }

  void stop(void)
  {
    this->stop_ = true;
    this->percentage_ = 0;
  }

  void close(void) {
    this->is_calibrating_ = false;
    this->stop_ = true;
  }

  void loop() {
    switch (this->current_state_)
    {
    case CALIBRATION_INIT: {
      this->is_calibrating_ = true;
      this->stop_ = false;

      this->idx_ = 0;
      this->brightness_ = 0;
      this->error_count_ = 0;
      this->unknown_value_ = 0.0f;

      this->timer_poll_.init(500);

      this->current_state_ = CALIBRATION_STEP_1;
      //ESP_LOGD(TAG, "CALIBRATION_INIT");
    }
    break;

    case CALIBRATION_STEP_1: {
      if (this->timer_poll_.is_due()) {
        this->brightness_cmd_busy_ = false;
        this->current_state_ = CALIBRATION_STEP_2;

        //ESP_LOGD(TAG, "CALIBRATION_STEP_1");
      }
    }
    break;

    case CALIBRATION_STEP_2: { // 100% brightness
      if (this->send_brightness_cmd_(&this->brightness_cmd_busy_, 1000)) {
        this->timer_poll_.init(5000);

        this->current_state_ = CALIBRATION_STEP_3;

        //ESP_LOGD(TAG, "CALIBRATION_STEP_2");
      }
    }
    break;

    case CALIBRATION_STEP_3: {
      if (this->timer_poll_.is_due()) {
        this->poll_cmd_busy_ = false;
        this->current_state_ = CALIBRATION_STEP_4;

        //ESP_LOGD(TAG, "CALIBRATION_STEP_3");
      }
    }
    break;

    case CALIBRATION_STEP_4: { // average wattage from 100% brightness
      if (this->send_get_state_cmd_(&this->poll_cmd_busy_, 16)) {
        this->full_output_wattage_ = this->avg_wattage_;
        this->error_value_ = (float)(this->avg_wattage_ * 1.999f);

        if (this->avg_wattage_ <= 1.0f) {
          this->is_calibrating_ = false;
          break;
        }

        if (this->avg_wattage_ / 100.0f < 0.3f) {
          this->unknown_value_ = 0.3f;
        }

        this->brightness_cmd_busy_ = false;
        this->current_state_ = CALIBRATION_STEP_5;

        //ESP_LOGD(TAG, "CALIBRATION_STEP_4 %f %f", this->avg_wattage_, this->error_value_);
      }
    }
    break;

    case CALIBRATION_STEP_5: { // brightness at 0%
      if (this->send_brightness_cmd_(&this->brightness_cmd_busy_, 0)) {
        this->timer_poll_.init(5000);
        this->current_state_ = CALIBRATION_STEP_6;
        //ESP_LOGD(TAG, "CALIBRATION_STEP_5");
      }
    }
    break;

    case CALIBRATION_STEP_6: {
      if (!this->timer_poll_.is_due()) {
        break;
      }

      this->percentage_ = 0;
      if (this->idx_ < this->tmp_calc_.size()) {
        this->brightness_cmd_busy_ = false;
        this->current_state_ = CALIBRATION_STEP_7;
        //ESP_LOGD(TAG, "CALIBRATION_STEP_6");
      }
    }
    break;

    case CALIBRATION_STEP_7: { // set variable brightness 
      if (this->send_brightness_cmd_(&this->brightness_cmd_busy_, this->brightness_)) {
        this->timer_poll_.init(800);
        this->current_state_ = CALIBRATION_STEP_8;
        //ESP_LOGD(TAG, "CALIBRATION_STEP_7");
      }
    }
    break;

    case CALIBRATION_STEP_8: { // 800ms after setting variable brightness
      if (this->timer_poll_.is_due()) {
        this->poll_cmd_busy_ = false;
        this->current_state_ = CALIBRATION_STEP_9;
        //ESP_LOGD(TAG, "CALIBRATION_STEP_8");
      }
    }
    break;

    case CALIBRATION_STEP_9: {
      if (!this->send_get_state_cmd_(&this->poll_cmd_busy_, 8))
        break;

      //ESP_LOGD(TAG, "CALIBRATION_STEP_9 %f %f", this->avg_wattage_, this->full_output_wattage_);

      if (this->avg_wattage_ >= this->idx_ * this->unknown_value_) {
        this->tmp_calc_[this->idx_].brightness = this->brightness_;
        this->tmp_calc_[this->idx_].avg_wattage = this->avg_wattage_;
        this->idx_ += 1;
      }
      this->brightness_ += 5;

      if (this->percentage_ < (this->avg_wattage_ * 100.0f) / this->full_output_wattage_) {
        this->percentage_ = (uint32_t)(this->avg_wattage_ * 100.0f / this->full_output_wattage_);
      }

      if (this->percentage_ >= 100) {
        this->percentage_ = 100;
      }

      if (this->avg_wattage_ < this->full_output_wattage_) {

        if (this->avg_wattage_ < this->full_output_wattage_ - this->error_value_) {
          this->error_count_ += 1;
        }

        //ESP_LOGD(TAG, "CALIBRATION_STEP_%u %u, %u, %u", this->current_state_, this->error_count_, this->stop_, this->idx_);
        if (this->error_count_ < 10 && !this->stop_ && this->idx_ < this->tmp_calc_.size()) {
          this->brightness_cmd_busy_ = false;
          this->current_state_ = CALIBRATION_STEP_7;
          break;
        }
      }

      this->current_state_ = CALIBRATION_FINISHED;
      this->percentage_ = 100;

      this->close();
    }
    break;

    default:
      this->current_state_ = CALIBRATION_INIT;
      break;
    }

    if (!this->is_calibrating_ || this->stop_) {
      this->close();
    }
  }

  void fill_calibration_table(uint16_t (&values)[2][100]) {
    int i;

    for (i = 0; i < this->idx_; i++) {
      values[0][i] = this->tmp_calc_[i].brightness;
      values[1][i] = (uint16_t)(1000.0f / this->full_output_wattage_ * this->tmp_calc_[i].avg_wattage);
    }

    for (i = this->idx_; i < this->tmp_calc_.size(); i++) {
      values[0][i] = 2000;
      values[1][i] = 2000;
    }
  }

protected:

  bool send_brightness_cmd_(bool* is_busy, int brightness) {

    if (!*is_busy) {
      this->last_id = this->parent_->send_calibration_brightness_cmd_(brightness);
      this->timer_poll_.init(200);
      *is_busy = true;
    }

    if (*is_busy) {
      if (this->timer_poll_.is_due()) {
        *is_busy = false;
        return false;
      }

      if (this->parent_->process_serial_input_(SHD_CALIBRATION_BRIGHTNESS_CMD, this->last_id)) {
        return true;
      }
    }

    return false;
  }

  bool send_get_state_cmd_(bool* is_busy, uint32_t send_amount) {
    if (!*is_busy) {
      this->sum_wattage_ = 0.0f;
      this->send_poll_count_ = 0;
      this->last_id = this->parent_->send_cmd_(SHD_GET_STATE_CMD, 0, 0);
      this->timer_poll_.init(200);
      *is_busy = true;
    }

    if (*is_busy) {
      if (this->timer_poll_.is_due()) {
        *is_busy = false;
      } else if (this->parent_->process_serial_input_(SHD_GET_STATE_CMD, this->last_id)) {
        if (this->parent_->get_current_wattage() > 0.0f) {
          this->sum_wattage_ += this->parent_->get_current_wattage();
        }

        //ESP_LOGD(TAG, "send_get_state_cmd_ state: %u, cnt: %u, wattage: %f", this->current_state_, this->send_poll_count_, this->parent_->get_current_wattage());

        this->send_poll_count_ += 1;
        if (this->send_poll_count_ >= send_amount) {
          this->avg_wattage_ = this->sum_wattage_ / send_amount;

          *is_busy = false;
          return true;
        }
      }
      
      if (this->send_poll_count_ < send_amount) {
        this->last_id = this->parent_->send_cmd_(SHD_GET_STATE_CMD, 0, 0);
        this->timer_poll_.init(200);
        *is_busy = true;
      }
    }

    return false;
  }

  enum CalibrationState : uint8_t {
    CALIBRATION_INIT,
    CALIBRATION_STEP_1,
    CALIBRATION_STEP_2,
    CALIBRATION_STEP_3,
    CALIBRATION_STEP_4,
    CALIBRATION_STEP_5,
    CALIBRATION_STEP_6,
    CALIBRATION_STEP_7,
    CALIBRATION_STEP_8,
    CALIBRATION_STEP_9,
    CALIBRATION_FINISHED,
  };

  typedef struct tmp_calc_t {
    uint16_t brightness;
    float avg_wattage;
  } tmp_calc_t;

  uint16_t brightness_;
  uint8_t error_count_;
  float full_output_wattage_;
  float unknown_value_;
  float avg_wattage_;
  float error_value_;
  bool is_calibrating_;
  uint8_t percentage_;
  bool stop_;
  TimerPoll timer_poll_;
  bool brightness_cmd_busy_;
  bool poll_cmd_busy_;
  CalibrationState current_state_;
  float sum_wattage_;
  uint8_t send_poll_count_;
  uint8_t last_id;
  std::array<tmp_calc_t, 100> tmp_calc_;
  uint8_t idx_;
  ShdComponent *parent_;
};

}  // namespace shd
}  // namespace esphome

#endif
