#ifndef __ESPHOME_DIMMER_PWM_H_
#define __ESPHOME_DIMMER_PWM_H_

#include "esphome.h"
#include "esphome/core/component.h" 
#include "esphome/components/switch/switch.h"
#include "esphome/core/log.h"

#include "shelly_dimmer_pwm.h"

namespace esphome {
  
class EspHomeDimmer : public Component, public switch_::Switch {

public:

  EspHomeDimmer() 
  : dimmer() {
  }
  
  ~EspHomeDimmer() {
  }

  void setup() {
    dimmer.setup();
  }

  void dump_config() override {
    ESP_LOGCONFIG("pwm", "Shelly Dimmer STM32 PWM:");

    dimmer.setup();
  }

  void loop() override {
    dimmer.loop();
  }

  void write_state(bool state) override {
    this->publish_state(state);
  }

  Dimmer dimmer;
};

}  // namespace esphome

#endif
