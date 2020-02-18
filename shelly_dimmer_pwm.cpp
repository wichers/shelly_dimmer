#include "shelly_dimmer_pwm.h"
#include "stm32flash.h"
#include "debug.h"

#define PWM_FIRMWARE_MAJOR_VERSION 0x02
#define PWM_FIRMWARE_MINOR_VERSION 0x16
#define PWM_GET_STATE_CMD 0x10
#define PWM_VERSION_CMD 0x11
#define PWM_SET_STATE_CMD 0x20

static const char *TAG = "pwm";

//A0 (pin 6) goes to a Thermistor.
//BOOT0 (pin 31) on the stm32 goes to GPIO4 (pin 16) on the esp.
//GPIO12 (pin 10) on the esp goes to the S1 switch (short S1 to ground makes the pin high, via PNP transistor)
//GPIO14 (pin 9) on the esp goes to the S2 switch (short S2 to ground makes the pin high, via PNP transistor)

// Constructor
Dimmer::Dimmer() 
{
}

// Destructor
Dimmer::~Dimmer() {
}

void Dimmer::setup() {
  Serial.end();
  delay(50); 
  Serial.begin(115200, SERIAL_8N1);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); // set Boot0 flag
  
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW); // put into reset
  delay(50);
  digitalWrite(5, HIGH); // pull out of reset

  stm32flash *flash = new stm32flash(Serial);
  DEBUG_MSG(TAG, "stm32flash init %u", flash->init(true));
  delete flash;
  
  digitalWrite(4, LOW); // set Boot0 flag
  digitalWrite(5, LOW); // put into reset
  delay(50);
  digitalWrite(5, HIGH); // pull out of reset

  this->_send_version_cmd();
  delay(50);

  DEBUG_MSG(TAG, "Avail %u", Serial.available());

  if (this->_read_available_cmd_with_timeout(100)) {
    if (!this->_parse_cmd(PWM_VERSION_CMD)) {
      // upgrade firmware
      

    }
  }

  //DEBUG_MSG(TAG, "Board temperature %f", this->get_board_temp());

  // set calibration values  
  //this->_send_cmd(0x30, payload, sizeof(payload));  
  //this->_send_cmd(0x31, payload, sizeof(payload));  
  
  // 0x02,0x20,0x06,0x00,0x00,0x02,0x00,0x0F,0x00,0x00,0x39,0x04
}
  
void Dimmer::loop() {

  // poll every two seconds for new data
  if (this->start_time == 0 || millis() - start_time > 1000  * 2) {
    this->start_time = millis();
    this->_send_polling_cmd();
  }
  
  
  if (this->_read_available_cmd()) {
    this->_parse_cmd(0);
  }
}
  
uint16_t Dimmer::_checksum(uint8_t *buf, int len) {
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum += buf[i];
    }
    return checksum;
}

void Dimmer::_send_cmd(uint8_t cmd, uint8_t *payload, uint8_t len) {
  uint16_t checksum;
  uint8_t pos = 0;

  data_[0] = 0x01;
  data_[1] = counter_++;
  data_[2] = cmd;
  data_[3] = len;

  pos += 4;

  if (payload) {
    memcpy(data_ + 4, payload, len);
    pos += len;
  }

  // calculate checksum from id and onwards
  checksum = this->_checksum(data_ + 1, 3 + len); 
  data_[pos++] = checksum >> 8;
  data_[pos++] = checksum & 0xff;
  data_[pos++] = 0x04;

  Serial.write(data_, pos);
}

//01 33 04 02 00 00 00 39 04
void Dimmer::_send_switch_cmd(uint16 brightness) {
  uint8_t payload[2];

  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;

//3.065991800000000,RX,0x01,0x21,0x04,0x02,0xE8,0x03,0x01,0x12,0x04 -> switch on

  this->_send_cmd(0x01, payload, sizeof(payload));
}

void Dimmer::_send_on_cmd(void) {
  uint16_t brightness = std::max<uint8_t>(std::min<uint8_t>(this->brightness_, 1), 100) * 10;
  //this->_send_dimming_cmd(brightness, brightness / 2);
  this->_send_switch_cmd(brightness);
}

void Dimmer::_send_off_cmd(void) {
//  this->_send_dimming_cmd(0, (this->brightness_ * 10) / 2);
  this->_send_switch_cmd(0);
}

void Dimmer::_send_dimming_cmd(uint16_t brightness, uint32_t previous_state) {
  uint8_t payload[6];
  memset(payload, 0, sizeof(payload));

  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;

  payload[2] = previous_state & 0xff;
  payload[3] = previous_state >> 8;
  payload[4] = previous_state >> 16;
  payload[5] = previous_state >> 24;
  
  this->_send_cmd(0x02, payload, sizeof(payload));
}

void Dimmer::_send_polling_cmd(void) {
  this->_send_cmd(PWM_GET_STATE_CMD, 0, 0);
}

void Dimmer::_send_version_cmd(void) {
  DEBUG_MSG(TAG, "Sending version command");
  this->_send_cmd(PWM_VERSION_CMD, 0, 0);
}

// set fade rate x1-x5 
void Dimmer::_send_fade_rate_cmd(uint8_t fade_rate) {
  static const uint8_t fade_rate_tab[] = {0x05, 0x05, 0x0a, 0x0f, 0x14, 0x19, 0x19, 0x19};
  this->_send_set_state_cmd(0, 2, fade_rate_tab[fade_rate & 0x07]);
}

// calibration
//01 89 20 06   00 00   02 00   0f 00   00 c0 04
//01 e6 20 06   00 00   01 00   0f 00   01 1c 04
void Dimmer::_send_set_state_cmd(uint16_t brightness, uint16_t func, uint16_t fade_rate) {
  uint8_t payload[6];

  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;

  // func = 0x00 == set current?
  // 1, 2 (used in combination with fade rate) or ...
  payload[2] = func & 0xff;
  payload[3] = func >> 8;

  // as specified in STM32 assembly
  if (fade_rate > 100)
    fade_rate = 100;
  
  payload[4] = fade_rate & 0xff;
  payload[5] = fade_rate >> 8;
  
  this->_send_cmd(PWM_SET_STATE_CMD, payload, sizeof(payload));
}

void Dimmer::_send_calibration_cmd(void) {
  uint8_t payload[72];
  memset(payload, 0, sizeof(payload));
  this->_send_cmd(0x30, payload, sizeof(payload));  
}

bool Dimmer::_read_available_cmd(void) {
  while (Serial.available() != 0) {
    DEBUG_MSG(TAG, "Yes! %u", Serial.available());

    Serial.read((char *) &this->data_[this->data_index_], 1);
    int check = this->_check_byte();
    if (check > 1) {
      // finished
      this->data_index_ = 0;
      return true;
    } else if (check == 0) {
      // wrong data
      DEBUG_MSG(TAG, "Byte %i of received data frame is invalid.", this->data_index_);
      this->data_index_ = 0;
    } else {
      // next byte
      this->data_index_++;
    }
  }
  return false;
}


bool Dimmer::_read_available_cmd_with_timeout(int timeout) {
  int start = millis();
  while (start + timeout > millis()) {
    return this->_read_available_cmd();
  }
  return false;
}

int Dimmer::_check_byte() {
  uint8_t index = this->data_index_;
  uint8_t byte = this->data_[index];

  if (index == 0) {
    return byte == 0x01;
  }

  if (index < 4) {
    return 1;
  }

  uint8_t data_length = this->data_[3];
  if ((4 + data_length + 3) > sizeof(this->data_)) {
    return 0;
  }

  if (index < 4 + data_length + 1) {
    return 1;
  }

  if (index == 4 + data_length + 1) {
    uint16_t checksum = (this->data_[index - 1] << 8 | this->data_[index]);
    if (checksum != this->_checksum(&this->data_[1], 3 + data_length)) {
      return 0;
    }
    return 1;
  }

  if (index == 4 + data_length + 2 && byte == 0x04) {
    return index;
  }
  
  return 0;
}

bool Dimmer::_parse_cmd(uint8_t expected_cmd) {
  uint8_t pos = 0;
  uint8_t id, cmd, len;
  bool ret = false;
  
  if (data_[pos++] != 0x01)
    return false;
    
  id = data_[pos++];  
  cmd = data_[pos++]; 
  len = data_[pos++]; 
  
  switch (cmd) {
    case 0x03:
      {
        uint16_t brightness = this->data_[pos + 1] << 8 | this->data_[pos + 0];

      }
      break;
    case PWM_GET_STATE_CMD:
      {
        // 1 when returning fade_rate, 0 when returning wattage, brightness?
        uint16_t unknown_0 = this->data_[pos + 1] << 8 | this->data_[pos + 0];
        
        uint16_t brightness = this->data_[pos + 3] << 8 | this->data_[pos + 2];
        uint32_t wattage = this->data_[pos + 7] << 24 | 
            this->data_[pos + 6] << 16 | 
            this->data_[pos + 5] << 8 | 
            this->data_[pos + 4];
        wattage = 1000000/wattage;
        uint32_t fade_rate = this->data_[pos + 8] << 24 | 
            this->data_[pos + 9] << 16 | 
            this->data_[pos + 10] << 8 | 
            this->data_[pos + 11];
      }
      break;  
    case PWM_VERSION_CMD:
      // returns a static, is this a version number?
      ret = data_[pos] == PWM_FIRMWARE_MINOR_VERSION && 
        data_[pos + 1] == PWM_FIRMWARE_MAJOR_VERSION;
      break;
    case 0x01:
    case 0x02:
    case 0x04:
    case PWM_SET_STATE_CMD:
    case 0x30:
    case 0x31:
      ret = (this->data_[pos] == 0x01);
      break;
  }

  return ret && (expected_cmd == 0 || expected_cmd == cmd);
}

// TODO: check resistor values on pcb, thermistor temperature is incorrect
float Dimmer::get_board_temp() {
  const double VCC = 3.3;             // 3.3v?
  const double R2 = 10000;            // 10k ohm series resistor?
  const double adc_resolution = 1023; // 10-bit adc

  const double A = 0.001129148;   // thermistor equation parameters
  const double B = 0.000234125;
  const double C = 0.0000000876741; 

  double Vout, Rth, temperature, adc_value; 

  adc_value = analogRead(A0);
  Vout = (adc_value * VCC) / adc_resolution;
  Rth = (VCC * R2 / Vout) - R2;

/*  Steinhart-Hart Thermistor Equation:
 *  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
 *  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin
  temperature = temperature - 273.15;  // Temperature in degree celsius
  return (float) temperature;
} 
