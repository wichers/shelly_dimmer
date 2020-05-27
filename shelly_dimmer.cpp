#include "shelly_dimmer.h"
#include "stm32flash.h"
#include "esphome/core/log.h"

namespace esphome {
namespace shd { 

#include "stm_v1.6.0.h"
  
// #define SHELLY_HW_DIMMING

static const uint8_t SHD_DRIVER_MAJOR_VERSION = 0x00; 
static const uint8_t SHD_DRIVER_MINOR_VERSION = 0x04; 

static const uint8_t SHD_SWITCH_CMD = 0x01; 
static const uint8_t SHD_GET_STATE_CMD = 0x10; 
static const uint8_t SHD_VERSION_CMD = 0x11; 
static const uint8_t SHD_SET_STATE_CMD = 0x20; 

static const uint8_t SHD_START_BYTE = 0x01; 
static const uint8_t SHD_END_BYTE = 0x04; 

static const uint8_t SHD_ACK_TIMEOUT = 200 ; // 200 ms ACK timeout

static const uint8_t GPIO_SHELLY_DIMMER_BOOT0 = 4;
static const uint8_t GPIO_SHELLY_DIMMER_RST_INV = 5;

static const char *TAG = "shd";

/*********************************************************************************************\
 * Helper Functions
\*********************************************************************************************/

//
// changeUIntScale
// Change a value for range a..b to c..d, using only unsigned int math
//
// New version, you don't need the "to_min < to_max" precondition anymore
//
// PRE-CONDITIONS (if not satisfied, you may 'halt and catch fire')
//    from_min < from_max  (not checked)
//    from_min <= num <= from-max  (chacked)
// POST-CONDITIONS
//    to_min <= result <= to_max
//
uint16_t ShdComponent::changeUIntScale(uint16_t inum, uint16_t ifrom_min, uint16_t ifrom_max,
                                       uint16_t ito_min, uint16_t ito_max) {
  // guard-rails
  if (ifrom_min >= ifrom_max) {
    if (ito_min > ito_max) {
      return ito_max;
    } else {
      return ito_min;  // invalid input, return arbitrary value
    }
  }
  // convert to uint31, it's more verbose but code is more compact
  uint32_t num = inum;
  uint32_t from_min = ifrom_min;
  uint32_t from_max = ifrom_max;
  uint32_t to_min = ito_min;
  uint32_t to_max = ito_max;

  // check source range
  num = (num > from_max ? from_max : (num < from_min ? from_min : num));

  // check to_* order
  if (to_min > to_max) {
    // reverse order
    num = (from_max - num) + from_min;
    to_min = ito_max;
    to_max = ito_min;
  }

  uint32_t numerator = (num - from_min) * (to_max - to_min);
  uint32_t result;
  if (numerator >= 0x80000000L) {
    // don't do rounding as it would create an overflow
    result = numerator / (from_max - from_min) + to_min;
  } else {
    result = (((numerator * 2) / (from_max - from_min)) + 1) / 2 + to_min;
  }
  return (uint32_t) (result > to_max ? to_max : (result < to_min ? to_min : result));
} 

uint16_t ShdComponent::checksum(uint8_t *buf, int len)
{
    uint16_t chksm = 0;
    for (uint8_t i = 0; i < len; i++)
        chksm += buf[i];
    return chksm;
}

int ShdComponent::check_byte()
{
    uint8_t index = this->byte_counter;
    uint8_t byte = this->buffer[index];

    if (index == 0)
        return byte == SHD_START_BYTE;

    if (index < 4)
        return 1;

    uint8_t data_length = this->buffer[3];
    if ((4 + data_length + 3) > SHD_BUFFER_SIZE)
        return 0;

    if (index < 4 + data_length + 1)
        return 1;

    if (index == 4 + data_length + 1)
    {
        uint16_t chksm = (this->buffer[index - 1] << 8 | this->buffer[index]);
        if (chksm != checksum(&this->buffer[1], 3 + data_length))
            return 0;

        return 1;
    }

    if (index == 4 + data_length + 2 && byte == SHD_END_BYTE)
        return index;
    
    return 0;
}

/*********************************************************************************************\
 * Internal Functions
\*********************************************************************************************/

bool ShdComponent::SerialSend(const uint8_t data[], uint16_t len)
{
    int retries = 3;

#ifdef ESPHOME_LOG_HAS_VERY_VERBOSE
    char log_data[256];
    uint32_t buffer_offset = 0;

    buffer_offset += sprintf(log_data, "SHD: Tx Packet:");
    for (int i = 0; i < len; i++)
        buffer_offset += sprintf(log_data, " %02x", data[i]);
    ESP_LOGVV(TAG, "%s", log_data);
#endif

    while (retries--)
    {
        Serial.write(data, len);
        Serial.flush();

        // wait for any response
        uint32_t snd_time = millis();
        while (millis() - snd_time < SHD_ACK_TIMEOUT)
        {
            if (this->SerialInput())
                return true;

            delay(1);
        }

        // timeout
        ESP_LOGD(TAG, "SHD: serial send timeout");
    }
    return false;
}

bool ShdComponent::SendCmd(uint8_t cmd, uint8_t *payload, uint8_t len)
{
    uint8_t data[4 + 72 + 3]; // maximum payload for 0x30 packet is 72
    uint16_t chksm;
    uint8_t pos = 0;

    data[0] = SHD_START_BYTE;
    data[1] = this->counter++;
    data[2] = cmd;
    data[3] = len;

    pos += 4;

    if (payload)
    {
        memcpy(data + 4, payload, len);
        pos += len;
    }

    // calculate checksum from id and onwards
    chksm = checksum(data + 1, 3 + len); 
    data[pos++] = chksm >> 8;
    data[pos++] = chksm & 0xff;
    data[pos++] = SHD_END_BYTE;

    return this->SerialSend(data, pos);
}

bool ShdComponent::SetBrightness(uint16 brightness)
{
    brightness = this->changeUIntScale(brightness, 0, 255, 0, 1000);

    uint8_t payload[2];

    payload[0] = brightness & 0xff;
    payload[1] = brightness >> 8;

    return this->SendCmd(SHD_SWITCH_CMD, payload, sizeof(payload));
}

bool ShdComponent::SendFadeRate(uint16_t brightness, uint8_t fade_rate)
{
    return this->SendSetState(brightness, 2, fade_rate);
}

bool ShdComponent::SendSetState(uint16_t brightness, uint16_t func, uint16_t fade_rate)
{
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

    return this->SendCmd(SHD_SET_STATE_CMD, payload, sizeof(payload));
}

void ShdComponent::SyncState()
{
    ESP_LOGD(TAG, "SHD: Set Brightness Want %d, Is %d", this->req_brightness, this->dimmer.brightness);

#ifdef SHELLY_HW_DIMMING
    ESP_LOGD(TAG, "SHD: Set Fade Want %d, Is %d", Settings.light_speed, this->dimmer.fade_rate);

    // TODO(jamesturton): HW dimming seems to conflict with SW dimming. See how
    // we can disbale SW dimming when using HW dimming.
    if (Settings.light_speed != this->dimmer.fade_rate)
    {
        this->SendFadeRate(this->req_brightness, Settings.light_speed);
        this->DebugState();
    }
    else
#endif
    if (this->req_brightness != this->dimmer.brightness)
    {
        this->SetBrightness(this->req_brightness);
        this->DebugState();
    }
}

void ShdComponent::DebugState()
{
    ESP_LOGD(TAG, "SHD: MCU v%d.%d, Brightness:%d(%d%%), Power:%d, Fade:%d",
                        this->dimmer.version_major, this->dimmer.version_minor,
                        this->dimmer.brightness,
                        this->changeUIntScale(this->dimmer.brightness, 0, 255, 0, 100),
                        this->dimmer.power,
                        this->dimmer.fade_rate);
}

bool ShdComponent::PacketProcess()
{
    uint8_t pos = 0;
    uint8_t id, cmd, len;
    bool ret = false;
    
    if (this->buffer[pos++] != SHD_START_BYTE)
        return false;
        
    id = this->buffer[pos++];
    cmd = this->buffer[pos++];
    len = this->buffer[pos++];
    
    switch (cmd)
    {
        case 0x03:
            {
                uint16_t brightness = this->buffer[pos + 1] << 8 | this->buffer[pos + 0];
                brightness = this->changeUIntScale(brightness, 0, 1000, 0, 255);
            }
            break;
        case SHD_GET_STATE_CMD:
            {
                // 1 when returning fade_rate, 0 when returning wattage, brightness?
                uint16_t unknown_0 = this->buffer[pos + 1] << 8 | this->buffer[pos + 0];
                
                uint16_t brightness = this->buffer[pos + 3] << 8 | this->buffer[pos + 2];
                brightness = this->changeUIntScale(brightness, 0, 1000, 0, 255);

                uint32_t wattage_raw = this->buffer[pos + 7] << 24 | 
                        this->buffer[pos + 6] << 16 | 
                        this->buffer[pos + 5] << 8 | 
                        this->buffer[pos + 4];
                float wattage = 0;
                if (wattage_raw > 0)
                    wattage = 1000000 / wattage_raw;
                uint32_t fade_rate = this->buffer[pos + 8];

                ESP_LOGD(TAG, "SHD: ShdPacketProcess: Brightness:%d Power:%d Fade:%d", brightness, wattage_raw, fade_rate);
                this->dimmer.brightness = brightness;
                this->dimmer.power = wattage_raw;
                this->dimmer.fade_rate = fade_rate;
            }
            break;  
        case SHD_VERSION_CMD:
            {
                // returns a static, is this a version number?
                ret = this->buffer[pos] == SHD_FIRMWARE_MINOR_VERSION && 
                    this->buffer[pos + 1] == SHD_FIRMWARE_MAJOR_VERSION;

                this->dimmer.version_minor = this->buffer[pos];
                this->dimmer.version_major = this->buffer[pos + 1];
            }
            break;
        case SHD_SWITCH_CMD:
        case 0x02:
        case 0x04:
        case SHD_SET_STATE_CMD:
        case 0x30:
        case 0x31:
            {
                ret = (this->buffer[pos] == 0x01);
            }
            break;
    }

    return ret;
}

#if 0
bool ShdComponent::SetChannels()
{
    // upscale and then downscale to account for rounding errors
    uint16_t brightness = 100;
    brightness = this->changeUIntScale(brightness, 0, 255, 0, 1000);
    brightness = this->changeUIntScale(brightness, 0, 1000, 0, 255);
    this->req_brightness = brightness;

    this->DebugState();

    return this->SyncState();
}
#endif

void ShdComponent::ResetToAppMode()
{
    ESP_LOGD(TAG, "SHD: Request co-processor reset configuration, PIN %d to High", GPIO_SHELLY_DIMMER_RST_INV);

    pinMode(GPIO_SHELLY_DIMMER_BOOT0, OUTPUT);
    digitalWrite(GPIO_SHELLY_DIMMER_BOOT0, LOW);

    pinMode(GPIO_SHELLY_DIMMER_RST_INV, OUTPUT);
    digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, LOW);
    delay(50);
    
    // clear in the receive buffer
    while (Serial.available())
        Serial.read();
    
    digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, HIGH); // pull out of reset
    delay(50); // wait 50ms fot the co-processor to come online
}

void ShdComponent::ResetToDFUMode()
{
    ESP_LOGD(TAG, "SHD: Request co-processor reset configuration, PIN %d to High", GPIO_SHELLY_DIMMER_RST_INV);

    pinMode(GPIO_SHELLY_DIMMER_BOOT0, OUTPUT);
    digitalWrite(GPIO_SHELLY_DIMMER_BOOT0, HIGH);

    pinMode(GPIO_SHELLY_DIMMER_RST_INV, OUTPUT);
    digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, LOW);
    delay(50);
    
    // clear in the receive buffer
    while (Serial.available())
        Serial.read();
    
    digitalWrite(GPIO_SHELLY_DIMMER_RST_INV, HIGH); // pull out of reset
    delay(50); // wait 50ms fot the co-processor to come online
}

void ShdComponent::loop()
{
    // poll every two seconds for new data
    if (this->start_time == 0 || millis() - start_time > 1000  * 2) {
      this->start_time = millis();

      ESP_LOGD(TAG, "SHD: Poll");
      this->SendCmd(SHD_GET_STATE_CMD, 0, 0);
      this->SyncState();
    }
}

bool ShdComponent::SendVersion()
{
    ESP_LOGI(TAG, "SHD: Sending version command");
    return this->SendCmd(SHD_VERSION_CMD, 0, 0);
}

bool ShdComponent::UpdateFirmware(const uint8_t data[], unsigned int size)
{
    bool ret = true;
    stm32_t *stm = stm32_init(&Serial, STREAM_SERIAL, 1); 
    if (stm)
    {
  		off_t 	offset = 0;
    	uint8_t		buffer[256];
    	unsigned int	len;
    	const uint8_t *p_st = data;
    	uint32_t	addr, start, end;
    	stm32_err_t s_err;

      stm32_erase_memory(stm, 0, STM32_MASS_ERASE);

   		addr = stm->dev->fl_start;
   		end = addr + size;
  		while(addr < end && offset < size) 
  		{
    			uint32_t left	= end - addr;
    			len		= sizeof(buffer) > left ? left : sizeof(buffer);
    			len		= len > size - offset ? size - offset : len;

    			if (len == 0) 
    			{
    					break;
    			}
        
    			memcpy(buffer, p_st, len);
    			p_st += len;
    	
    			s_err = stm32_write_memory(stm, addr, buffer, len);
    			if (s_err != STM32_ERR_OK) 
    			{
    			    ret = false;
              break;
    			}

    			addr	+= len;
    			offset	+= len;
      }
      stm32_close(stm);
    }
    return ret;
}

void ShdComponent::setup()
{
    ESP_LOGD(TAG, "SHD: Shelly Dimmer Driver v%d.%d", SHD_DRIVER_MAJOR_VERSION, SHD_DRIVER_MINOR_VERSION);

    Serial.begin(115200, SERIAL_8N1);
    Serial.flush();

    this->ResetToAppMode();
    bool got_version = this->SendVersion();
    if (!got_version || (got_version && 
      (this->dimmer.version_minor != SHD_FIRMWARE_MINOR_VERSION || 
      this->dimmer.version_major != SHD_FIRMWARE_MAJOR_VERSION))) 
    {
      ESP_LOGI(TAG, "We need to update the firmware to %u.%u", SHD_FIRMWARE_MAJOR_VERSION, SHD_FIRMWARE_MINOR_VERSION);
      
      Serial.end();
      Serial.begin(115200, SERIAL_8E1);
      this->ResetToDFUMode();
      this->UpdateFirmware(stm_firmware, sizeof(stm_firmware));
      Serial.end();

      this->ResetToAppMode();
      Serial.begin(115200, SERIAL_8N1);
      
      this->SendVersion();
    }
    delay(100);
    this->SyncState();
}

void ShdComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Shelly Dimmer STM32:");
    ESP_LOGD(TAG, "Version Major %u", this->dimmer.version_major);
    ESP_LOGD(TAG, "Version Minor %u", this->dimmer.version_minor);
}

bool ShdComponent::SerialInput()
{
    while (Serial.available())
    {
        yield();
        uint8_t serial_in_byte = Serial.read();
        this->buffer[this->byte_counter] = serial_in_byte;
        
        int check = this->check_byte();

        if (check > 1)
        {
            // finished
            this->byte_counter++;

#ifdef ESPHOME_LOG_HAS_VERY_VERBOSE
            char log_data[256];
            uint32_t buffer_offset = 0;

            buffer_offset += sprintf(log_data, "SHD: RX Packet:");
            for (int i = 0; i < this->byte_counter; i++)
                buffer_offset += sprintf(log_data, " %02x", this->buffer[i]);
            ESP_LOGVV(TAG, "%s", log_data);
#endif
            this->byte_counter = 0;

            this->PacketProcess();

            return true;
        }
        else if (check == 0)
        {
            // wrong data
            ESP_LOGD(TAG, "SHD: Byte %i of received data frame is invalid", this->byte_counter);
            this->byte_counter = 0;
        }
        else
        {
            this->byte_counter++;
        }
        
    }
    return false;
}

/*********************************************************************************************\
 * Driver Interface
\*********************************************************************************************/

#if 0
bool Xdrv32(uint8_t function)
{
    bool result = false;

    if (SHELLY_DIMMER == my_module_type)
    {
        switch (function)
        {
        case FUNC_EVERY_SECOND:
            this->Poll();
            break;
        case FUNC_MODULE_INIT:
            result = this->ModuleSelected();
            break;
        case FUNC_INIT:
            this->Init();
            break;
        case FUNC_SET_CHANNELS:
            result = this->SetChannels();
            break;
        }
    }
    return result;
}
#endif

}  // namespace shd
}  // namespace esphome
