#ifndef __SHELLY_DIMMER__H_
#define __SHELLY_DIMMER__H_

#include "esphome.h"
#include "esphome/core/component.h" 
#include "esphome/components/switch/switch.h"
#include "esphome/core/log.h"

#include "stm32flash.h"

namespace esphome {
namespace shd {

static const uint8_t SHD_BUFFER_SIZE = 0xff;

class ShdComponent : public Component, public switch_::Switch
{
public:

    void loop() override;
    void setup() override;
    void write_state(bool state) override {
      this->publish_state(state);
    }

    void dump_config() override;

    typedef struct
    {
        uint8_t version_major = 0;
        uint8_t version_minor = 0;

        uint32_t brightness = 0;
        uint32_t power = 0;
        uint32_t fade_rate = 0;
    } shd_dimmer_t;

protected:

    uint16_t changeUIntScale(uint16_t inum, uint16_t ifrom_min, uint16_t ifrom_max,
                                           uint16_t ito_min, uint16_t ito_max);
    uint16_t checksum(uint8_t *buf, int len);
    int check_byte();
    bool SerialSend(const uint8_t data[] = nullptr, uint16_t len = 0);
    bool SendCmd(uint8_t cmd, uint8_t *payload, uint8_t len);
    bool SetBrightness(uint16 brightness);
    bool SendFadeRate(uint16_t brightness, uint8_t fade_rate);
    bool SendSetState(uint16_t brightness, uint16_t func, uint16_t fade_rate);
    void SyncState();
    void DebugState();
    bool PacketProcess();
    void ResetToAppMode();
    void ResetToDFUMode();
    bool SendVersion();
    bool SerialInput();
    bool UpdateFirmware(const uint8_t data[], unsigned int size);

private:

    uint8_t buffer[SHD_BUFFER_SIZE]; // Serial receive buffer
    int byte_counter = 0;      // Index in serial receive buffer
    uint8_t req_brightness = 0;
    shd_dimmer_t dimmer;
    uint32_t start_time = 0;
    uint8_t counter = 1;        // Packet counter
};

}  // namespace shd
}  // namespace esphome

#endif
