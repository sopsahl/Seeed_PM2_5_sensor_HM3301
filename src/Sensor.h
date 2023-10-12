#ifndef SEEED_PM2_5_SENSOR_HM3301_H
#define SEEED_PM2_5_SENSOR_HM3301_H

#include "Arduino.h"
#include "Wire.h"
#include "wiring_private.h"

#include "HM330XErrorCode.h"

#define DEFAULT_IIC_ADDR  0x40
#define SELECT_COMM_CMD   0X88
#define TwoWire Wire1(&sercom1, 0, 1);


class Sensor {
  
  public:

    HM330XErrorCode IIC_write_byte(uint8_t reg, uint8_t byte);

    HM330XErrorCode IIC_read_byte(uint8_t reg, uint8_t* byte);

    void set_iic_addr(uint8_t IIC_ADDR);

    HM330XErrorCode IIC_read_16bit(uint8_t start_reg, uint16_t* value);

    HM330XErrorCode IIC_write_16bit(uint8_t reg, uint16_t value);

    HM330XErrorCode IIC_read_bytes(uint8_t start_reg, uint8_t* data, uint32_t data_len);

    HM330XErrorCode IIC_SEND_CMD(uint8_t CMD);

    //From the Initial Sensor code

    HM330X(uint8_t IIC_ADDR = DEFAULT_IIC_ADDR);

    HM330XErrorCode init();

    HM330XErrorCode read_sensor_value(uint8_t* data, uint32_t data_len);

  private:

    uint8_t _IIC_ADDR;

    //From the Initial Sensor Code

    HM330XErrorCode select_comm();
};


#endif //SEEED_PM2_5_SENSOR_HM3301_H