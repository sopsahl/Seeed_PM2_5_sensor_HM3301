#include "Sensor.h"
#include "Wire.h"
#include "wiring_private.h"

TwoWire Wire1(&sercom1, 0, 1);


/**
    @brief I2C write byte
    @param reg :Register address of operation object
    @param byte :The byte to be wrote.
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode Sensor::IIC_write_byte(uint8_t reg, uint8_t byte) {
    int ret = 0;
    Wire1.beginTransmission(_IIC_ADDR);
    Wire1.write(reg);
    Wire1.write(byte);
    ret = Wire1.endTransmission();
    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}

/**
    @brief I2C write 16bit value
    @param reg: Register address of operation object
    @param value: The 16bit value to be wrote .
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode Sensor::IIC_write_16bit(uint8_t reg, uint16_t value) {
    int ret = 0;
    Wire1.beginTransmission(_IIC_ADDR);
    Wire1.write(reg);

    Wire1.write((uint8_t)(value >> 8));
    Wire1.write((uint8_t) value);
    ret = Wire1.endTransmission();
    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}

/**
    @brief I2C read byte
    @param reg: Register address of operation object
    @param byte: The byte to be read in.
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode Sensor::IIC_read_byte(uint8_t reg, uint8_t* byte) {
    uint32_t time_out_count = 0;
    Wire1.beginTransmission(_IIC_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission(false);

    Wire1.requestFrom(_IIC_ADDR, (uint8_t) 1);
    while (1 != Wire1.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    *byte = Wire1.read();
    return NO_ERROR;
}

/**
    @brief I2C read 16bit value
    @param reg: Register address of operation object
    @param byte: The 16bit value to be read in.
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode Sensor::IIC_read_16bit(uint8_t start_reg, uint16_t* value) {
    uint32_t time_out_count = 0;
    uint8_t val = 0;
    *value = 0;
    Wire1.beginTransmission(_IIC_ADDR);
    Wire1.write(start_reg);
    Wire1.endTransmission(false);

    Wire1.requestFrom(_IIC_ADDR, sizeof(uint16_t));
    while (sizeof(uint16_t) != Wire1.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    val = Wire1.read();
    *value |= (uint16_t) val << 8;
    val = Wire1.read();
    *value |= val;
    return NO_ERROR;
}

/**
    @brief I2C read some bytes
    @param reg: Register address of operation object
    @param data: The buf  to be read in.
    @param data_len: The length of buf need to read in.
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode Sensor::IIC_read_bytes(uint8_t start_reg, uint8_t* data, uint32_t data_len) {
    HM330XErrorCode ret = NO_ERROR;
    uint32_t time_out_count = 0;
    Wire1.beginTransmission(_IIC_ADDR);
    Wire1.write(start_reg);
    Wire1.endTransmission(false);

    Wire1.requestFrom(_IIC_ADDR, data_len);
    while (data_len != Wire1.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }

    for (int i = 0; i < data_len; i++) {
        data[i] = Wire1.read();
    }
    return ret;
}

//Code from 

/**
    @brief change the I2C address from default.
    @param IIC_ADDR: I2C address to be set
*/
void Sensor::set_iic_addr(uint8_t IIC_ADDR) {
    _IIC_ADDR = IIC_ADDR;
}

HM330XErrorCode Sensor::IIC_SEND_CMD(uint8_t CMD) {
    Wire1.beginTransmission(_IIC_ADDR);
    Wire1.write(CMD);
    byte ret = Wire1.endTransmission();
    if (ret == 0) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}


Sensor::HM330X(uint8_t IIC_ADDR) {
    set_iic_addr(IIC_ADDR);
}

HM330XErrorCode Sensor::select_comm() {
    return IIC_SEND_CMD(SELECT_COMM_CMD);
}

HM330XErrorCode Sensor::init() {
    Wire1.begin();
    pinPeripheral(0, PIO_SERCOM);
    pinPeripheral(1, PIO_SERCOM);
    return select_comm();
}

HM330XErrorCode Sensor::read_sensor_value(uint8_t* data, uint32_t data_len) {
    uint32_t time_out_count = 0;
    HM330XErrorCode ret = NO_ERROR;
    Wire1.requestFrom(0x40, 29);
    while (data_len != Wire1.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    for (int i = 0; i < data_len; i++) {
        data[i] = Wire1.read();
    }
    return ret;
}
