/*
    Seeed_HM330X.cpp
    Driver for Seeed PM2.5 Sensor(HM300)

    Copyright (c) 2018 Seeed Technology Co., Ltd.
    Website    : www.seeed.cc
    Author     : downey
    Create Time: August 2018
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Seeed_HM330X.h"

HM330X::HM330X(uint8_t IIC_ADDR) {
    set_iic_addr(IIC_ADDR);
    twoWire myWire(&sercom2, 11, 12);
}

HM330XErrorCode HM330X::select_comm() {
    return IIC_SEND_CMD(SELECT_COMM_CMD);
}

HM330XErrorCode HM330X::init() {
    myWire.begin();
    return select_comm();
}

HM330XErrorCode HM330X::read_sensor_value(uint8_t* data, uint32_t data_len) {
    uint32_t time_out_count = 0;
    HM330XErrorCode ret = NO_ERROR;
    myWire.requestFrom(0x40, 29);
    while (data_len != myWire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    for (int i = 0; i < data_len; i++) {
        data[i] = Wire.read();
    }
    return ret;
}

/**
    @brief I2C write byte
    @param reg :Register address of operation object
    @param byte :The byte to be wrote.
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode HM330X::IIC_write_byte(uint8_t reg, uint8_t byte) {
    int ret = 0;
    myWire.beginTransmission(_IIC_ADDR);
    myWire.write(reg);
    myWire.write(byte);
    ret = myWire.endTransmission();
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
HM330XErrorCode HM330X::IIC_write_16bit(uint8_t reg, uint16_t value) {
    int ret = 0;
    myWire.beginTransmission(_IIC_ADDR);
    myWire.write(reg);

    myWire.write((uint8_t)(value >> 8));
    myWire.write((uint8_t) value);
    ret = myWire.endTransmission();
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
HM330XErrorCode HM330X::IIC_read_byte(uint8_t reg, uint8_t* byte) {
    uint32_t time_out_count = 0;
    myWire.beginTransmission(_IIC_ADDR);
    myWire.write(reg);
    myWire.endTransmission(false);

    myWire.requestFrom(_IIC_ADDR, (uint8_t) 1);
    while (1 != myWire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    *byte = myWire.read();
    return NO_ERROR;
}

/**
    @brief I2C read 16bit value
    @param reg: Register address of operation object
    @param byte: The 16bit value to be read in.
    @return result of operation,non-zero if failed.
*/
HM330XErrorCode HM330X::IIC_read_16bit(uint8_t start_reg, uint16_t* value) {
    uint32_t time_out_count = 0;
    uint8_t val = 0;
    *value = 0;
    myWire.beginTransmission(_IIC_ADDR);
    myWire.write(start_reg);
    myWire.endTransmission(false);

    myWire.requestFrom(_IIC_ADDR, sizeof(uint16_t));
    while (sizeof(uint16_t) != myWire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    val = myWire.read();
    *value |= (uint16_t) val << 8;
    val = myWire.read();
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
HM330XErrorCode HM330X::IIC_read_bytes(uint8_t start_reg, uint8_t* data, uint32_t data_len) {
    HM330XErrorCode ret = NO_ERROR;
    uint32_t time_out_count = 0;
    myWire.beginTransmission(_IIC_ADDR);
    myWire.write(start_reg);
    myWire.endTransmission(false);

    myWire.requestFrom(_IIC_ADDR, data_len);
    while (data_len != myWire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }

    for (int i = 0; i < data_len; i++) {
        data[i] = myWire.read();
    }
    return ret;
}

/**
    @brief change the I2C address from default.
    @param IIC_ADDR: I2C address to be set
*/
void HM330X::set_iic_addr(uint8_t IIC_ADDR) {
    _IIC_ADDR = IIC_ADDR;
}

HM330XErrorCode HM330X::IIC_SEND_CMD(uint8_t CMD) {
    myWire.beginTransmission(_IIC_ADDR);
    myWire.write(CMD);
    byte ret = myWire.endTransmission();
    if (ret == 0) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}
