/*
    basic_demo.ino
    Example for Seeed PM2.5 Sensor(HM300)

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

#include <Seeed_HM330X.h>
#include <Wire.h>


HM330X sensor;
uint8_t buf[30];
uint16_t readings[6];
char query[128];

void generate_readings(uint8_t* data, uint16_t* reads) {
  uint16_t value = 0;
  for (int i = 2; i < 8; i++) {
    reads[i-2] = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
  }
}


/*30s*/
void setup() {
  Serial.begin(115200);
  delay(100);  

    Serial.println("Serial start");
    if (sensor.init()) {
        Serial.println("HM330X init failed!!");
        while (1);
    }


}


void loop() {
    if (sensor.read_sensor_value(buf, 29)) {
        Serial.println("HM330X read result failed!!");
    }

    generate_readings(buf, readings);
    sprintf(query, "%u, %u, %u, %u, %u, %u", readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);
    sprintf(query, "INSERT INTO particulate_aja_room (PM1_SPM, PM2_5_SPM, PM10_SPM, PM1_AE, PM2_5_AE, PM10_AE) VALUES (%u, %u, %u, %u, %u, %u)", readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);

    Serial.println(query);
    delay(5000);
}
