#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Seeed_HM330X.h>

TwoWire Wire1(&sercom1, 8, 9);
Adafruit_SHT31 sht31 = Adafruit_SHT31(&Wire1);

HM330X sensor;

uint8_t buf[30];
uint16_t readings[6];
char query[128];
char table_particulate[] = "particulate levels";
char table_temphum[] = "temperature/humidity levels";


void setup() {
  Serial.begin(9600);

  Serial.println("Serial start");

  // Temperature and humidity sensor initialization
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1);
  }

  // Particulate sensor initialization
  if (sensor.init()) {
      Serial.println("HM330X init failed!!");
      while (1);
  }

}

void loop() {
  
  //Particle Sensor code
    
    if (sensor.read_sensor_value(buf, 29)) {
        Serial.println("HM330X read result failed!!");
    }
    
    generate_readings(buf, readings);

    sprintf(query, "INSERT INTO %s (PM1_SPM, PM2_5_SPM, PM10_SPM, PM1_AE, PM2_5_AE, PM10_AE) VALUES (%u, %u, %u, %u, %u, %u)", table_particulate, readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);
    Serial.println(query);
  

    delay(5000);


    //Temperature/Humidity Sensor code

    float Temperature = sht31.readTemperature();
    float Humidity = sht31.readHumidity();

    sprintf(query, "INSERT INTO %s (humidity, temperature) VALUES (%s, %s)", table_temphum, String(Humidity,2).c_str(), String(Temperature,2).c_str());    
    Serial.println(query);

}


void generate_readings(uint8_t* data, uint16_t* reads) {
  uint16_t value = 0;
  for (int i = 2; i < 8; i++) {
    reads[i-2] = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
  }
}



