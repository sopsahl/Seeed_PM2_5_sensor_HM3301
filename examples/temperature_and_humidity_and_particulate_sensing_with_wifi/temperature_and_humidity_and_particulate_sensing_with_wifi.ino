// Code for the Seeed HM330X Particulate Sensor and SHT30 Temperature and Humidity Sensor for use in the Downstairs lab 
// Should upload the Atmospheric and standard particulate concentrations at 1, 2.5, and 10 micron resolution
// Should upload the Temperature (in deg C) and Humidity (%) 
// Must update the location, or column in the database to upload files into (found in table)

#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Seeed_HM330X.h>
#include <WiFiNINA.h>
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>

TwoWire Wire1(&sercom1, 8, 9);
Adafruit_SHT31 sht31 = Adafruit_SHT31(&Wire1);

HM330X sensor;

uint8_t buf[30];
uint16_t readings[6];
char query[128];

// Network Info
char ssid[] = "TP-Link_9D04";                                              
char pass[] = "qn&ngpwd"; 
int status = WL_IDLE_STATUS;      // the Wifi radio's status

// Database Adress
IPAddress server_addr(18,25,16,44); 
uint16_t server_port = 3307;
// Database Account
char user[]         = "lab";
char password[]     = "qn&ngpwdDB1";
//Database Name
char database[] = "qnndb";
char table[]    = "particulate_aja_room";

WiFiClient client;
MySQL_Connection conn((Client *)&client);
MySQL_Cursor* cursor;


void setup() {
  Serial.begin(9600);

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  Serial.println("----------------------------------------");
  printData();
  Serial.println("----------------------------------------");

  // Test MySQL connection
  if (conn.connect(server_addr, server_port, user, password, database)) {
    Serial.println("Connected to MySQL server");
  }
  else {
    Serial.println("Connection failed");
  }

  cursor = new MySQL_Cursor(&conn);

  delay(10000);

  Serial.begin(115200);
  delay(100);

  // TEMPERATURE/HUMIDITY SENSOR INITIALIZATION
  if (! sht31.begin(0x44)) {   
    Serial.println("Couldn't find SHT31");
    while (1);
  }

  
  // PARTICLE SENSOR INITIALIZATION
  if (sensor.init()) {
      Serial.println("HM330X init failed!!");
      while (1);
  }

  Serial.println("Setup Complete");

}


void loop() {
  
  //Particle Sensor code
  
  if (sensor.read_sensor_value(buf, 29)) {
      Serial.println("HM330X read result failed!!");
  }
  generate_readings(buf, readings);

  delay(100);

  //Temperature/Humidity Sensor code

  float Temperature = sht31.readTemperature();
  float Humidity = sht31.readHumidity();

  sprintf(query, "INSERT INTO %s (humidity, temperature, PM1_SPM, PM2_5_SPM, PM10_SPM, PM1_AE, PM2_5_AE, PM10_AE) VALUES (%s, %s, %u, %u, %u, %u, %u, %u)", table, String(Humidity,2).c_str(), String(Temperature,2).c_str(), readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);
  Serial.println(query);

  while (status != WL_CONNECTED) {
    Serial.print("Disconnected-----Attempting to connect to network: ");
    Serial.println(ssid);
    // // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // // wait 10 seconds for connection:
    delay(10000);
  }
  
  
  if (conn.connected()) {

    MySQL_Cursor *cur_mem = new MySQL_Cursor(&conn);

    //Upload into Database

    cur_mem->execute(query);
    Serial.println("query added to database");
    delete cur_mem;
  }

  else {
    conn.close();

    delay(60000);

    MySQL_Connection conn((Client *)&client);

    if (conn.connect(server_addr, server_port, user, password, database)) {
    Serial.println("Connected to MySQL server");
    }
    
  }


  delay(60000);

}


void generate_readings(uint8_t* data, uint16_t* reads) {
  uint16_t value = 0;
  for (int i = 2; i < 8; i++) {
    reads[i-2] = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
  }
}

void printData() {
  Serial.println("Board Information:");
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}