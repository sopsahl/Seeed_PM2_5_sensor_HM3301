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
char ssid[] = ""; //FILL THIS OUT                                        
char pass[] = ""; //FILL THIS OUT
int status = WL_IDLE_STATUS;      // the Wifi radio's status

// Database Adress
// IPAddress server_addr(); //FILL THIS OUT
// uint16_t server_port = ; //FILL THIS OUT
// Database Account
char user[]         = ""; //FILL THIS OUT
char password[]     = ""; //FILL THIS OUT
//Database Name
char database[] = ""; //FILL THIS OUT
char table_particulate[]    = ""; //FILL THIS OUT
char table_temphum[] = ""; //FILL THIS OUT

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
  Serial.begin(115200);

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

}


void loop() {
  if (conn.connect(server_addr, server_port, user, password, database)) {
    delay(1000);


    //Particle Sensor code
    
    if (sensor.read_sensor_value(buf, 29)) {
        Serial.println("HM330X read result failed!!");
    }
    
    generate_readings(buf, readings);

    MySQL_Cursor *cur_mem = new MySQL_Cursor(&conn);
    sprintf(query, "INSERT INTO %s (PM1_SPM, PM2_5_SPM, PM10_SPM, PM1_AE, PM2_5_AE, PM10_AE) VALUES (%u, %u, %u, %u, %u, %u)", table_particulate, readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);
    Serial.println(query);
    cur_mem->execute(query);
  

    delay(5000);


    //Temperature/Humidity Sensor code

    float Temperature = sht31.readTemperature();
    float Humidity = sht31.readHumidity();


    sprintf(query, "INSERT INTO %s (humidity, temperature) VALUES (%s, %s)", table_temphum, String(Humidity,2).c_str(), String(Temperature,2).c_str());    
    Serial.println(query);
    cur_mem->execute(query);
    delete cur_mem;

  }

  conn.close();
  delay(30000);

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