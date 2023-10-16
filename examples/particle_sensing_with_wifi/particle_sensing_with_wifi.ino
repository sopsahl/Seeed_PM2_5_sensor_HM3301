#include <WiFiNINA.h>
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>

#include <Seeed_HM330X.h>

HM330X sensor;
uint8_t buf[30];
uint16_t Particle_readings[6];
char query[256];

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
char table[]    = ""; //FILL THIS OUT

WiFiClient client;
MySQL_Connection conn((Client *)&client);
MySQL_Cursor* cursor;



void setup() {
  // Setup Serial
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
  //Serial.println("----------------------------------------");

  // Test MySQL connection
  if (conn.connect(server_addr, server_port, user, password, database)) {
     Serial.println("Connected to MySQL server");
   }
   else {
    Serial.println("Connection failed");
   }
  cursor = new MySQL_Cursor(&conn);
  Serial.begin(115200);
  delay(100);
  Serial.println("Serial start");
  if (sensor.init()) {
      Serial.println("HM330X init failed!!");
      while (1);
  }
}

void loop() {

  if (conn.connect(server_addr, server_port, user, password, database)) {
    delay(1000);
    MySQL_Cursor *cur_mem = new MySQL_Cursor(&conn);
    
    //Particle Sensor code
    
    if (sensor.read_sensor_value(buf, 29)) {
        Serial.println("HM330X read result failed!!");
    }
    
    generate_readings(buf, Particle_readings);
    
    sprintf(query, "INSERT INTO %s (PM1_SPM, PM2_5_SPM, PM10_SPM, PM1_AE, PM2_5_AE, PM10_AE) VALUES (%u, %u, %u, %u, %u, %u)", table, Particle_readings[0], Particle_readings[1], Particle_readings[2], Particle_readings[3], Particle_readings[4], Particle_readings[5]);
    Serial.println(query);
    cur_mem->execute(query);
    delete cur_mem;
  }
  
    //Serial.println("Connection failed.");
  conn.close();
  delay(30000);
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

void generate_readings(uint8_t* data, uint16_t* reads) {
  for (int i = 2; i < 8; i++) {
    reads[i-2] = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
  }
}

