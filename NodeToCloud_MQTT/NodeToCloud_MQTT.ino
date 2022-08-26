#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <WiFiUdp.h>

#define DEBUG_PRINTER Serial

#define DHTPIN D3     // what digital pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

/************************* WiFi Access Point *********************************/

#define WLAN_SSID  //WLAN SSID
#define WLAN_PASS  //SSID Password

/************************* Adafruit.io Setup *********************************/

#define ARB_SERVER      "mqtt-tr.iot-ignite.com"
#define ARB_SERVERPORT  8883                   // use 8883 for SSL
#define ARB_USERNAME    //MQTT USERNAME
#define ARB_PW          //MQTT PASSWORD

/************************* Device, Node and Sensors **************************/
//Device
#define DEVICE_ID //MQTT DEVICE ID

// Node
#define NODE_ID "SmartOfficeESP8266Node"

// Sensors
#define SENSOR_DHT22_TEMPERATURE "DHT22Temperature"
#define SENSOR_DHT22_HUMIDITY "DHT22Humidity"

//Methods
#define PUBLISH_INVENTORY DEVICE_ID  "/publish/DeviceProfile/Status/DeviceNodeInventory"
#define PUBLISH_PRESENCE DEVICE_ID  "/publish/DeviceProfile/Status/DeviceNodePresence"
#define PUBLISH_TEMPERATURE_DATA DEVICE_ID  "/publish/DeviceProfile/" NODE_ID "/" SENSOR_DHT22_TEMPERATURE
#define PUBLISH_HUMIDITY_DATA DEVICE_ID  "/publish/DeviceProfile/" NODE_ID "/" SENSOR_DHT22_HUMIDITY

float temp_c;  // Values read from sensor
float hum;

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
//WiFiClient client;
// or... use WiFiFlientSecure for SSL
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, ARB_SERVER, ARB_SERVERPORT, DEVICE_ID, ARB_USERNAME, ARB_PW);

/****************************** Feeds ***************************************/

Adafruit_MQTT_Publish inventory = Adafruit_MQTT_Publish(&mqtt, PUBLISH_INVENTORY, MQTT_QOS_0);
Adafruit_MQTT_Publish presence = Adafruit_MQTT_Publish(&mqtt, PUBLISH_PRESENCE, MQTT_QOS_0);
Adafruit_MQTT_Publish tempData = Adafruit_MQTT_Publish(&mqtt, PUBLISH_TEMPERATURE_DATA, MQTT_QOS_0);
Adafruit_MQTT_Publish humData = Adafruit_MQTT_Publish(&mqtt, PUBLISH_HUMIDITY_DATA, MQTT_QOS_0);


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
static const char ntpServerName[] = "0.tr.pool.ntp.org";
const int timeZone = 0;

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
void sendNTPpacket(IPAddress &address);
time_t prevDisplay = 0; // when the digital clock was displayed

void setup() {
  Serial.begin(115200);
  delay(10);

  // Initialize DHT
  dht.begin();

  Serial.println(F("IoT-Ignite MQTT Demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: ")); Serial.println(WiFi.localIP());


  client.setInsecure();

  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);

  // Get initial temperature
  temp_f = dht.readTemperature();
  Serial.println();
  Serial.print("Initial Temperature: ");
  Serial.println(temp_f);
  Serial.println();

  // Get initial humidity
  temp_f = dht.readHumidity();
  Serial.println();
  Serial.print("Initial Humidity: ");
  Serial.println(temp_f);
  Serial.println();


}

//Wait 20 seconds before sending data to IoT-Ignite
int delayTime = 30000;
int startDelay = 0;
boolean inventory_sent = false;
boolean node_presence_sent = false;
boolean temp_presence_sent = false;
boolean hum_presence_sent = false;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  if (millis() - startDelay < delayTime) {
    //waiting delaytime
  } else {
    startDelay = millis();
    //Get temperature in Celcius
    temp_f = dht.readTemperature();
    Serial.print(F("\nSending temperature: "));
    Serial.print(temp_c);
    Serial.print("...");
    if (!isnan(temp_c)) {
      //Publish to IoT-Ignite
      sendData(SENSOR_DHT22_TEMPERATURE, temp_c);
    }

    //Get humidty
    hum = dht.readHumidity();
    Serial.print(F("\nSending humidity: "));
    Serial.print(hum);
    Serial.print("...");
    if (!isnan(hum)) {
      //Publish to IoT-Ignite
      sendData(SENSOR_DHT22_HUMIDITY, hum);
    }

    if (!node_presence_sent) {
      sendNodePresence(1, "");
      node_presence_sent = true;
    }

    if (!temp_presence_sent && sendSensorPresence(SENSOR_DHT22_TEMPERATURE, 1, "")) {
      temp_presence_sent = true;
    }

    if (!hum_presence_sent && sendSensorPresence(SENSOR_DHT22_HUMIDITY, 1, "")) {
      hum_presence_sent = true;
    }

  }


  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
    if(! mqtt.ping()) {
    mqtt.disconnect();
    }
  */
}

//{"data":[{"nodeId":"NODE_ID","things":[{"id":"SENSOR_DHT22_TEMPERATURE","dataType":"FLOAT","vendor":"VENDOR","actuator":false,"type":"TYPE"}]}]}
bool sendInventory() {
  String packet = "";

  StaticJsonDocument<384> doc;

  JsonObject inventory_json = doc["data"].createNestedObject();
  inventory_json["nodeId"] = NODE_ID;

  JsonObject things_temp_json = inventory_json["things"].createNestedObject();
  things_temp_json["id"] = SENSOR_DHT22_TEMPERATURE;
  things_temp_json["dataType"] = "FLOAT";
  things_temp_json["vendor"] = "DHT";
  things_temp_json["actuator"] = false;
  things_temp_json["type"] = "Temp&Hum Sensor";

  JsonObject things_hum_json = inventory_json["things"].createNestedObject();
  things_hum_json["id"] = SENSOR_DHT22_HUMIDITY;
  things_hum_json["dataType"] = "FLOAT";
  things_hum_json["vendor"] = "DHT";
  things_hum_json["actuator"] = false;
  things_hum_json["type"] = "Temp&Hum Sensor";

  serializeJson(doc, packet);
  serializeJson(doc, Serial);

  return inventory.publish(packet.c_str());
}

//{"data":{"sensorData":[{"date":1533284119476,"values":["20000"]}]}}
bool sendData(String sensor, float value) {
  String packet = "";

  StaticJsonDocument<128> doc;

  JsonObject sensor_data_json = doc["data"]["sensorData"].createNestedObject();
  sensor_data_json["date"] = (String(now()) + "000");
  sensor_data_json["values"][0] = value;

  serializeJson(doc, packet);
  serializeJson(doc, Serial);

  if (sensor == SENSOR_DHT22_TEMPERATURE) {
    return tempData.publish(packet.c_str());
  } else {
    return humData.publish(packet.c_str());
  }
}

//{"data":[{"nodeId":"NODE_ID","description":"description","connected":true}]}
bool sendNodePresence(int connected, String message) {
  String packet = "";

  StaticJsonDocument<128> doc;

  JsonObject presence_json = doc["data"].createNestedObject();
  presence_json["nodeId"] = NODE_ID;
  presence_json["description"] = message;
  presence_json["connected"] = connected;

  serializeJson(doc, packet);
  serializeJson(doc, Serial);

  Serial.println("Presence :");
  Serial.println(packet.c_str());


  return presence.publish(packet.c_str());
}

//{"data":[{"nodeId":"NODE_ID","thingId":"THING_ID","description":"description","connected":true}]}
bool sendSensorPresence(String sensor, int connected, String message) {
  String packet = "";

  StaticJsonDocument<192> doc;

  JsonObject thing_presence_json = doc["data"].createNestedObject();
  thing_presence_json["nodeId"] = NODE_ID;
  thing_presence_json["thingId"] = sensor;
  thing_presence_json["description"] = message;
  thing_presence_json["connected"] = connected;

  serializeJson(doc, packet);
  serializeJson(doc, Serial);

  return presence.publish(packet.c_str());
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 5 seconds..."));
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  sendInventory();
  Serial.println(F("MQTT Connected!"));
}

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() {

  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address) {

  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
