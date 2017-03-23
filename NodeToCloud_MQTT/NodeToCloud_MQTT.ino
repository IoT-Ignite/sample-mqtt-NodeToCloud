/***************************************************
  Adafruit MQTT Library Arbitrary Data Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Stuart Feichtinger
  Modifed from the mqtt_esp8266 example written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Time.h>
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>
#include <DHT.h>
#include <WiFiUdp.h>

#define DEBUG_PRINTER Serial

#define DHTPIN D7     // what digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
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

#define WLAN_SSID       "..."
#define WLAN_PASS       "..."

/************************* Adafruit.io Setup *********************************/

#define ARB_SERVER      "mqtt.ardich.com"
#define ARB_SERVERPORT  8883                   // use 8883 for SSL
#define ARB_USERNAME    "..."
#define ARB_PW          "..."

/************************* Device, Node and Sensors **************************/
//Device
#define DEVICE_ID "..."

// Node
#define NODE_ID "Virtual Node"

// Sensors
#define SENSOR_DHT11_TEMPERATURE "DHT11 Temp"
#define SENSOR_DHT11_HUMIDITY "DHT11 Hum"

//Methods
#define PUBLISH_PRESENCE DEVICE_ID  "/publish/DeviceProfile/Status/DeviceNodePresence"
#define PUBLISH_TEMPERATURE_DATA DEVICE_ID  "/publish/DeviceProfile/" NODE_ID "/" SENSOR_DHT11_TEMPERATURE
#define PUBLISH_HUMIDITY_DATA DEVICE_ID  "/publish/DeviceProfile/" NODE_ID "/" SENSOR_DHT11_HUMIDITY

float temp_f;  // Values read from sensor

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
//WiFiClient client;
// or... use WiFiFlientSecure for SSL
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, ARB_SERVER, ARB_SERVERPORT, DEVICE_ID, ARB_USERNAME, ARB_PW);

/****************************** Feeds ***************************************/

Adafruit_MQTT_Publish presence = Adafruit_MQTT_Publish(&mqtt, PUBLISH_PRESENCE, MQTT_QOS_1);
Adafruit_MQTT_Publish tempData = Adafruit_MQTT_Publish(&mqtt, PUBLISH_TEMPERATURE_DATA, MQTT_QOS_1);
Adafruit_MQTT_Publish humData = Adafruit_MQTT_Publish(&mqtt, PUBLISH_HUMIDITY_DATA, MQTT_QOS_1);


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
int delayTime = 5000;
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
    Serial.print(temp_f);
    Serial.print("...");
    if(!isnan(temp_f)) {
      //Publish to IoT-Ignite
      if (! sendData(SENSOR_DHT11_TEMPERATURE, temp_f)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
      }
    }

    //Get humidty
    temp_f = dht.readHumidity();
    Serial.print(F("\nSending humidity: "));
    Serial.print(temp_f);
    Serial.print("...");
    if(!isnan(temp_f)) {
      //Publish to IoT-Ignite
      if (! sendData(SENSOR_DHT11_HUMIDITY, temp_f)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
      }
    }

    if(!node_presence_sent) {
      sendNodePresence(1, "");
      node_presence_sent = true;
    }

    if(!temp_presence_sent && sendSensorPresence(SENSOR_DHT11_TEMPERATURE, 1, "")) {
        temp_presence_sent = true;
    }

    if(!hum_presence_sent && sendSensorPresence(SENSOR_DHT11_HUMIDITY, 1, "")) {
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

boolean sendData(String sensor, float value) {
  String packet = "";

  //StaticJsonBuffer<400> jsonBuffer;
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  JsonObject& data = root.createNestedObject("data");
  JsonArray& sensorDataArray = data.createNestedArray("sensorData");

  JsonObject& sensorData = jsonBuffer.createObject();

  sensorDataArray.add(sensorData);
  sensorData["date"] = (String(now()) + "000");

  Serial.println(String(now()) + "000");
  JsonArray& values = jsonBuffer.createArray();

  sensorData["values"] = values;

  values.add(value);

  root.printTo(packet);

  Serial.println(sensor);
  Serial.println(packet.c_str());

  if(sensor == SENSOR_DHT11_TEMPERATURE) {
    return tempData.publish(packet.c_str());
  } else {
    return humData.publish(packet.c_str());
  }
}

void sendNodePresence(int connected, String message) {
  String packet = "";

  //StaticJsonBuffer<400> jsonBuffer;
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  JsonArray& data = root.createNestedArray("data");
  JsonObject& nodeConnection = jsonBuffer.createObject();

  data.add(nodeConnection);

  nodeConnection["nodeId"] = NODE_ID;
  nodeConnection["description"] = message;
  nodeConnection["connected"] = connected;

  root.printTo(packet);

  Serial.println("Presence :");
  Serial.println(packet.c_str());

  if (! presence.publish(packet.c_str())) {
    Serial.println(F("Node Presence Sent Failed!"));
  } else {
    Serial.println(F("Node Presence Sent."));
  }
}

boolean sendSensorPresence(String sensor, int connected, String message) {
  String packet = "";

  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  JsonArray& data = root.createNestedArray("data");

  JsonObject& tempSensorConnection = jsonBuffer.createObject();
  data.add(tempSensorConnection);

  tempSensorConnection["nodeId"] = NODE_ID;
  tempSensorConnection["thingId"] = sensor;
  tempSensorConnection["description"] = message;
  tempSensorConnection["connected"] = connected;

  root.printTo(packet);

  Serial.println("Presence :");
  Serial.println(packet.c_str());

  Serial.println(sensor);
  if (! presence.publish(packet.c_str())) {
    Serial.println(F("Sensor Presence Sent Failed!"));
    return false;
  } else {
    Serial.println(F("Sensor Presence Sent."));
    return true;
  }
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
  Serial.println(F("MQTT Connected!"));
}

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
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
void sendNTPpacket(IPAddress &address)
{
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
