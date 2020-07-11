#ifdef ARDUINO

#include <Arduino.h>
#include "LacrosseReceiver.h"

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/
#define WLAN_SSID       "diagonalley"
#define WLAN_PASS       ""

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "fpp1.rbhome"
#define AIO_SERVERPORT  1883 //8883
#define AIO_CLIENT    "lacrosse_bridge"
#define AIO_USERNAME  "drasch"
#define AIO_KEY       ""

//WiFiClientSecure client;
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
//, AIO_USERNAME, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, "time/seconds");

// static const char *fingerprint PROGMEM = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

void MQTT_connect();

// set timezone offset from UTC
int timeZone = -4; // UTC - 4 eastern daylight time (nyc)
int interval = 4; // trigger every X hours

int last_min = -1;

void timecallback(uint32_t current) {
  // adjust to local time zone
  current += (timeZone * 60 * 60);
  int curr_hour = (current / 60 / 60) % 24;
  int curr_min  = (current / 60 ) % 60;
  int curr_sec  = (current) % 60;

  Serial.print("Time: "); 
  Serial.print(curr_hour); Serial.print(':'); 
  Serial.print(curr_min); Serial.print(':'); 
  Serial.println(curr_sec);
  
  // only trigger on minute change
  if(curr_min != last_min) {
    last_min = curr_min;
    
    Serial.println("This will print out every minute!");
  }
}



LacrosseReceiver receiver(D2); // RF receiver connected to pin 5
uint32_t msec;

const char * TEMP_TOPIC = "diy/lacrosse/%d/temperature";
const char * HUMIDITY_TOPIC = "diy/lacrosse/%d/humidity";

void setup() {

    Serial.begin(115200);
    Serial.print(F("\nAdafruit IO anonymous Time Demo"));
    Serial.print("pin: ");
    Serial.println(D2);

    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
    }
    Serial.println(F(" WiFi connected."));
    Serial.println("IP address: "); Serial.println(WiFi.localIP());

    timefeed.setCallback(timecallback);
    mqtt.subscribe(&timefeed);


    receiver.enableReceive();
    msec = millis();
    Serial.println("Online");
    //pinMode(LED_BUILTIN, OUTPUT);

     // check the fingerprint of io.adafruit.com's SSL cert
    //client.setFingerprint(fingerprint);

}

void loop() {
    if (millis() - msec > 1000) {
        msec = millis();
        measure m = receiver.getNextMeasure();
        while (m.type != UNKNOWN) {
            Serial.print("Sensor #");
            Serial.print(m.sensorAddr);
            Serial.print(": ");
            if (m.sign < 0) Serial.print("-");
            Serial.print(m.units);
            Serial.print(".");
            Serial.print(m.decimals);
            Serial.println((m.type == TEMPERATURE)? " °C" : " %rh");

            const char * topic = (m.type == TEMPERATURE)? TEMP_TOPIC : HUMIDITY_TOPIC;
            const char * sign = (m.sign < 0) ? "-":"";
            char topic_buffer[40];
            char value_buffer[10];
            sprintf(topic_buffer, topic, m.sensorAddr);
            sprintf(value_buffer, "%s%d.%d", sign, m.units, m.decimals);
            mqtt.publish(topic_buffer, value_buffer);

            m = receiver.getNextMeasure();
        }
    }
    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();

    // wait 10 seconds for subscription messages
    // since we have no other tasks in this example.
    mqtt.processPackets(100);

    // keep the connection alive
    mqtt.ping();

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }

  Serial.println("MQTT Connected!");
}
#else
// This is needed to enable build in native environment
#include <iostream>

int main(int argc, char *argv[]) {
    std::cout << "This environment is for testing purposes only!";
    return 0;
}
#endif // #ifdef ARDUINO
