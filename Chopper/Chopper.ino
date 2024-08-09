#define D1_PIN 5
#define D2_PIN 4
#define D3_PIN 0
#define D4_PIN 2
#define D5_PIN 14
#define D6_PIN 12
#define D7_PIN 13

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>
#define emptyString String()

#include "secrets.h"
#include "net.h"

#define HEARTBEAT_DELAY 5000   //ms delay between heatbeat messages
#define SIGNALS_PER_HIT 10     //effecively, how long do we need to receive IR signal to count as one "hit"

const char MQTT_TOPIC_CHOPPER_GET[] = "$aws/things/Chopper/shadow/name/ChopperShadow/update/accepted";
const char MQTT_TOPIC_CHOPPER_UPDATE[] = "$aws/things/Chopper/shadow/name/ChopperShadow/update";
const char MQTT_TOPIC_CONTROLLER_UPDATE[] = "$aws/things/Controller/shadow/name/ControllerShadow/update";

unsigned int hitCount = 0;
unsigned int signalCount = 0;
unsigned long lastHeartbeat = 0;

void setupIR(void)
{
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
}


void sendHeartbeat(void)
{
  JsonDocument doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  reported["connected"] = "true";

  char shadow[measureJson(doc) + 1];
  serializeJson(doc, shadow, sizeof(shadow));
  publishToTopic(MQTT_TOPIC_CHOPPER_UPDATE, shadow);
}

void sendHitCount(void)
{
  JsonDocument doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  reported["hitcount"] = hitCount;

  char shadow[measureJson(doc) + 1];
  serializeJson(doc, shadow, sizeof(shadow));
  publishToTopic(MQTT_TOPIC_CHOPPER_UPDATE, shadow);
}

uint8_t blue   = D7_PIN;

void setWifiLED(bool on)
{
  digitalWrite(blue, on ? HIGH : LOW);
}

void messageReceivedLocal(char *topic, byte *payload, unsigned int length)
{
  
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, MQTT_TOPIC_CHOPPER_GET) == 0) {
    JsonDocument doc;
    deserializeJson(doc, (char *) payload);
    JsonObject state = doc["state"];
    JsonObject reported = state["reported"];
    if (reported.containsKey("hitcount")) {
      int hits = reported["hitcount"];
      if (hitCount != hits) {
        Serial.print("Resetting hits to: ");
        Serial.println(hits);
        hitCount = hits;
      }
    }
  }

}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println();

  pinMode(blue,   OUTPUT);
  setWifiLED(false);

  WiFi.hostname(THINGNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));

  NTPConnect();

  setupIR();

  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceivedLocal);

  connectToMqtt();
  subscribeToTopic(MQTT_TOPIC_CHOPPER_GET);
}

void loop()
{
  now = time(nullptr);
  if (!client.connected())
  {
    setWifiLED(false);
    Serial.println("Reconnecting...");
    checkWiFiThenMQTT();
    subscribeToTopic(MQTT_TOPIC_CHOPPER_GET);
    //checkWiFiThenMQTTNonBlocking();
    //checkWiFiThenReboot();
  }
  else
  {
    //Serial.println("Already connected");
    setWifiLED(true);
    client.loop();
    if (millis() - lastHeartbeat > HEARTBEAT_DELAY)
    {
      Serial.println("Heartbeat");
      lastHeartbeat = millis();
      sendHeartbeat();
    }

    if (IrReceiver.decode()) {
        IrReceiver.printIRResult9Short(&Serial);
        if (IrReceiver.decodedIRData.protocol == NEC && IrReceiver.decodedIRData.command == 0x0) {
            signalCount++;
            if (signalCount >= SIGNALS_PER_HIT) {
              hitCount++;
              signalCount = 0;
              Serial.print("Hit! ");
              Serial.println(hitCount);
              sendHitCount();
            }
        }
        IrReceiver.resume();
    }
  }
}