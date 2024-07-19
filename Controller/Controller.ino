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

const char MQTT_TOPIC_CHOPPER_GET[] = "$aws/things/Chopper/shadow/name/ChopperShadow/update/accepted";
const char MQTT_TOPIC_CHOPPER_UPDATE[] = "$aws/things/Chopper/shadow/name/ChopperShadow/update";
const char MQTT_TOPIC_CONTROLLER_UPDATE[] = "$aws/things/Controller/shadow/name/ControllerShadow/update";

#define HEARTBEAT_DELAY 5000   //ms delay between heatbeat messages

unsigned int hitcount = 0;
unsigned long lastHeartbeat = 0;
int buttonPrevState = LOW;

void sendHeartbeat(void)
{
  JsonDocument doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  reported["connected"] = "true";

  char shadow[measureJson(doc) + 1];
  serializeJson(doc, shadow, sizeof(shadow));
  publishToTopic(MQTT_TOPIC_CONTROLLER_UPDATE, shadow);
}

void resetHitcount()
{
  JsonDocument doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  reported["hitcount"] = "0";

  char shadow[measureJson(doc) + 1];
  serializeJson(doc, shadow, sizeof(shadow));
  publishToTopic(MQTT_TOPIC_CHOPPER_UPDATE, shadow);
}

/* pin to led mapping */
uint8_t green1 = D4_PIN;
uint8_t green2 = D3_PIN;
uint8_t green3 = D2_PIN; 
uint8_t yellow = D1_PIN;
uint8_t red    = D5_PIN;
uint8_t button = D6_PIN;
uint8_t blue   = D7_PIN;

void updateHealthLEDs()
{
  if (hitcount >= 4) {
   digitalWrite(red, HIGH);
   digitalWrite(yellow, LOW);
   digitalWrite(green1, LOW);
   digitalWrite(green2, LOW);
   digitalWrite(green3, LOW); 
  }
  else if (hitcount == 3) {
   digitalWrite(red, LOW);
   digitalWrite(yellow, HIGH);
   digitalWrite(green1, LOW);
   digitalWrite(green2, LOW);
   digitalWrite(green3, LOW); 
  }
  else if (hitcount == 2) {
   digitalWrite(red, LOW);
   digitalWrite(yellow, LOW);
   digitalWrite(green1, LOW);
   digitalWrite(green2, LOW);
   digitalWrite(green3, HIGH); 
  }
  else if (hitcount == 1) {
  digitalWrite(red, LOW);
   digitalWrite(yellow, LOW);
   digitalWrite(green1, LOW);
   digitalWrite(green2, HIGH);
   digitalWrite(green3, HIGH); 
  }
  else {  //hitcount == 0
   digitalWrite(red, LOW);
   digitalWrite(yellow, LOW);
   digitalWrite(green1, HIGH);
   digitalWrite(green2, HIGH);
   digitalWrite(green3, HIGH); 
  }
}

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
    int hits = reported["hitcount"];
    Serial.print("hits: ");
    Serial.println(hits);
    hitcount = hits;
  }

}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println();
  
  pinMode(green1, OUTPUT);
  pinMode(green2, OUTPUT);
  pinMode(green3, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(red,    OUTPUT);
  pinMode(blue,   OUTPUT);
  pinMode(button, INPUT);

  updateHealthLEDs();
  setWifiLED(false);
  
  WiFi.hostname(THINGNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));

  NTPConnect();

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
    //checkWiFiThenMQTTNonBlocking();
    //checkWiFiThenReboot();
    subscribeToTopic(MQTT_TOPIC_CHOPPER_GET);
  }
  else
  {
    //Serial.println("Already connected");
    setWifiLED(true);
    client.loop();
    if (millis() - lastHeartbeat > HEARTBEAT_DELAY)
    {
      //Serial.println("Heartbeat");
      lastHeartbeat = millis();
      sendHeartbeat();
    }

    updateHealthLEDs();

    int buttonState = digitalRead(button);
    if (buttonState == HIGH && buttonPrevState == LOW) {
      Serial.println("Reset hitcount");
      resetHitcount();
    }
    buttonPrevState = buttonState;
    
  }
}

