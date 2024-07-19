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

unsigned int hitCount = 0;
unsigned long lastHeartbeat = 0;

void sendHeartbeat(void)
{
  JsonDocument doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  reported["connected"] = "true";

  //Serial.print("Heartbeat msg = ");
  //serializeJson(doc, Serial);

  char shadow[measureJson(doc) + 1];
  serializeJson(doc, shadow, sizeof(shadow));
  if (!client.publish(MQTT_TOPIC_CONTROLLER_UPDATE, shadow, false))
    pubSubErr(client.state());
}

/* pin to led mapping */
uint8_t green1 = D4_PIN;
uint8_t green2 = D3_PIN;
uint8_t green3 = D2_PIN; 
uint8_t yellow = D1_PIN;
uint8_t red    = D5_PIN;
uint8_t button = D6_PIN;
uint8_t blue   = D7_PIN;

void resetHealthLEDs()
{
  digitalWrite(green1, HIGH);
  digitalWrite(green2, HIGH);
  digitalWrite(green3, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(red, LOW);
}

void setWifiLED(bool on)
{
  digitalWrite(blue, on ? HIGH : LOW);
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

  resetHealthLEDs();
  setWifiLED(false);
  
  WiFi.hostname(THINGNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));

  NTPConnect();

  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);
  //client.setKeepAlive(5);

  connectToMqtt();
  subscribeToTopic(MQTT_TOPIC_CHOPPER_GET);
  //subscribeToTopic("chopper/will");

  
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
    //subscribeToTopic("chopper/will");
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

    /*
    int buttonState = digitalRead(button);
    if (buttonState == HIGH) {
      //Serial.println("BUTTON == HIGH");
      digitalWrite(red, HIGH);
      digitalWrite(yellow, LOW);
    }
    else {
      digitalWrite(red, LOW);
      digitalWrite(yellow, HIGH);
    }
  */
  }
}

