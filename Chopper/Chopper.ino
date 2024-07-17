#ifdef ESP32
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#error Platform not supported
#endif
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson (use v6.xx)
#include <time.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>
#define emptyString String()

#include "secrets.h"

const int MQTT_PORT = 8883;
//const char MQTT_SUB_TOPIC[] = "$aws/things/" THINGNAME "/shadow/update";
//const char MQTT_PUB_TOPIC[] = "$aws/things/" THINGNAME "/shadow/update";

#ifdef USE_SUMMER_TIME_DST
uint8_t DST = 1;
#else
uint8_t DST = 0;
#endif

#define HEARTBEAT_DELAY 1000   //ms delay between heatbeat messages


WiFiClientSecure net;

#ifdef ESP8266
BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);
#endif

PubSubClient client(net);

unsigned int hitCount = 0;
unsigned long lastHeartbeat = 0;
time_t now;
time_t nowish = 1510592825;

void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void setupIR(void)
{
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
}


void messageReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void pubSubErr(int8_t MQTTErr)
{
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    Serial.print("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    Serial.print("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    Serial.print("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    Serial.print("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    Serial.print("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    Serial.print("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    Serial.print("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    Serial.print("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    Serial.print("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    Serial.print("Connect unauthorized");
}

void connectToMqtt(bool nonBlocking = false)
{
  Serial.print("MQTT connecting ");
  while (!client.connected())
  {
    if (client.connect(THINGNAME))
    {
      Serial.println("connected!");
      //if (!client.subscribe(MQTT_SUB_TOPIC))
      //  pubSubErr(client.state());
    }
    else
    {
      Serial.print("failed, reason -> ");
      pubSubErr(client.state());
      if (!nonBlocking)
      {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else
      {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
}

void connectToWiFi(String init_str)
{
  if (init_str != emptyString)
    Serial.println(init_str);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  if (init_str != emptyString)
    Serial.println("ok!");
}

void checkWiFiThenMQTT(void)
{
  connectToWiFi("Checking WiFi");
  connectToMqtt();
}

unsigned long previousMillis = 0;
const long interval = 5000;

void checkWiFiThenMQTTNonBlocking(void)
{
  connectToWiFi(emptyString);
  if (millis() - previousMillis >= interval && !client.connected()) {
    previousMillis = millis();
    connectToMqtt(true);
  }
}

void checkWiFiThenReboot(void)
{
  connectToWiFi("Checking WiFi");
  Serial.print("Rebooting");
  ESP.restart();
}


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
  if (!client.publish("$aws/things/Chopper/shadow/name/ChopperShadow/update", shadow, false))
    pubSubErr(client.state());
}

void sendHitCount(void)
{
  JsonDocument doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  reported["hitcount"] = hitCount;

  //Serial.print("Heartbeat msg = ");
  //serializeJson(doc, Serial);

  char shadow[measureJson(doc) + 1];
  serializeJson(doc, shadow, sizeof(shadow));
  if (!client.publish("$aws/things/Chopper/shadow/name/ChopperShadow/update", shadow, false))
    pubSubErr(client.state());
}

void setup()
{
  Serial.begin(115200);
  delay(5000);
  Serial.println();
  Serial.println();
#ifdef ESP32
  WiFi.setHostname(THINGNAME);
#else
  WiFi.hostname(THINGNAME);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));

  NTPConnect();

  setupIR();

#ifdef ESP32
  net.setCACert(cacert);
  net.setCertificate(client_cert);
  net.setPrivateKey(privkey);
#else
  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
#endif

  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(messageReceived);
  client.setKeepAlive(5);

  connectToMqtt();
}

void loop()
{
  now = time(nullptr);
  if (!client.connected())
  {
    Serial.println("Reconnecting...");
    checkWiFiThenMQTT();
    //checkWiFiThenMQTTNonBlocking();
    //checkWiFiThenReboot();
  }
  else
  {
    //Serial.println("Already connected");
    client.loop();
    if (millis() - lastHeartbeat > HEARTBEAT_DELAY)
    {
      Serial.println("Heartbeat");
      lastHeartbeat = millis();
      sendHeartbeat();
    }

    if (IrReceiver.decode()) {
        IrReceiver.printIRResultShort(&Serial);
        if (IrReceiver.decodedIRData.protocol == NEC && IrReceiver.decodedIRData.command == 0x0) {
            //IrReceiver.printIRResultShort(&Serial);
            Serial.print("Hit! ");
            hitCount++;
            Serial.println(hitCount);
            sendHitCount();
            //IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        }
        IrReceiver.resume();

        //Serial.println();
    }
  }
}