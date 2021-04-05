/*
* The MIT License (MIT)
*
* Copyright (C) 2018 Gabriel Nikol
*/
#include <I2Cdev.h>
#include <MPU9250_9Axis_MotionApps41.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiUdpSocket.h>
#include <MqttSnClient.h>
#include <ArduinoOTA.h>

const char* ssid     = "habr";
const char* password = "hello_habrahabr!";

MPU9250 mpu;

#define SDA 4
#define SCL 5


#define buffer_length 24
char buffer[buffer_length];
uint16_t buffer_pos = 0;

IPAddress ip(10, 10, 10, 30);
IPAddress gateway(10, 10, 10, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress gatewayIPAddress(10, 10, 10, 100);
uint16_t localUdpPort = 10000;

// #define gatewayHostAddress "arsmb.de"

WiFiUDP udp;
WiFiUdpSocket wiFiUdpSocket(udp, localUdpPort);
MqttSnClient<WiFiUdpSocket> mqttSnClient(wiFiUdpSocket);

//const char* topic = "adam/thigh_l"
const char* clientId = "thigh_l";
char* subscribeTopicName = "main";
char* publishTopicName = "adam/thigh_l";
String messageMQTT;

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[48]; // FIFO storage buffer
bool blinkState = false;
bool sendQuat = false;

Quaternion q;           // [w, x, y, z]         quaternion container
Quaternion q_in;
Quaternion q_out;

int8_t qos = 0;

void mqttsn_callback(char *topic, uint8_t *payload, uint16_t length, bool retain) {
  for (uint16_t i = 0; i < length; i++) {
    messageMQTT += (char)payload[i];
  }
  if (messageMQTT == "1"){
    sendQuat = true;
    messageMQTT = "";
  }
  else if (messageMQTT == "stop") {
    sendQuat = false;
    messageMQTT = "";
  }
}

void setup_wifi() {
  delay(10);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
  }
}

void convertIPAddressAndPortToDeviceAddress(IPAddress& source, uint16_t port, device_address& target) {
  // IPAdress 0 - 3 bytes
  target.bytes[0] = source[0];
  target.bytes[1] = source[1];
  target.bytes[2] = source[2];
  target.bytes[3] = source[3];
  // Port 4 - 5 bytes
  target.bytes[4] = port >> 8;
  target.bytes[5] = (uint8_t) port ;
}

void setup() {
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);
  Serial.begin(115200);
  setup_wifi();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();
  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
  });
  ArduinoOTA.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  mqttSnClient.begin();
  device_address gateway_device_address;
  convertIPAddressAndPortToDeviceAddress(gatewayIPAddress, localUdpPort, gateway_device_address);
  mqttSnClient.connect(&gateway_device_address, clientId, 180);
  mqttSnClient.setCallback(mqttsn_callback);
  mqttSnClient.subscribe(subscribeTopicName, qos);
}


void loop() {
  fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
      mpu.resetFIFO();
      }
    else if (fifoCount % packetSize != 0) {
      mpu.resetFIFO();
      }
    else if (fifoCount >= packetSize  && sendQuat) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mqttSnClient.publish((uint8_t*)&q, publishTopicName, qos);
        //Serial.println("Send");
        blinkState = !blinkState;
        digitalWrite(LED_BUILTIN, blinkState);
        mpu.resetFIFO();
      }
  ArduinoOTA.handle();
  mqttSnClient.loop();
}