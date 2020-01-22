#include <WiFi.h>
#include "MPU9250.h"

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>

float AccelX;
float AccelY;
float AccelZ;

char ssid[] = "BTHub6-P28P";          // your network SSID (name)
char pass[] = "rxJCgCfxh6xv";                    // your network password

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192,168,1,89);        // remote IP of your computer
const unsigned int outPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }


    // Connect to WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
#ifdef ESP32
    Serial.println(localPort);
#else
    Serial.println(Udp.localPort());
#endif
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  AccelX = IMU.getAccelX_mss();
  Serial.print("\t");

  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  AccelY = IMU.getAccelY_mss();


  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  AccelZ = IMU.getAccelZ_mss();

  // Serial.print(IMU.getGyroX_rads(),6);
  // Serial.print("\t");
  // Serial.print(IMU.getGyroY_rads(),6);
  // Serial.print("\t");
  // Serial.print(IMU.getGyroZ_rads(),6);
  // Serial.print("\t");
  // Serial.print(IMU.getMagX_uT(),6);
  // Serial.print("\t");
  // Serial.print(IMU.getMagY_uT(),6);
  // Serial.print("\t");
  // Serial.print(IMU.getMagZ_uT(),6);
  // Serial.print("\t");
  // Serial.println(IMU.getTemperature_C(),6);
  delay(100);



   OSCMessage msg("/Sensor1/AccelX");
    msg.add((float)AccelX);
    msg.add((float)AccelY);
    msg.add((float)AccelZ);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    delay(500);
}



