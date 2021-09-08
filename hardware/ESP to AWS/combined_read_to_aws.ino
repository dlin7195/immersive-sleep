// include necessary libraries
// pulse_ox sensor
#include <heartRate.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
// accelerometer
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// neopixels
#include <Adafruit_NeoPixel.h>
// wifi + pubsub client
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
extern "C" {
#include "libb64/cdecode.h"
}
// for processing data into JSON
#include <ArduinoJson.h>
// for getting time
#include <NTPClient.h>
#include <WiFiUdp.h>

const char* ssid = "wifi ssid";
const char* password = "wifi password";

// find awsEndpoint in AWS Console: Manage - Things
// HTTPS Rest endpoint 
const char* awsEndpoint = "sample endpoint";

// certificates need to formatted as such: comment out the BEGIN and END 
// lines, add " character at the start of each line and a " and backslash
// at the end of each line

// xxxxxxxxxx-certificate.pem.crt
const String certificatePemCrt = \
//-----BEGIN CERTIFICATE-----
// "place certificate here"
//-----END CERTIFICATE-----


// xxxxxxxxxx-private.pem.key
const String privatePemKey = \
//-----BEGIN RSA PRIVATE KEY-----
// "place private key here"
//-----END RSA PRIVATE KEY-----

const String caPemCrt = \
//-----BEGIN CERTIFICATE-----
// "place CA certificate here"
//-----END CERTIFICATE-----

WiFiClientSecure wiFiClient;
void msgReceived(char* topic, byte* payload, unsigned int len);
PubSubClient pubSubClient(awsEndpoint, 8883, msgReceived, wiFiClient);

#define NEO_PIN 5
#define NUMPIXELS 12
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

#include <TimeLib.h>
int brightness = 0;
uint32_t IR_read;
uint32_t Red_read;
float accx;
float accy;
float accz;
float gyrox;
float gyroy;
float gyroz;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57))
  {
    Serial.println("Failed to find MAX30102");
    while (1) {
      delay(10);
    }
  }

  // setup for MAX30102 --> use default
  // not currently outputting anything, most likely needs external 5v source
  // EDIT: fixed: change pullup resistors to 5k (10k is too much)
  particleSensor.setup();
  
  // setup for MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  // setup for neopixels
  pixels.begin();
  pixels.setBrightness(30); //adjust brightness here
  pixels.show(); // Initialize all pixels to 'off'

  // setup wifi + aws connect
  Serial.println("ESP8266 AWS IoT Example");

  Serial.print("Connecting to "); Serial.print(ssid);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();
  Serial.print(", WiFi connected, IP address: "); Serial.println(WiFi.localIP());

  // start NTP timeclient
  timeClient.begin();

  // get current time, otherwise certificates are flagged as expired
  setCurrentTime();

  uint8_t binaryCert[certificatePemCrt.length() * 3 / 4];
  int len = b64decode(certificatePemCrt, binaryCert);
  wiFiClient.setCertificate(binaryCert, len);
  
  uint8_t binaryPrivate[privatePemKey.length() * 3 / 4];
  len = b64decode(privatePemKey, binaryPrivate);
  wiFiClient.setPrivateKey(binaryPrivate, len);

  uint8_t binaryCA[caPemCrt.length() * 3 / 4];
  len = b64decode(caPemCrt, binaryCA);
  wiFiClient.setCACert(binaryCA, len);

  Serial.println("");
  delay(100);
}

unsigned long lastPublish;
// int msgCount;

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accx = a.acceleration.x;
  accy = a.acceleration.y;
  accz = a.acceleration.z;
  gyrox = g.gyro.x;
  gyroy = g.gyro.y;
  gyroz = g.gyro.z;

  IR_read = particleSensor.getIR();
  Red_read = particleSensor.getRed();

  unsigned long epoch_time;
  timeClient.update();
  epoch_time = timeClient.getEpochTime();
  String str_epoch = String(epoch_time);

  // create JSON object
  StaticJsonDocument<192> doc;

  // format all readings into JSON
  doc["Accel_x"] = accx;
  doc["Accel_y"] = accy;
  doc["Accel_z"] = accz;
  doc["Gyro_x"] = gyrox;
  doc["Gyro_y"] = gyroy;
  doc["Gyro_z"] = gyroz;
  doc["IR"] = IR_read;
  doc["Red"] = Red_read;
  doc["Timestamp"] = str_epoch; // do seconds since epoch

  // serialize JSON in buffer
  char buff[256];
  serializeJson(doc, buff);

  pubSubCheckConnect();

  if (millis() - lastPublish > 5000) {
    // publish buffer contents periodically
    pubSubClient.publish("sensor_data", buff);
    Serial.print("Published: "); Serial.println(buff);
    lastPublish = millis();
  }
  
  delay(250);
}

void msgReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void pubSubCheckConnect() {
  if ( ! pubSubClient.connected()) {
    Serial.print("PubSubClient connecting to: "); Serial.print(awsEndpoint);
    while ( ! pubSubClient.connected()) {
      Serial.print(".");
      pubSubClient.connect("ESPthing");
    }
    Serial.println(" connected");
    pubSubClient.subscribe("inTopic");
  }
  pubSubClient.loop();
}

int b64decode(String b64Text, uint8_t* output) {
  base64_decodestate s;
  base64_init_decodestate(&s);
  int cnt = base64_decode_block(b64Text.c_str(), b64Text.length(), (char*)output, &s);
  return cnt;
}

void setCurrentTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: "); Serial.print(asctime(&timeinfo));
}
