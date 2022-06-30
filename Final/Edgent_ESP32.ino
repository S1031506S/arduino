
// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLcdEaWRjg"
#define BLYNK_DEVICE_NAME "New Template"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_WROVER_BOARD
//#define USE_TTGO_T7
//#define USE_ESP32C3_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "BlynkEdgent.h"
#include "DHT.h"
#define DHTPIN 23
#define relayPin1 22
#define relayPin2 21
#define light_sensor 39
#define WLAN_SSID "Nano"
#define WLAN_PASS "ssssssss"

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"

// Using port 8883 for MQTTS
#define AIO_SERVERPORT  8883

// Adafruit IO Account Configuration
// (to obtain these values, visit https://io.adafruit.com and click on Active Key)
#define AIO_USERNAME "suka82502511"
#define AIO_KEY      "aio_axWR26eWdiQRDCjMXSb2eWqJstAN"

/************ Global State (you don't need to change this!) ******************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com root CA
const char* adafruitio_root_ca = \
                                 "-----BEGIN CERTIFICATE-----\n" \
                                 "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
                                 "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
                                 "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
                                 "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
                                 "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
                                 "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
                                 "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
                                 "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
                                 "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
                                 "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
                                 "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
                                 "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
                                 "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
                                 "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
                                 "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
                                 "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
                                 "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
                                 "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
                                 "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
                                 "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
                                 "-----END CERTIFICATE-----\n";

/****************************** Feeds ***************************************/
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish soil = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil");
Adafruit_MQTT_Publish light = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/light");

DHT dht(DHTPIN, DHT11, 15);
void Dht11()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) { // 如果沒有h or t讀到數值
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // Now we can publish stuff
  Serial.print(F("\nSending Humidity = "));
  Serial.print(h);
  Serial.print("...");
  if (! humid.publish(h)) {
    // publish value h to adafruit io as name humid
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  Serial.print(F("Sending Temperature = "));
  Serial.print(t);
  Serial.print("...");
  if (! temp.publish(t)) {
    // publish value t to adafruit io as name temp
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  Blynk.virtualWrite(V5, h);  // V5 is for Humidity
  Blynk.virtualWrite(V6, t);  // V6 is for Temperature
  Serial.print("Temp: "); Serial.print(t); Serial.print("°C");
  Serial.print("\t");
  Serial.print("Humidity: "); Serial.print(h); Serial.println("\%");

}
double val;
void Light_sensor(){
  val = (analogRead(light_sensor) * 100 ) / 4095;
  // max is 4095, so quantizing to 0 ~ 100
  if (isnan(val)) { // 如果沒有讀到val數值
    Serial.println("Failed to read from Light_sensor!");
    return;
  }
  Serial.print("light intensity: "); Serial.print(val); Serial.print("\%");
  Serial.print(F("\nSending val "));
  Serial.print(val);
  Serial.print(F(" to light..."));
  if (! light.publish(val)) {
    // publish value val to adafruit io as name light
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  Serial.print("\t");
  Blynk.virtualWrite(V7, val);  // V7 is for Humidity
  Serial.print("\t");
}

void Soil_sensor(){
  double ini_soil_per = analogRead(34); // 類比輸入ADC
  if (ini_soil_per < 1000) { // 修正sensor數值範圍為1000~2650
    ini_soil_per = 1000;
  }
  if (ini_soil_per > 2650) {
    ini_soil_per = 2650;
  }
  double soil_per = 100 - (ini_soil_per - 1000) * 100 / (2650 - 1000);
  Serial.print("Soil moisture:"); Serial.print(soil_per); Serial.print("\%");
  Serial.print(F("\nSending val "));
  Serial.print(soil_per);
  Serial.print(F(" to soil..."));
  if (! soil.publish(soil_per)) {
    // publish value soil_per to adafruit io as name soil
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  if (isnan(soil_per)) { // 如果沒有讀到soil_per數值
    Serial.println("Failed to read from Soil_sensor!");
    return;
  }
  Blynk.virtualWrite(V8, soil_per);  // V8 is for Humidity

  Serial.println();
  Serial.println();
  Serial.println();
}

void setup()
{
  pinMode(DHTPIN, OUTPUT);
  pinMode(relayPin1, OUTPUT); // For bulb
  pinMode(relayPin2, OUTPUT); // For pump
  pinMode(light_sensor, INPUT);
  Serial.begin(115200);
  dht.begin();

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) { // 顯示正在連接中...
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);
  delay(100);

  BlynkEdgent.begin();
}

BLYNK_WRITE(V1) { // control by button V1
// Blynk.Cloud is telling Esp32 “I'm WRITING something to Virtual Pin V1”

  int buttonState1 = param.asInt();
  // get the value parameter from the virtual pin and store it
  // in the local integer variable called virtual_pin_value
  if ((buttonState1 == 1) || (val < 40)) {
    // when press button send 1 to bs1, and if light intensity low also
    // turn on the light
    digitalWrite(relayPin1, LOW); //低電位觸發
    Serial.println("Turn on the light");
  }
  if (buttonState1 == 0) {
    digitalWrite(relayPin1, HIGH); //低電位觸發
    Serial.println("Turn off the light");
  }
}

BLYNK_WRITE(V2) { // control by button V2

  int buttonState2 = param.asInt();
  if (buttonState2 == 1) { // when press button send 1 to bs2
    digitalWrite(relayPin2, HIGH); //高電位觸發
    Serial.println("Watering");
  }
  if (buttonState2 == 0) { // when press button send 0 to bs2
    digitalWrite(relayPin2, LOW);
    Serial.println("Stop watering");
  }
}

void loop() {
  MQTT_connect();
  BlynkEdgent.run();
  Dht11();
  Light_sensor();
  Soil_sensor();

  delay(10000);
  // wait a couple seconds to avoid rate limit
}


void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3; // retry for 3 times, otherwise connect failed
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
}
