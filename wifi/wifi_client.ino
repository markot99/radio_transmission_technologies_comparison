// the source code is based on this example:
// https://randomnerdtutorials.com/esp32-https-requests/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// microcontroller specific settings
#ifdef CONFIG_IDF_TARGET_ESP32C6
#define BOOT_PIN 8
#define SENDING_DATA_PIN 14
#elif CONFIG_IDF_TARGET_ESP32S3
#include "LoRaWan_APP.h"

#define BOOT_PIN 47
#define SENDING_DATA_PIN 48
#endif

// Access point settings
const char* ssid = "airsensenet";
const char* password = "mypassword12";

const char* serverURL = "<SET_HTTPS_URL>";

// deep sleep duration
#define SLEEP_INTERVALL_SEC 5

// data to send
#define DATA_SIZE 100
uint8_t dataArray[DATA_SIZE] = { 0 };

// Root CA certificate for test.tech
const char* rootCACertificate =
  "-----BEGIN CERTIFICATE-----\n"
  "<CERTIFICATE_CONTENT>\n"
  "-----END CERTIFICATE-----\n";

//! @brief Put the device into deep sleep mode
void deepsleep() {
// Settings required for the WiFi LoRa 32(V3) to reduce power consumption in deep sleep
#ifdef CONFIG_IDF_TARGET_ESP32S3
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);

  Radio.Sleep();
  SPI.end();
  pinMode(RADIO_DIO_1, ANALOG);
  pinMode(RADIO_NSS, ANALOG);
  pinMode(RADIO_RESET, ANALOG);
  pinMode(RADIO_BUSY, ANALOG);
  pinMode(LORA_CLK, ANALOG);
  pinMode(LORA_MISO, ANALOG);
  pinMode(LORA_MOSI, ANALOG);
#endif
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVALL_SEC * 1000000);
  esp_deep_sleep_start();
}

//! @brief Setup the wifi client
void setup() {
  // set boot pin to indicate that the boot process finished
  pinMode(BOOT_PIN, OUTPUT);
  digitalWrite(BOOT_PIN, HIGH);

  // configure sending data pin
  pinMode(SENDING_DATA_PIN, OUTPUT);

  // connect to WiFi
  WiFi.begin(ssid, password);

  // wait until connected
  while (WiFi.status() != WL_CONNECTED) {}
}

void loop() {
  // setup wifi client
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  HTTPClient https;
  
  // send data
  if (https.begin(client, serverURL)) {
    https.addHeader("Content-Type", "application/octet-stream");

    digitalWrite(SENDING_DATA_PIN, HIGH);
    https.POST(dataArray, DATA_SIZE);
    digitalWrite(SENDING_DATA_PIN, LOW);

    https.end();
  }

  // disconnect from WiFi and go to deep sleep
  WiFi.disconnect();
  deepsleep();
}