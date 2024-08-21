// the source code is based on this example:
// https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/security/BLE_server/BLE_server_passkey/BLE_server_passkey.ino

#include "BLEDevice.h"

// microcontroller specific settings
#ifdef CONFIG_IDF_TARGET_ESP32C6
#define BOOT_PIN 8
#define SENDING_DATA_PIN 14
#elif CONFIG_IDF_TARGET_ESP32S3
#include "LoRaWan_APP.h"

#define BOOT_PIN 47
#define SENDING_DATA_PIN 48

static RadioEvents_t RadioEvents;
#endif

// service and characteristic UUIDs
static BLEUUID serviceUUID("16e9227a-50d8-487f-8e6a-cc58e0e3e3d6");
static BLEUUID characteristicUUID("aba7ff45-9d23-46db-810a-d1c574abda95");

// BLE client variables
static BLEAddress* pServerAddress;
static boolean doConnect = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

// deep sleep duration
#define SLEEP_INTERVALL_SEC 5


// data to send
const size_t dataSize = 1;
uint8_t dataArray[dataSize] = { 0 };

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

//! @brief BLE security callbacks, used for encryption
class MySecurity : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() {
    return 784564;
  }

  void onPassKeyNotify(uint32_t pass_key) {}

  bool onConfirmPIN(uint32_t pass_key) {
    vTaskDelay(5000);
    return true;
  }

  bool onSecurityRequest() {
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) {}
};

//! @brief BLE advertised device callbacks
//! @note used to find the server
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == "AirSense") {
      advertisedDevice.getScan()->stop();

      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    }
  }
};

//! @brief Setup the BLE client
void setup() {
  // set boot pin to indicate that the boot process finished
  pinMode(BOOT_PIN, OUTPUT);
  digitalWrite(BOOT_PIN, HIGH);

  // configure sending data pin
  pinMode(SENDING_DATA_PIN, OUTPUT);

  // initialize BLE and start scanning
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

//! @brief Main loop
void loop() {
  // wait for the server to be found
  if (doConnect == true) {
    // connect to the server
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new MySecurity());
    
    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
    pSecurity->setCapability(ESP_IO_CAP_OUT);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    BLEClient* pClient = BLEDevice::createClient();

    pClient->connect(*pServerAddress);

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      return;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(characteristicUUID);
    if (pRemoteCharacteristic == nullptr) {
      return;
    }

    // send data to the server
    digitalWrite(SENDING_DATA_PIN, HIGH);
    pRemoteCharacteristic->writeValue(dataArray, dataSize, true);
    digitalWrite(SENDING_DATA_PIN, LOW);
    
    // disconnect and go to deep sleep
    pClient->disconnect();
    deepsleep();
  }
}