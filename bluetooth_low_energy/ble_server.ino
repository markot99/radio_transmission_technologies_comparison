// the source code is based on this example:
// https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/security/BLE_client/BLE_client_passkey/BLE_client_passkey.ino

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// service and characteristic UUIDs
static BLEUUID serviceUUID("16e9227a-50d8-487f-8e6a-cc58e0e3e3d6");
static BLEUUID characteristicUUID("aba7ff45-9d23-46db-810a-d1c574abda95");

// BLE server variables
BLEServer *pServer = NULL;
bool deviceDisconnected = false;

// maximum data size to receive
#define MAXDATA 100

//! @brief BLE security callbacks, used for encryption
class MySecurity : public BLESecurityCallbacks {
  bool onConfirmPIN(uint32_t pin) {
    return false;
  }

  uint32_t onPassKeyRequest() {
    return 784564;
  }

  void onPassKeyNotify(uint32_t pass_key) {
  }

  bool onSecurityRequest() {
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
    if (cmpl.success) {
      uint16_t length;
      esp_ble_gap_get_whitelist_size(&length);
    }
  }
};

//! @brief BLE server callbacks, used for handling disconnections
class MyServerCallbacks : public BLEServerCallbacks {
  void onDisconnect(BLEServer *pServer) {
    deviceDisconnected = true;
  }
};

//! @brief Setup the BLE server
void setup() {
  // initialize the BLE device
  BLEDevice::init("AirSense");

  // set the security settings
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new MySecurity());

  // create the BLE server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(serviceUUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    characteristicUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  // set the initial value of the characteristic
  uint8_t initialValue[MAXDATA] = { 0 };
  pCharacteristic->setValue(initialValue, MAXDATA);

  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
  pSecurity->setCapability(ESP_IO_CAP_OUT);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
}

void loop() {
  // if the device is disconnected, start advertising again
  if (deviceDisconnected) {
    deviceDisconnected = false;
    delay(500);
    pServer->startAdvertising();
  }
}