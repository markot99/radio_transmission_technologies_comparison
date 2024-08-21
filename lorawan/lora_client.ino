// the source code is based on this example:
// https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/examples/LoRaWAN/LoRaWan/LoRaWan.ino

#include "LoRaWan_APP.h"

#define BOOT_PIN 47
#define SENDING_DATA_PIN 48

// the things network configuration
uint8_t devEui[] = {<SET_DEV_EUI> };
uint8_t appEui[] = {<SET_DEV_EUI> };
uint8_t appKey[] = {<SET_APP_KEY> };

// lorawan configuration
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
DeviceClass_t loraWanClass = CLASS_A;

// deep sleep duration
uint32_t appTxDutyCycle = 3000000;

// true for confirmed messages, false for unconfirmed messages
bool isTxConfirmed = true;

// default LoRaWAN settings from the manufacturer
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
bool overTheAirActivation = true;
bool loraWanAdr = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

//! @brief Set data to send
static void prepareTxFrame(uint8_t port) {
  appDataSize = 5;
  appData[0] = 0;
  appData[1] = 1;
  appData[2] = 2;
  appData[3] = 3;
  appData[4] = 4;
}

//! @brief Setup the LoRaWAN client
void setup() {
  // set boot pin to indicate that the boot process finished
  pinMode(BOOT_PIN, OUTPUT);
  digitalWrite(BOOT_PIN, HIGH);

  // configure sending data pin
  pinMode(SENDING_DATA_PIN, OUTPUT);

  // initialize manufacturer's library
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
}

//! @brief Main loop
void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        LoRaWAN.setDefaultDR(3);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        digitalWrite(SENDING_DATA_PIN, HIGH);
        LoRaWAN.send();
        digitalWrite(SENDING_DATA_PIN, LOW);
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}