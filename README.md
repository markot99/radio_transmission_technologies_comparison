# Source code for comparing different radio transmission technologies

This repository contains the source code that was used to analyze the power consumption of different radio transmission technologies. The analysis was carried out as part of my master's thesis.

The experimental comparisons were carried out using the following microcontrollers:
- [WiFi LoRa 32(V3) from Heltec Automation](https://heltec.org/project/wifi-lora-32-v3/)
- [FireBeetle 2 from DFRobot](https://www.dfrobot.com/product-2771.html)

The following Board Manager URLs must be added in Arduino:
- https://resource.heltec.cn/download/package_heltec_esp32_index.json
- https://espressif.github.io/arduino-esp32/package_esp32_index.json

The following device libraries must then be installed in the Boards Manager:
- **Heltec ESP32 Series Dev-boards** by Heltec Automation (Version 3.0.0)
- **esp32** by Espressif Systems (Version 3.0.3)

Further information on using the boards can be found here:
- [WiFi LoRa 32(V3) from Heltec Automation](https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_C6)
- [FireBeetle 2 from DFRobot](https://docs.heltec.org/en/node/esp32/wifi_lora_32/index.html)

The following microcontroller pins are set to indicate specific events:

| Pin Type                    | WiFi LoRa 32(V3) Pin | FireBeetle 2 Pin |
|-----------------------------|----------------------|------------------|
| Boot process completed      | 47                   | 8                |
| Sending data                | 48                   | 14               |

The microcontroller-specific settings are automatically adjusted via preprocessor instructions.