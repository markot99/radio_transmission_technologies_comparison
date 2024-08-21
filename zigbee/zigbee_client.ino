// the source code is based on this example:
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Zigbee/Zigbee_Temperature_Sensor/Zigbee_Temperature_Sensor.ino

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#define BOOT_PIN 8

// zigbee settings
#define CUSTOM_CLUSTER_ID 0xFFF2
#define CUSTROM_ATTRIBUTE_1_ID 0x0001U

#define MANUFACTURER_NAME \
  "\x0B" \
  "ESPRESSIF"
#define MODEL_IDENTIFIER "\x09" CONFIG_IDF_TARGET

#define ESP_ZB_ZED_CONFIG() \
  { \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
    .nwk_cfg = { \
      .zed_cfg = { \
        .ed_timeout = ED_AGING_TIMEOUT, \
        .keep_alive = ED_KEEP_ALIVE, \
      }, \
    }, \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { .radio_mode = ZB_RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, }

#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000
#define ESP_ENDPOINT 10
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

// deep sleep duration
#define SLEEP_INTERVALL_SEC 5

// flag to indicate if the device is connected to the network
static bool connected = false;

//! @brief Callback to start top level commissioning
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

//! @brief Zigbee signal handler
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      log_i("Zigbee stack initialized");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        log_i("Start network steering");
        log_i("Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new()) {
          log_i("Start network steering");
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
          log_i("Device rebooted");
          connected = true;
        }
      } else {
        log_w("Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        log_i(
          "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
          extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

        connected = true;
      } else {
        log_i("Network steering was not successful (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    default: log_i("ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status)); break;
  }
}

//! @brief Zigbee task
static void esp_zb_task(void *pvParameters) {
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
  esp_zb_init(&zb_nwk_cfg);

  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = ESP_ENDPOINT,
    .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
    .app_device_version = 0,
  };
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

  // custom clusters
  esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);

  uint8_t data[] = { 1, 2, 3, 4, 5 };

  // Create z_data with an extra byte
  uint8_t z_data[sizeof(data) + 1] = { sizeof(data) };

  // Copy data to z_data starting from the second position (index 1)
  memcpy(z_data + 1, data, sizeof(data));

  esp_zb_custom_cluster_add_custom_attr(custom_cluster, CUSTROM_ATTRIBUTE_1_ID, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, z_data);
  esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
  esp_zb_device_register(ep_list);


  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}

//! @brief Setup the Zigbee client
void setup() {
  // set boot pin to indicate that the boot process finished
  pinMode(BOOT_PIN, OUTPUT);
  digitalWrite(BOOT_PIN, HIGH);

  // init Zigbee
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  // configure deep sleep
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVALL_SEC * 1000000ULL);

  // start Zigbee task
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
  // wait until connected
  while (!connected) {
    // wait until data is sent and go to deep sleep
    delay(100);
  }
  delay(150);
  esp_deep_sleep_start();
}