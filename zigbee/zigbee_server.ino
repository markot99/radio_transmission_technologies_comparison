// the source code is based on this example:
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Zigbee/Zigbee_Thermostat/Zigbee_Thermostat.ino

#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof(arr[0]))

// zigbee settings
#define CUSTOM_CLUSTER_ID 0xFFF2
#define CUSTROM_ATTRIBUTE_1_ID 0x0001U

#define MANUFACTURER_NAME \
  "\x0B" \
  "ESPRESSIF"
#define MODEL_IDENTIFIER "\x09" CONFIG_IDF_TARGET

#define ESP_ZB_ZC_CONFIG() \
  { \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR, .install_code_policy = INSTALLCODE_POLICY_ENABLE, .nwk_cfg = { \
      .zczr_cfg = { \
        .max_children = MAX_CHILDREN, \
      }, \
    } \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { .radio_mode = ZB_RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, }

typedef struct device_params_s {
  esp_zb_ieee_addr_t ieee_addr;
  uint8_t endpoint;
  uint16_t short_addr;
} device_params_t;

typedef struct zbdata_s {
  uint8_t len;
  uint8_t data[];
} ESP_ZB_PACKED_STRUCT zbdata_t;

static device_params_t device;

#define MAX_CHILDREN 10                                                  
#define INSTALLCODE_POLICY_ENABLE false                                  
#define HA_ENDPOINT 1                                        
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK 

//! @brief Callback to start top level commissioning
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

//! @brief Callback to handle the binding request
static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
  esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)user_ctx;

  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    /* Local binding succeeds */
    if (bind_req->req_dst_addr == esp_zb_get_short_address()) {
      log_i("Successfully bind the device sensor from address(0x%x) on endpoint(%d)", device.short_addr, device.endpoint);

      /*Read string */
      esp_zb_zcl_read_attr_cmd_t read_req2;
      read_req2.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
      read_req2.zcl_basic_cmd.src_endpoint = HA_ENDPOINT;
      read_req2.zcl_basic_cmd.dst_endpoint = device.endpoint;
      read_req2.zcl_basic_cmd.dst_addr_u.addr_short = device.short_addr;
      read_req2.clusterID = CUSTOM_CLUSTER_ID;

      uint16_t attributes2[] = {
        CUSTROM_ATTRIBUTE_1_ID,
      };
      read_req2.attr_number = ARRAY_LENGTH(attributes2);
      read_req2.attr_field = attributes2;

      esp_zb_zcl_read_attr_cmd_req(&read_req2);
    }
    if (bind_req->req_dst_addr == device.short_addr) {
      log_i("The device from address(0x%x) on endpoint(%d) successfully binds us", device.short_addr, device.endpoint);
    }
    free(bind_req);
  }
}

//! @brief Callback to handle the finding of the device
static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
  if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
    /* Store the information of the remote device */
    device_params_t *sensor = (device_params_t *)user_ctx;
    sensor->endpoint = endpoint;
    sensor->short_addr = addr;
    esp_zb_ieee_address_by_short(sensor->short_addr, sensor->ieee_addr);
    log_d("Device found: short address(0x%x), endpoint(%d)", sensor->short_addr, sensor->endpoint);

    /* 1. Send binding request to the sensor */
    esp_zb_zdo_bind_req_param_t *bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
    bind_req->req_dst_addr = addr;
    log_d("Request device to bind us");

    /* populate the src information of the binding */
    memcpy(bind_req->src_address, sensor->ieee_addr, sizeof(esp_zb_ieee_addr_t));
    bind_req->src_endp = endpoint;
    bind_req->cluster_id = CUSTOM_CLUSTER_ID;
    log_d("Bind device sensor");

    /* populate the dst information of the binding */
    bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    esp_zb_get_long_address(bind_req->dst_address_u.addr_long);
    bind_req->dst_endp = HA_ENDPOINT;

    log_i("Request device sensor to bind us");
    esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);

    /* 2. Send binding request to self */
    bind_req = (esp_zb_zdo_bind_req_param_t *)calloc(sizeof(esp_zb_zdo_bind_req_param_t), 1);
    bind_req->req_dst_addr = esp_zb_get_short_address();

    /* populate the src information of the binding */
    esp_zb_get_long_address(bind_req->src_address);
    bind_req->src_endp = HA_ENDPOINT;
    bind_req->cluster_id = CUSTOM_CLUSTER_ID;

    /* populate the dst information of the binding */
    bind_req->dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    memcpy(bind_req->dst_address_u.addr_long, sensor->ieee_addr, sizeof(esp_zb_ieee_addr_t));
    bind_req->dst_endp = endpoint;

    log_i("Bind device sensor");
    esp_zb_zdo_device_bind_req(bind_req, bind_cb, bind_req);
  }
}

//! @brief Callback to find the device
static void find_device(esp_zb_zdo_match_desc_req_param_t *param, esp_zb_zdo_match_desc_callback_t user_cb, void *user_ctx) {
  uint16_t cluster_list[] = { CUSTOM_CLUSTER_ID };
  param->profile_id = ESP_ZB_AF_HA_PROFILE_ID;
  param->num_in_clusters = 1;
  param->num_out_clusters = 0;
  param->cluster_list = cluster_list;
  esp_zb_zdo_match_cluster(param, user_cb, (void *)&device);
}

//! @brief Zigbee signal handler
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      log_i("Zigbee stack initialized");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        log_i("Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new()) {
          log_i("Start network formation");
          esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
          log_i("Device rebooted");
          log_i("Openning network for joining for %d seconds", 7200);
          esp_zb_bdb_open_network(7200);
        }
      } else {
        log_e("Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        log_i(
          "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
          extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        log_i("Restart network formation (status: %s)", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        log_i("Network steering started");
      }
      break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
      dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
      log_i("New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
      esp_zb_zdo_match_desc_req_param_t cmd_req;
      cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
      cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
      find_device(&cmd_req, user_find_cb, NULL);
      break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
      if (err_status == ESP_OK) {
        if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
          log_i("Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
        } else {
          log_w("Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
        }
      }
      break;
    default: log_i("ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status)); break;
  }
}

//! @brief Zigbee attribute handler
static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute) {
  if ((cluster_id == CUSTOM_CLUSTER_ID) && (attribute->id == CUSTROM_ATTRIBUTE_1_ID) && (attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING)) {
    zbdata_t *zbstr = (zbdata_t *)attribute->data.value;

    if (zbstr == nullptr) {
      log_i("zbstr nullptr");
      return;
    }

    log_i("len: %d", zbstr->len);

    for (int i = 0; i < zbstr->len; i++) {
      log_i("Data is '%d'", zbstr->data[i]);
    }
  }
}

//! @brief Zigbee read attribute response handler
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message) {
  if (!message) {
    log_e("Empty message");
  }
  if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Received message: error status(%d)", message->info.status);
  }
  log_i(
    "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->info.src_address.u.short_addr,
    message->info.src_endpoint, message->info.dst_endpoint, message->info.cluster);

  esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
  while (variable) {
    log_i(
      "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status, message->info.cluster,
      variable->attribute.id, variable->attribute.data.type, variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
    if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
      esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
    }

    variable = variable->next;
  }

  return ESP_OK;
}

//! @brief Zigbee action handler
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID: ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message); break;
    default: log_w("Receive Zigbee action(0x%x) callback", callback_id); break;
  }
  return ret;
}

//! @brief Create custom clusters
static esp_zb_cluster_list_t *custom_clusters_create(esp_zb_thermostat_cfg_t *thermostat) {
  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(thermostat->basic_cfg));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)MANUFACTURER_NAME));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)MODEL_IDENTIFIER));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(thermostat->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
  return cluster_list;
}

//! @brief Create custom endpoint
static esp_zb_ep_list_t *custom_ep_create(uint8_t endpoint_id, esp_zb_thermostat_cfg_t *thermostat) {
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
  esp_zb_endpoint_config_t endpoint_config = {
    .endpoint = endpoint_id, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID, .app_device_version = 0
  };
  esp_zb_ep_list_add_ep(ep_list, custom_clusters_create(thermostat), endpoint_config);
  return ep_list;
}

//! @brief Zigbee task
static void esp_zb_task(void *pvParameters) {
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
  esp_zb_init(&zb_nwk_cfg);

  esp_zb_thermostat_cfg_t thermostat_cfg = ESP_ZB_DEFAULT_THERMOSTAT_CONFIG();
  esp_zb_ep_list_t *esp_zb_thermostat_ep = custom_ep_create(HA_ENDPOINT, &thermostat_cfg);
  esp_zb_device_register(esp_zb_thermostat_ep);

  esp_zb_core_action_handler_register(zb_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}

//! @brief Setup the Zigbee coordinator
void setup() {
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };

  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  // start Zigbee task
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void loop() {
  delay(1000);
}