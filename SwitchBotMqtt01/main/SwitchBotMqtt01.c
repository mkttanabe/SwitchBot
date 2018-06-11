/**
 * 
 * SwitchBotMqtt01.c (confirmed on ESP-IDF v3.0-dev-2561-g358c822)
 * 
 * SwitchBot remote controller for ESP32.
 * 
 * ESP32 will send the "Press" command to your SwitchBot via BLE,
 * when received MQTT message '{"topic":"msg", "message":"1"}'.
 * 
 * Based on the following official program by Wonderlabs,Inc.
 * https://github.com/OpenWonderLabs/python-host/
 * 
 * Thanks for "ESP32 MQTT Library" by Mr.Turan.
 * https://github.com/tuanpmt/espmqtt
 * 
 * Thanks for CloudMQTT.
 * https://www.cloudmqtt.com/
 * 
 * 
 * Copyright 2018 KLab Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdio.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "mqtt_client.h"

#define TAG          "SW01"
#define TAG_WIFI TAG " [WIFI]"
#define TAG_MQTT TAG " [MQTT]"
#define TAG_GAP  TAG " [GAP]"
#define TAG_GATT TAG " [GATT]"
#define INVALID_HANDLE   0

// for WiFi
#define WIFI_SSID      "ssid"
#define WIFI_PASS      "pass"

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// for MQTT
#define MQTT_HOST       "****.cloudmqtt.com"
#define MQTT_URL        "***************"
#define MQTT_PORT       12345
#define MQTT_CLIENT_ID  "*************"
#define MQTT_USERNAME   "*************"
#define MQTT_PASSWORD   "*************"

// for BLE
// MAC address of your SwitchBot
static esp_bd_addr_t targetDeviceMacAddr = {0xc0, 0x65, 0x9a, 0x00, 0x00, 0x00};
// SwitchBot "Press" command
static uint8_t cmd_press[3] = {0x57, 0x01, 0x00};

// User defined service of SwitchBot
// cba20d00-224d-11e6-9fb8-0002a5d5c51b
static esp_gatt_srvc_id_t user_service1_id = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {
                .uuid128[15]=0xcb,.uuid128[14]=0xa2,.uuid128[13]=0x0d,.uuid128[12]=0x00,
                .uuid128[11]=0x22,.uuid128[10]=0x4d,.uuid128[9] =0x11,.uuid128[8] =0xe6,
                .uuid128[7] =0x9f,.uuid128[6] =0xb8,.uuid128[5] =0x00,.uuid128[4] =0x02,
                .uuid128[3] =0xa5,.uuid128[2] =0xd5,.uuid128[1] =0xc5,.uuid128[0] =0x1b,
                },
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

// User defined characteristic of the service above
// cba20002-224d-11e6-9fb8-0002a5d5c51b
static esp_gatt_id_t user_chr2_id = {
    .uuid = {
        .len = ESP_UUID_LEN_128,
        .uuid = {
            .uuid128[15]=0xcb,.uuid128[14]=0xa2,.uuid128[13]=0x00,.uuid128[12]=0x02,
            .uuid128[11]=0x22,.uuid128[10]=0x4d,.uuid128[9] =0x11,.uuid128[8] =0xe6,
            .uuid128[7] =0x9f,.uuid128[6] =0xb8,.uuid128[5] =0x00,.uuid128[4] =0x02,
            .uuid128[3] =0xa5,.uuid128[2] =0xd5,.uuid128[1] =0xc5,.uuid128[0] =0x1b,
        },
    },
    .inst_id = 0,
};

// BLE scan parameters
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30
};

static bool BLEInitDone = false;
static uint32_t scanDuration = 10; // scan timeout 10 seconds

// GATT client profile
static bool connectedGATT = false;

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

// prototype
static void esp_gattc_cb_sub_PROFILE_A(
    esp_gattc_cb_event_t event,
    esp_gatt_if_t gattc_if, 
    esp_ble_gattc_cb_param_t *param);

// One gatt-based profile one app_id and one gattc_if, 
// this array will store the gattc_if returned by ESP_GATTS_REG_EVT
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = esp_gattc_cb_sub_PROFILE_A, // GATT callback for PROFILE_A
        .gattc_if = ESP_GATT_IF_NONE, // initial value
    },
};

// for 128-bit UUID
void dumpUUID128(uint8_t id[ESP_UUID_LEN_128])
{
    ESP_LOGI(TAG_GATT,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", 
        id[15],id[14],id[13],id[12],id[11],id[10],id[9],id[8],
        id[7], id[6], id[5], id[4], id[3], id[2], id[1],id[0]);
}

// GATT event handler (PROFILE_A)
static void esp_gattc_cb_sub_PROFILE_A(
    esp_gattc_cb_event_t event, 
    esp_gatt_if_t gattc_if, 
    esp_ble_gattc_cb_param_t *param)
{
    uint16_t conn_id = 0;
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    esp_err_t err = 0;

    switch (event) {
    // When GATT client is registered, the event comes
    case ESP_GATTC_REG_EVT: // 0
        ESP_LOGI(TAG_GATT, "ESP_GATTC_REG_EVT");
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;

    case ESP_GATTC_UNREG_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_UNREG_EVT");
        break;

    // When the ble physical connection is set up, the event comes
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_CONNECT_EVT");
        break;

    // When the ble physical connection disconnected, the event comes
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_DISCONNECT_EVT");
        break;

    // When GATT virtual connection is set up, the event comes
    case ESP_GATTC_OPEN_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_DISCONNECT_EVT");
        conn_id = p_data->open.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG_GATT, "ESP_GATTC_OPEN_EVT: conn_id=%d if=%d sts=%d mtu=%d",
                        conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        if (p_data->open.status != 0) {
            connectedGATT = false;
            esp_ble_gap_start_scanning(scanDuration);
            break;
        }
        err = esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
        ESP_LOGI(TAG_GATT, "esp_ble_gattc_search_service: err=%d", err);
        break;

    // When GATT virtual connection is closed, the event comes
    case ESP_GATTC_CLOSE_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_CLOSE_EVT");
        esp_ble_gattc_app_unregister(gattc_if);
        connectedGATT = false;
        break;

    // When GATT service discovery result is got, the event comes
    case ESP_GATTC_SEARCH_RES_EVT: { // 7
        esp_gatt_srvc_id_t *srvc_id =
            (esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;
        conn_id = p_data->search_res.conn_id;
        ESP_LOGI(TAG_GATT, "ESP_GATTC_SEARCH_RES_EVT: conn_id=%x", conn_id);
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGI(TAG_GATT, "svc: uuid16=0x%04x", srvc_id->id.uuid.uuid.uuid16);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
            ESP_LOGI(TAG_GATT, "svc: uuid32=0x%08x", srvc_id->id.uuid.uuid.uuid32);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGI(TAG_GATT, "svc: uuid128=");
            dumpUUID128(srvc_id->id.uuid.uuid.uuid128);
            // found the target service of SwitchBot
            if (memcmp(
                srvc_id->id.uuid.uuid.uuid128, 
                user_service1_id.id.uuid.uuid.uuid128, 
                ESP_UUID_LEN_128) == 0) {
                ESP_LOGI(TAG_GATT, "found Target Service!");
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle =
                    p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle =
                    p_data->search_res.end_handle;
            }
        } else {
            ESP_LOGE(TAG, "svc: UNKNOWN LEN %d", srvc_id->id.uuid.len);
        }
        break;
    }

    // When GATT service discovery is completed
    case ESP_GATTC_SEARCH_CMPL_EVT:
        conn_id = p_data->search_cmpl.conn_id;
        ESP_LOGI(TAG_GATT, "ESP_GATTC_SEARCH_CMPL_EVT: conn_id=%x sts=%d",
                conn_id, p_data->search_cmpl.status);

        // Get the counts of the characteristics under the target service
        uint16_t count = 0;
        esp_gatt_status_t sts = 
            esp_ble_gattc_get_attr_count(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    ESP_GATT_DB_CHARACTERISTIC,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    INVALID_HANDLE,
                    &count);

        ESP_LOGI(TAG_GATT, "esp_ble_gattc_get_attr_count: sts=%d, count=%d",
                            sts, count);

        if (count > 0){
            esp_gattc_char_elem_t *char_elem_result = 
                (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);

            // Get the handle of the target characteristic
            sts = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    user_chr2_id.uuid,
                    char_elem_result,
                    &count);

            gl_profile_tab[PROFILE_A_APP_ID].char_handle =
                char_elem_result[0].char_handle;

            // Write "Press" command
            ESP_LOGI(TAG_GATT, "writing cmd_press");
            esp_ble_gattc_write_char(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                    sizeof(cmd_press),
                    cmd_press,
                    ESP_GATT_WRITE_TYPE_RSP,
                    ESP_GATT_AUTH_REQ_NONE);

            ESP_LOGI(TAG_GATT, "done");

            if (char_elem_result) {
                free(char_elem_result);
            }
            esp_ble_gattc_close(gattc_if, p_data->open.conn_id);
        }
        break;

    // When register for notification of a service completes, the event comes
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(TAG_GATT, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        break;
    }
    
    // When GATT notification or indication arrives, the event comes
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_NOTIFY_EVT: len=%d val=%08x",
                p_data->notify.value_len, *(uint32_t *)p_data->notify.value);
        break;

    // When GATT characteristic write operation completes, the event comes
    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_WRITE_CHAR_EVT: sts=%d",
            p_data->write.status);
        // Disconnect
        esp_ble_gattc_close(gattc_if, conn_id);
        break;

    // When GATT characteristic descriptor write completes, the event comes
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(TAG_GATT, "ESP_GATTC_WRITE_DESCR_EVT: sts=%d",
            p_data->write.status);
        break;
    default:
        break;
    }
}

// GATT event handler
static void esp_gattc_cb(
    esp_gattc_cb_event_t event,
    esp_gatt_if_t gattc_if,
    esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGI(TAG_GATT, "\tevent=%d if=%d", event, gattc_if);

    // If event is register event, store the gattc_if for each profile
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(TAG_GATT, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id, 
                    param->reg.status);
            return;
        }
    }
    // dispatch
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE ||
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

// GAP event handler
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    //ESP_LOGI(TAG_GAP, "esp_gap_cb event=%d", event);
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGI(TAG_GAP, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        esp_ble_gap_start_scanning(scanDuration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG_GAP, "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
        // scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Scan start failed");
        }
        break;

     case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG_GAP, "ESP_GAP_BLE_SEC_REQ_EVT");
        // send the positive(true) security response to the peer device
        // to accept the security request.
        // If not accept the security request, should sent the security
        // response with negative(false) accept value
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        ESP_LOGI(TAG_GAP, "ESP_GAP_BLE_SCAN_RESULT_EVT");
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        switch (scan_result->scan_rst.search_evt) {
        ESP_LOGI(TAG_GAP, "scan_rst.search_evt=%d", scan_result->scan_rst.search_evt);
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            ESP_LOGI(TAG_GAP, "ESP_GAP_SEARCH_INQ_RES_EVT");
            // MAC address of the peripheral
            esp_bd_addr_t mac;
            memcpy(mac, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
            ESP_LOGI(TAG_GAP, "found: MAC addr=[%02x:%02x:%02x:%02x:%02x:%02x]",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

            // Found your SwitchBot
            if (memcmp(scan_result->scan_rst.bda, targetDeviceMacAddr,
                        ESP_BD_ADDR_LEN) == 0) {
                ESP_LOGI(TAG_GAP, "found Target Device!");
                if (connectedGATT == false) {
                    connectedGATT = true;
                    ESP_LOGI(TAG_GAP, "Connecting to the device..");
                    esp_ble_gap_stop_scanning();
                    esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                        scan_result->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
                 }
            } else {
                ESP_LOGI(TAG_GAP, "not Target device..");
            }
            
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG_GAP, "ESP_GAP_SEARCH_INQ_CMPL_EVT");
            // Unregister the GATTC App when scan timeout occurs
            // See "ESP_GATTC_CLOSE_EVT"
            if (gl_profile_tab[PROFILE_A_APP_ID].gattc_if != ESP_GATT_IF_NONE) {
                esp_ble_gattc_app_unregister(gl_profile_tab[PROFILE_A_APP_ID].gattc_if);
            }
            break;

        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

// Register BLE callbacks
void BLE_client_appRegister(void)
{
    esp_err_t sts;

    ESP_LOGI(TAG, "register BLE callback..");

    // register the scan callback function to the gap moudule
    if ((sts = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register error, error code=%x", sts);
        return;
    }
    // register the callback function to the gattc module
    if ((sts = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gattc register error, error code=%x", sts);
        return;
    }
    esp_ble_gattc_app_register(PROFILE_A_APP_ID);
}


// Init BLE central
void BLE_central_init()
{
    if (!BLEInitDone) {
        BLEInitDone = true;
        esp_err_t err = esp_bluedroid_enable();
        ESP_LOGE(TAG, "esp_bluedroid_enable: ret=%x", err);
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_bt_controller_init(&bt_cfg);
        esp_bt_controller_enable(ESP_BT_MODE_BTDM);
        esp_bluedroid_init();
        esp_bluedroid_enable();
    }
    BLE_client_appRegister();
}

// MQTT event handler
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    switch (event->event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
            // test
            msg_id = esp_mqtt_client_publish(client, "msg", "from ESP32", 0, 0, 0);
            ESP_LOGI(TAG_MQTT, "published, msg_id=%d", msg_id);

            // Target topic name
            msg_id = esp_mqtt_client_subscribe(client, "msg", 0);
            ESP_LOGI(TAG_MQTT, "subscribed, msg_id=%d", msg_id);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG_MQTT, "topic[%.*s] data=[%.*s]",
                event->topic_len, event->topic, event->data_len, event->data);
            if (event->data[0] == '1') {
                // Got message: '{"topic":"msg", "message":"1"}'
                BLE_central_init();
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}

// Init MQTT
static void mqtt_init(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .host      = MQTT_HOST,
        .uri       = MQTT_URL,
        .port      = MQTT_PORT,
        .client_id = MQTT_CLIENT_ID,
        .username  = MQTT_USERNAME,
        .password  = MQTT_PASSWORD,
        .event_handle = mqtt_event_handler
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    // Wait for WiFi connection
    ESP_LOGI(TAG_MQTT, "waiting for WiFi Connection");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                            false, true, portMAX_DELAY);
    // Start MQTT session
    esp_mqtt_client_start(client);
}

// WiFi event handler
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG_WIFI, "connected!");
        ESP_LOGI(TAG_WIFI, "SYSTEM_EVENT_STA_GOT_IP ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        // Set the CONNECTED_BIT
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;

    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG_WIFI, "SYSTEM_EVENT_AP_STACONNECTED station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG_WIFI, "SYSTEM_EVENT_AP_STADISCONNECTED station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG_WIFI, "SYSTEM_EVENT_STA_DISCONNECTED");
        // Clear the CONNECTED_BIT
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        // Reconnect
        esp_wifi_connect();
        break;

    default:
        break;
    }
    return ESP_OK;
}

// Init WiFi 
void wifi_init()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG_WIFI, "connecting to AP [%s]..", WIFI_SSID);
}

// Entry point
void app_main()
{
    nvs_flash_init();
    wifi_init();
    mqtt_init();

}
