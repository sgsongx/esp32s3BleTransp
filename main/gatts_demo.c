/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ble_transp_config.h"

#define TAG "bletransp"
#define SERVICE_UUID 0x00FF
#define DATA_UUID 0xFF01
#define PACKET_SIZE 512
#define DEFAULT_CONN_INTERVAL 24 /* 24 * 1.25 ms = 30 ms */

_Static_assert(BLE_TRANSP_UART_NUM >= 0 && BLE_TRANSP_UART_NUM < UART_NUM_MAX,
               "Invalid UART peripheral");
_Static_assert(BLE_TRANSP_UART_BAUD_RATE > 0, "Invalid UART baud rate");
#if CONFIG_ESP_CONSOLE_UART
_Static_assert(BLE_TRANSP_UART_NUM != CONFIG_ESP_CONSOLE_UART_NUM,
               "Use a different UART from the log console, or move the console to USB");
#endif
_Static_assert(BLE_TRANSP_PHY == ESP_BLE_GAP_PHY_1M_PREF_MASK ||
               BLE_TRANSP_PHY == ESP_BLE_GAP_PHY_2M_PREF_MASK ||
               BLE_TRANSP_PHY == ESP_BLE_GAP_PHY_CODED_PREF_MASK,
               "Select one BLE PHY: 1M, 2M or Coded");

enum { ATTR_SERVICE, ATTR_DECLARATION, ATTR_DATA, ATTR_CCCD, ATTR_COUNT };
static uint16_t handles[ATTR_COUNT];
static const uint16_t primary_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t cccd_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t service_uuid = SERVICE_UUID;
static const uint16_t data_uuid = DATA_UUID;
static const uint8_t properties = ESP_GATT_CHAR_PROP_BIT_WRITE |
                                ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
                                ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t initial_cccd[2] = {0, 0};
static const esp_gatts_attr_db_t attributes[ATTR_COUNT] = {
    [ATTR_SERVICE] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_uuid, ESP_GATT_PERM_READ,
         sizeof(service_uuid), sizeof(service_uuid), (uint8_t *)&service_uuid}},
    [ATTR_DECLARATION] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(properties), sizeof(properties), (uint8_t *)&properties}},
    [ATTR_DATA] = {{ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_16, (uint8_t *)&data_uuid, ESP_GATT_PERM_WRITE,
         PACKET_SIZE, 0, NULL}},
    [ATTR_CCCD] = {{ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_16, (uint8_t *)&cccd_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(initial_cccd), sizeof(initial_cccd), (uint8_t *)initial_cccd}},
};

static esp_ble_gap_ext_adv_params_t adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
    .interval_min = 0x20,
    .interval_max = 0x40,
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
};
static esp_ble_gap_ext_adv_t adv = {.instance = 0, .duration = 0, .max_events = 0};
static uint8_t adv_data[] = {
    2, ESP_BLE_AD_TYPE_FLAG, ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
    3, ESP_BLE_AD_TYPE_16SRV_CMPL, SERVICE_UUID & 0xff, SERVICE_UUID >> 8,
    10, ESP_BLE_AD_TYPE_NAME_CMPL, 'b', 'l', 'e', 't', 'r', 'a', 'n', 's', 'p',
    5, 0x12, DEFAULT_CONN_INTERVAL, 0, DEFAULT_CONN_INTERVAL, 0,
};
_Static_assert(sizeof(adv_data) <= 31, "Legacy advertising data exceeds 31 bytes");

typedef struct {
    uint16_t len;
    uint8_t data[PACKET_SIZE];
} uart_packet_t;

static QueueHandle_t uart_tx_queue;
static QueueHandle_t uart_events;
static SemaphoreHandle_t link_mutex;
/* All link fields are protected by link_mutex; no UART I/O while holding it. */
static struct {
    esp_gatt_if_t gatts_if;
    uint16_t conn_id;
    uint16_t mtu;
    bool connected;
    bool notify_enabled;
    bool congested;
    uint32_t generation;
} link = {.gatts_if = ESP_GATT_IF_NONE, .mtu = 23};

static void log_error(const char *operation, esp_err_t error)
{
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "%s: %s", operation, esp_err_to_name(error));
    }
}

static esp_gatt_status_t handle_write(const esp_ble_gatts_cb_param_t *param)
{
    if (param->write.is_prep) {
        return ESP_GATT_REQ_NOT_SUPPORTED;
    }
    if (param->write.offset != 0) {
        return ESP_GATT_INVALID_OFFSET;
    }
    if (param->write.len && param->write.value == NULL) {
        return ESP_GATT_INVALID_PDU;
    }
    if (param->write.handle == handles[ATTR_CCCD]) {
        if (param->write.len != 2) {
            return ESP_GATT_INVALID_ATTR_LEN;
        }
        uint16_t value = param->write.value[0] | (param->write.value[1] << 8);
        if (value != 0 && value != 1) {
            return ESP_GATT_CCC_CFG_ERR;
        }
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        if (link.notify_enabled != (value == 1)) {
            ++link.generation;
        }
        link.notify_enabled = (value == 1);
        xSemaphoreGive(link_mutex);
        ESP_LOGI(TAG, "UART notifications %s", value ? "enabled" : "disabled");
        return ESP_GATT_OK;
    }
    if (param->write.handle != handles[ATTR_DATA]) {
        return ESP_GATT_WRITE_NOT_PERMIT;
    }
    if (param->write.len > PACKET_SIZE) {
        return ESP_GATT_INVALID_ATTR_LEN;
    }
    if (param->write.len == 0) {
        return ESP_GATT_OK;
    }
    uart_packet_t packet = {.len = param->write.len};
    memcpy(packet.data, param->write.value, packet.len);
    /* The Bluetooth callback must not wait for a slow UART. */
    return xQueueSend(uart_tx_queue, &packet, 0) == pdTRUE ?
           ESP_GATT_OK : ESP_GATT_NO_RESOURCES;
}

static void uart_tx_task(void *arg)
{
    uart_packet_t packet;
    for (;;) {
        if (xQueueReceive(uart_tx_queue, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        size_t offset = 0;
        while (offset < packet.len) {
            int sent = uart_write_bytes(BLE_TRANSP_UART_NUM, packet.data + offset,
                                        packet.len - offset);
            if (sent <= 0) {
                ESP_LOGE(TAG, "UART TX failed, lost %u bytes", (unsigned)(packet.len - offset));
                break;
            }
            offset += sent;
        }
    }
}

static void notify_uart_data(uint8_t *data, size_t len, uint32_t generation)
{
    size_t offset = 0;
    while (offset < len) {
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        if (!link.connected || !link.notify_enabled || link.generation != generation) {
            xSemaphoreGive(link_mutex);
            return;
        }
        size_t chunk = link.mtu - 3;
        if (chunk > PACKET_SIZE) {
            chunk = PACKET_SIZE;
        }
        if (chunk > len - offset) {
            chunk = len - offset;
        }
        bool congested = link.congested;
        esp_gatt_if_t gatts_if = link.gatts_if;
        uint16_t conn_id = link.conn_id;
        xSemaphoreGive(link_mutex);
        /* Entering the BTC queue while holding link_mutex can deadlock its callbacks. */
        esp_err_t error = ESP_FAIL;
        if (!congested && esp_ble_get_cur_sendable_packets_num(conn_id) > 0) {
            error = esp_ble_gatts_send_indicate(gatts_if, conn_id,
                                               handles[ATTR_DATA], chunk, data + offset, false);
        }
        if (error == ESP_OK) {
            offset += chunk;
        } else if (error == ESP_ERR_INVALID_STATE) {
            return;
        } else {
            /* Keep this chunk until the controller has room, without blocking Bluetooth. */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void uart_rx_task(void *arg)
{
    uart_event_t event;
    uint8_t data[PACKET_SIZE];
    uint32_t observed_generation = 0;
    for (;;) {
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        uint32_t generation = link.generation;
        bool ready = link.connected && link.notify_enabled;
        xSemaphoreGive(link_mutex);
        if (observed_generation != generation) {
            /* Drop RX backlog at connection/subscription boundaries. */
            log_error("Flush UART RX", uart_flush_input(BLE_TRANSP_UART_NUM));
            xQueueReset(uart_events);
            observed_generation = generation;
            continue;
        }
        while (xQueueReceive(uart_events, &event, 0) == pdTRUE) {
            if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART RX overflow; discarding buffered input. Reduce sender rate or enable RTS/CTS");
                log_error("Flush UART RX", uart_flush_input(BLE_TRANSP_UART_NUM));
                xQueueReset(uart_events);
                break;
            } else if (event.type == UART_PARITY_ERR || event.type == UART_FRAME_ERR) {
                ESP_LOGW(TAG, "UART RX parity/frame error (%d)", event.type);
            }
        }
        /* Read the ring directly: UART_DATA events can be lost when their queue is full. */
        int received = uart_read_bytes(BLE_TRANSP_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(20));
        if (received > 0 && ready) {
            notify_uart_data(data, received, generation);
        } else if (received < 0) {
            ESP_LOGE(TAG, "UART RX failed");
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static void start_advertising(void)
{
    log_error("Start advertising", esp_ble_gap_ext_adv_start(1, &adv));
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_ERROR_CHECK(param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
        ESP_ERROR_CHECK(esp_ble_gap_config_ext_adv_data_raw(0, sizeof(adv_data), adv_data));
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        ESP_ERROR_CHECK(param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
        start_advertising();
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising as bletransp");
        } else {
            ESP_LOGE(TAG, "Advertising failed, status %d", param->ext_adv_start.status);
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Connection update: status %d, interval %.2f ms, latency %u, timeout %u ms",
                 param->update_conn_params.status, param->update_conn_params.conn_int * 1.25,
                 param->update_conn_params.latency, param->update_conn_params.timeout * 10);
        break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        ESP_LOGI(TAG, "PHY update: status %d, TX %u, RX %u (1=1M, 2=2M, 3=Coded)",
                 param->phy_update.status, param->phy_update.tx_phy, param->phy_update.rx_phy);
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_ERROR_CHECK(param->reg.status == ESP_GATT_OK ? ESP_OK : ESP_FAIL);
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        link.gatts_if = gatts_if;
        xSemaphoreGive(link_mutex);
        ESP_ERROR_CHECK(esp_ble_gap_set_device_name("bletransp"));
        ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(attributes, gatts_if, ATTR_COUNT, 0));
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        ESP_ERROR_CHECK(param->add_attr_tab.status == ESP_GATT_OK &&
                        param->add_attr_tab.num_handle == ATTR_COUNT ? ESP_OK : ESP_FAIL);
        memcpy(handles, param->add_attr_tab.handles, sizeof(handles));
        ESP_ERROR_CHECK(esp_ble_gatts_start_service(handles[ATTR_SERVICE]));
        break;
    case ESP_GATTS_START_EVT:
        ESP_ERROR_CHECK(param->start.status == ESP_GATT_OK ? ESP_OK : ESP_FAIL);
        ESP_ERROR_CHECK(esp_ble_gap_ext_adv_set_params(0, &adv_params));
        break;
    case ESP_GATTS_CONNECT_EVT: {
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        if (link.connected) {
            xSemaphoreGive(link_mutex);
            log_error("Reject additional peer", esp_ble_gatts_close(gatts_if, param->connect.conn_id));
            break;
        }
        link.conn_id = param->connect.conn_id;
        link.connected = true;
        link.notify_enabled = false;
        link.congested = false;
        link.mtu = 23;
        ++link.generation;
        xSemaphoreGive(link_mutex);
        esp_ble_conn_update_params_t params = {
            .min_int = DEFAULT_CONN_INTERVAL,
            .max_int = DEFAULT_CONN_INTERVAL,
            .latency = 0,
            .timeout = 400,
        };
        memcpy(params.bda, param->connect.remote_bda, sizeof(params.bda));
        /* Request once; later connection updates from the central remain in effect. */
        log_error("Request 30 ms interval", esp_ble_gap_update_conn_params(&params));
        log_error("Request PHY", esp_ble_gap_set_preferred_phy(param->connect.remote_bda, 0,
                   BLE_TRANSP_PHY, BLE_TRANSP_PHY, BLE_TRANSP_CODED_PHY_OPTIONS));
        ESP_LOGI(TAG, "Connected, conn_id %u", param->connect.conn_id);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        bool active = link.connected && link.conn_id == param->disconnect.conn_id;
        if (active) {
            link.connected = false;
            link.notify_enabled = false;
            link.congested = false;
            link.mtu = 23;
            ++link.generation;
        }
        xSemaphoreGive(link_mutex);
        if (active) {
            ESP_LOGI(TAG, "Disconnected, reason 0x%02x", param->disconnect.reason);
            start_advertising();
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        if (link.connected && link.conn_id == param->mtu.conn_id && param->mtu.mtu >= 23) {
            link.mtu = param->mtu.mtu;
        }
        xSemaphoreGive(link_mutex);
        ESP_LOGI(TAG, "MTU %u", param->mtu.mtu);
        break;
    case ESP_GATTS_CONGEST_EVT:
        xSemaphoreTake(link_mutex, portMAX_DELAY);
        if (link.connected && link.conn_id == param->congest.conn_id) {
            link.congested = param->congest.congested;
        }
        xSemaphoreGive(link_mutex);
        break;
    case ESP_GATTS_WRITE_EVT: {
        esp_gatt_status_t status = ESP_GATT_ERROR;
        if (link.connected && link.conn_id == param->write.conn_id) {
            status = handle_write(param);
        }
        if (param->write.need_rsp) {
            log_error("Write response", esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                       param->write.trans_id, status, NULL));
        }
        if (status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "BLE write rejected: status 0x%02x, len %u", status, param->write.len);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        log_error("Execute response", esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id,
                   param->exec_write.trans_id, ESP_GATT_REQ_NOT_SUPPORTED, NULL));
        break;
    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t response = {0};
        esp_gatt_status_t status = ESP_GATT_READ_NOT_PERMIT;
        if (param->read.handle == handles[ATTR_CCCD]) {
            if (param->read.offset > 2) {
                status = ESP_GATT_INVALID_OFFSET;
            } else {
                uint8_t value[2] = {link.notify_enabled ? 1 : 0, 0};
                response.attr_value.handle = param->read.handle;
                response.attr_value.offset = param->read.offset;
                response.attr_value.len = 2 - param->read.offset;
                memcpy(response.attr_value.value, value + param->read.offset, response.attr_value.len);
                status = ESP_GATT_OK;
            }
        }
        log_error("Read response", esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                   param->read.trans_id, status, &response));
        break;
    }
    default:
        break;
    }
}

void app_main(void)
{
    link_mutex = xSemaphoreCreateMutex();
    uart_tx_queue = xQueueCreate(8, sizeof(uart_packet_t));
    ESP_ERROR_CHECK(link_mutex && uart_tx_queue ? ESP_OK : ESP_ERR_NO_MEM);
    uart_config_t uart_config = {
        .baud_rate = BLE_TRANSP_UART_BAUD_RATE,
        .data_bits = BLE_TRANSP_UART_DATA_BITS,
        .parity = BLE_TRANSP_UART_PARITY,
        .stop_bits = BLE_TRANSP_UART_STOP_BITS,
        .flow_ctrl = BLE_TRANSP_UART_FLOW_CTRL,
        .rx_flow_ctrl_thresh = 64,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(BLE_TRANSP_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BLE_TRANSP_UART_NUM, BLE_TRANSP_UART_TX_PIN,
                    BLE_TRANSP_UART_RX_PIN, BLE_TRANSP_UART_RTS_PIN, BLE_TRANSP_UART_CTS_PIN));
    ESP_ERROR_CHECK(uart_driver_install(BLE_TRANSP_UART_NUM, 8192, 0, 20, &uart_events, 0));
    ESP_LOGI(TAG, "UART%d: TX=%d RX=%d baud=%d", BLE_TRANSP_UART_NUM,
             BLE_TRANSP_UART_TX_PIN, BLE_TRANSP_UART_RX_PIN, BLE_TRANSP_UART_BAUD_RATE);

    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_config));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    esp_bluedroid_config_t host_config = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&host_config));
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));
    ESP_ERROR_CHECK(esp_ble_gap_set_preferred_default_phy(BLE_TRANSP_PHY, BLE_TRANSP_PHY));
    ESP_ERROR_CHECK(xTaskCreate(uart_tx_task, "uart_tx", 3072, NULL, 10, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL) == pdPASS ?
                    ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
}
