/* Host regression tests: include the production implementation unchanged. */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>

#define TRUE 1
#define BLE_50_FEATURE_SUPPORT TRUE
#define BLE_50_EXTEND_ADV_EN TRUE
#define BLE_INCLUDED TRUE
#define CONFIG_BT_BLE_50_FEATURES_SUPPORTED 1
#define CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG 1
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_NVS_NO_FREE_PAGES 0x110d
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1110
typedef int esp_err_t;
#define ESP_ERROR_CHECK(value) assert((value) == ESP_OK)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)

typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(value) (value)
int xSemaphoreTake(SemaphoreHandle_t semaphore, unsigned timeout);
int xSemaphoreGive(SemaphoreHandle_t semaphore);
int xQueueSend(QueueHandle_t queue, const void *item, unsigned timeout);
int xQueueReceive(QueueHandle_t queue, void *item, unsigned timeout);
int xQueueReset(QueueHandle_t queue);
QueueHandle_t xQueueCreate(unsigned count, unsigned size);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xTaskCreate(void (*task)(void *), const char *name, unsigned stack,
                       void *arg, unsigned priority, void *handle);
void vTaskDelay(unsigned ticks);

enum { UART_NUM_0, UART_NUM_1, UART_NUM_2, UART_NUM_MAX };
enum { UART_DATA_8_BITS, UART_PARITY_DISABLE, UART_STOP_BITS_1,
       UART_HW_FLOWCTRL_DISABLE, UART_SCLK_DEFAULT };
enum { UART_DATA, UART_FIFO_OVF, UART_BUFFER_FULL, UART_PARITY_ERR, UART_FRAME_ERR };
#define UART_PIN_NO_CHANGE (-1)
typedef struct { int type; size_t size; } uart_event_t;
typedef struct {
    int baud_rate, data_bits, parity, stop_bits, flow_ctrl;
    unsigned rx_flow_ctrl_thresh;
    int source_clk;
} uart_config_t;
int uart_write_bytes(int uart, const void *data, size_t size);
int uart_read_bytes(int uart, void *data, unsigned size, unsigned timeout);
esp_err_t uart_flush_input(int uart);
esp_err_t uart_param_config(int uart, const uart_config_t *config);
esp_err_t uart_set_pin(int uart, int tx, int rx, int rts, int cts);
esp_err_t uart_driver_install(int uart, int rx, int tx, int count,
                                    QueueHandle_t *events, int flags);
const char *esp_err_to_name(esp_err_t error);
typedef struct { int unused; } esp_bt_controller_config_t;
typedef struct { int unused; } esp_bluedroid_config_t;
#define ESP_BT_MODE_CLASSIC_BT 1
#define ESP_BT_MODE_BLE 2
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() {0}
#define BT_BLUEDROID_INIT_CONFIG_DEFAULT() {0}
esp_err_t esp_bt_controller_mem_release(int mode);
esp_err_t esp_bt_controller_init(const esp_bt_controller_config_t *config);
esp_err_t esp_bt_controller_enable(int mode);
esp_err_t esp_bluedroid_init_with_cfg(const esp_bluedroid_config_t *config);
esp_err_t esp_bluedroid_enable(void);
esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);

#include "../main/gatts_demo.c"

uart_packet_t queued[8];
unsigned queue_count, lock_count, delay_count, send_count, fail_sends;
unsigned advertising_starts, responses, closed_connections;
uint16_t sendable = 1;
esp_gatt_status_t last_status;
esp_gatt_rsp_t last_response;
uint8_t notifications[4096];
size_t notification_length, notification_chunks[64];
void (*on_delay)(void);
void (*after_send)(void);
esp_ble_conn_update_params_t requested_connection;
static jmp_buf task_exit;
static unsigned rx_reads, rx_flushes, event_resets;
static const uint8_t *rx_input;
static unsigned rx_length;

int xSemaphoreTake(SemaphoreHandle_t semaphore, unsigned timeout)
{
    assert(lock_count == 0);
    ++lock_count;
    return pdTRUE;
}

int xSemaphoreGive(SemaphoreHandle_t semaphore)
{
    assert(lock_count == 1);
    --lock_count;
    return pdTRUE;
}

int xQueueSend(QueueHandle_t queue, const void *item, unsigned timeout)
{
    assert(timeout == 0);
    if (queue_count == 8) {
        return 0;
    }
    memcpy(&queued[queue_count++], item, sizeof(uart_packet_t));
    return pdTRUE;
}

void vTaskDelay(unsigned ticks)
{
    assert(lock_count == 0 && ticks > 0);
    assert(++delay_count < 20); /* Fail stalled retry loops rather than hanging. */
    assert(on_delay != NULL);
    on_delay();
}

uint16_t esp_ble_get_cur_sendable_packets_num(uint16_t conn_id)
{
    assert(conn_id == link.conn_id);
    return sendable;
}

esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn_id,
    uint16_t handle, uint16_t size, uint8_t *data, bool confirmation)
{
    assert(lock_count == 0 && !confirmation);
    assert(interface == link.gatts_if && conn_id == link.conn_id);
    assert(handle == handles[ATTR_DATA] && size <= link.mtu - 3);
    if (fail_sends) {
        --fail_sends;
        return ESP_FAIL;
    }
    assert(send_count < 64 && notification_length + size <= sizeof(notifications));
    notification_chunks[send_count++] = size;
    memcpy(notifications + notification_length, data, size);
    notification_length += size;
    if (after_send) {
        after_send();
    }
    return ESP_OK;
}

esp_err_t esp_ble_gatts_send_response(esp_gatt_if_t interface, uint16_t conn_id,
    uint32_t transaction, esp_gatt_status_t status, esp_gatt_rsp_t *response)
{
    ++responses;
    last_status = status;
    if (response) {
        last_response = *response;
    }
    return ESP_OK;
}

esp_err_t esp_ble_gap_ext_adv_start(uint8_t count, const esp_ble_gap_ext_adv_t *params)
{
    assert(count == 1 && params->instance == 0);
    ++advertising_starts;
    return ESP_OK;
}

esp_err_t esp_ble_gap_update_conn_params(esp_ble_conn_update_params_t *params)
{
    requested_connection = *params;
    return ESP_OK;
}

esp_err_t esp_ble_gatts_close(esp_gatt_if_t interface, uint16_t conn_id)
{
    ++closed_connections;
    return ESP_OK;
}

esp_err_t esp_ble_gap_set_preferred_phy(esp_bd_addr_t address,
    esp_ble_gap_all_phys_t all, esp_ble_gap_phy_mask_t tx,
    esp_ble_gap_phy_mask_t rx, esp_ble_gap_prefer_phy_options_t options)
{
    assert(tx == BLE_TRANSP_PHY && rx == BLE_TRANSP_PHY);
    return ESP_OK;
}

esp_err_t esp_ble_gap_set_device_name(const char *name)
{
    assert(strcmp(name, "bletransp") == 0);
    return ESP_OK;
}

esp_err_t esp_ble_gatts_create_attr_tab(const esp_gatts_attr_db_t *table,
    esp_gatt_if_t interface, uint16_t count, uint8_t instance)
{
    assert(table == attributes && count == ATTR_COUNT);
    return ESP_OK;
}

esp_err_t esp_ble_gatts_start_service(uint16_t handle) { return ESP_OK; }
esp_err_t esp_ble_gap_ext_adv_set_params(uint8_t instance,
    const esp_ble_gap_ext_adv_params_t *params) { return ESP_OK; }
esp_err_t esp_ble_gap_config_ext_adv_data_raw(uint8_t instance,
    uint16_t size, const uint8_t *data) { return ESP_OK; }

/* Initializers are compiled too, but tasks are driven explicitly by these tests. */
int xQueueReceive(QueueHandle_t queue, void *item, unsigned timeout) { return 0; }
int xQueueReset(QueueHandle_t queue) { ++event_resets; return pdTRUE; }
QueueHandle_t xQueueCreate(unsigned count, unsigned size) { return (void *)1; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)2; }
int xTaskCreate(void (*task)(void *), const char *name, unsigned stack,
    void *arg, unsigned priority, void *handle) { return pdPASS; }
int uart_write_bytes(int uart, const void *data, size_t size) { return (int)size; }
int uart_read_bytes(int uart, void *data, unsigned size, unsigned timeout)
{
    assert(lock_count == 0 && size >= rx_length && timeout > 0);
    if (rx_reads++) {
        longjmp(task_exit, 1);
    }
    memcpy(data, rx_input, rx_length);
    return (int)rx_length;
}
esp_err_t uart_flush_input(int uart) { ++rx_flushes; return ESP_OK; }
esp_err_t uart_param_config(int uart, const uart_config_t *config) { return ESP_OK; }
esp_err_t uart_set_pin(int uart, int tx, int rx, int rts, int cts) { return ESP_OK; }
esp_err_t uart_driver_install(int uart, int rx, int tx, int count,
    QueueHandle_t *events, int flags) { *events = (void *)3; return ESP_OK; }
esp_err_t esp_bt_controller_mem_release(int mode) { return ESP_OK; }
esp_err_t esp_bt_controller_init(const esp_bt_controller_config_t *config) { return ESP_OK; }
esp_err_t esp_bt_controller_enable(int mode) { return ESP_OK; }
esp_err_t esp_bluedroid_init_with_cfg(const esp_bluedroid_config_t *config) { return ESP_OK; }
esp_err_t esp_bluedroid_enable(void) { return ESP_OK; }
esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { return ESP_OK; }
esp_err_t esp_ble_gatts_register_callback(esp_gatts_cb_t callback) { return ESP_OK; }
esp_err_t esp_ble_gap_register_callback(esp_gap_ble_cb_t callback) { return ESP_OK; }
esp_err_t esp_ble_gatt_set_local_mtu(uint16_t mtu) { return ESP_OK; }
esp_err_t esp_ble_gap_set_preferred_default_phy(esp_ble_gap_phy_mask_t tx,
    esp_ble_gap_phy_mask_t rx) { return ESP_OK; }
esp_err_t esp_ble_gatts_app_register(uint16_t app_id) { return ESP_OK; }

static void reset_link(void)
{
    assert(lock_count == 0);
    memset(&link, 0, sizeof(link));
    link.gatts_if = 3;
    link.conn_id = 7;
    link.connected = true;
    link.mtu = 23;
    link.generation = 10;
    handles[ATTR_DATA] = 42;
    handles[ATTR_CCCD] = 43;
    queue_count = delay_count = send_count = notification_length = fail_sends = 0;
    responses = advertising_starts = closed_connections = 0;
    sendable = 1;
    on_delay = after_send = NULL;
    rx_reads = rx_flushes = event_resets = 0;
}

static esp_ble_gatts_cb_param_t write_request(uint16_t handle, uint8_t *data, uint16_t size)
{
    esp_ble_gatts_cb_param_t event = {0};
    event.write.conn_id = link.conn_id;
    event.write.handle = handle;
    event.write.len = size;
    event.write.value = data;
    event.write.need_rsp = true;
    return event;
}

static void test_binary_writes_and_backpressure(void)
{
    reset_link();
    uint8_t binary[] = {0x00, 0xff, 0x0a, 0x80, 0x00};
    esp_ble_gatts_cb_param_t event = write_request(handles[ATTR_DATA], binary, sizeof(binary));
    gatts_event_handler(ESP_GATTS_WRITE_EVT, link.gatts_if, &event);
    assert(last_status == ESP_GATT_OK && responses == 1 && queue_count == 1);
    assert(queued[0].len == sizeof(binary));
    assert(memcmp(queued[0].data, binary, sizeof(binary)) == 0);
    binary[0] = 99;
    assert(queued[0].data[0] == 0); /* Queue owns its copy before BLE callback returns. */
    for (unsigned i = 1; i < 8; ++i) {
        assert(handle_write(&event) == ESP_GATT_OK);
    }
    gatts_event_handler(ESP_GATTS_WRITE_EVT, link.gatts_if, &event);
    assert(last_status == ESP_GATT_NO_RESOURCES && queue_count == 8);
    event.write.need_rsp = false;
    gatts_event_handler(ESP_GATTS_WRITE_EVT, link.gatts_if, &event);
    assert(responses == 2 && queue_count == 8);
    event.write.len = 0;
    assert(handle_write(&event) == ESP_GATT_OK && queue_count == 8);
    event.write.len = PACKET_SIZE + 1;
    assert(handle_write(&event) == ESP_GATT_INVALID_ATTR_LEN);
    event.write.len = 1;
    event.write.value = NULL;
    assert(handle_write(&event) == ESP_GATT_INVALID_PDU);
    event.write.value = binary;
    event.write.offset = 1;
    assert(handle_write(&event) == ESP_GATT_INVALID_OFFSET);
    event.write.offset = 0;
    event.write.is_prep = true;
    assert(handle_write(&event) == ESP_GATT_REQ_NOT_SUPPORTED);
    event.write.is_prep = false;
    event.write.handle = 99;
    assert(handle_write(&event) == ESP_GATT_WRITE_NOT_PERMIT);
    puts("PASS binary writes, queue backpressure and input validation");
}

static void test_cccd(void)
{
    reset_link();
    uint8_t value[2] = {1, 0};
    esp_ble_gatts_cb_param_t event = write_request(handles[ATTR_CCCD], value, sizeof(value));
    assert(handle_write(&event) == ESP_GATT_OK && link.notify_enabled);
    assert(link.generation == 11 && queue_count == 0);
    assert(handle_write(&event) == ESP_GATT_OK && link.generation == 11);
    value[0] = 2;
    assert(handle_write(&event) == ESP_GATT_CCC_CFG_ERR && link.notify_enabled);
    value[0] = 1;
    value[1] = 1;
    assert(handle_write(&event) == ESP_GATT_CCC_CFG_ERR);
    event.write.len = 1;
    assert(handle_write(&event) == ESP_GATT_INVALID_ATTR_LEN);
    event.write.len = 2;
    value[0] = value[1] = 0;
    assert(handle_write(&event) == ESP_GATT_OK && !link.notify_enabled);
    assert(link.generation == 12);

    memset(&event, 0, sizeof(event));
    event.read.handle = handles[ATTR_CCCD];
    event.read.offset = 1;
    gatts_event_handler(ESP_GATTS_READ_EVT, link.gatts_if, &event);
    assert(last_status == ESP_GATT_OK && last_response.attr_value.len == 1);
    assert(last_response.attr_value.value[0] == 0);
    event.read.offset = 2;
    gatts_event_handler(ESP_GATTS_READ_EVT, link.gatts_if, &event);
    assert(last_status == ESP_GATT_OK && last_response.attr_value.len == 0);
    event.read.offset = 3;
    gatts_event_handler(ESP_GATTS_READ_EVT, link.gatts_if, &event);
    assert(last_status == ESP_GATT_INVALID_OFFSET);
    puts("PASS CCCD values, offsets and subscription generation");
}

static void unblock_notifications(void)
{
    link.congested = false;
    sendable = 1;
}

static void end_session(void)
{
    esp_ble_gatts_cb_param_t event = {0};
    event.disconnect.conn_id = link.conn_id;
    gatts_event_handler(ESP_GATTS_DISCONNECT_EVT, link.gatts_if, &event);
    event.connect.conn_id = 7;
    gatts_event_handler(ESP_GATTS_CONNECT_EVT, link.gatts_if, &event);
    uint8_t value[] = {1, 0};
    event = write_request(handles[ATTR_CCCD], value, sizeof(value));
    assert(handle_write(&event) == ESP_GATT_OK);
}

static void resubscribe(void)
{
    uint8_t value[] = {0, 0};
    esp_ble_gatts_cb_param_t event = write_request(handles[ATTR_CCCD], value, sizeof(value));
    assert(handle_write(&event) == ESP_GATT_OK);
    value[0] = 1;
    assert(handle_write(&event) == ESP_GATT_OK);
}

static void test_notifications(void)
{
    uint8_t binary[512];
    for (size_t i = 0; i < sizeof(binary); ++i) {
        binary[i] = (uint8_t)i;
    }
    const uint16_t mtus[] = {23, 247, 517};
    for (size_t m = 0; m < sizeof(mtus) / sizeof(mtus[0]); ++m) {
        reset_link();
        link.notify_enabled = true;
        link.mtu = mtus[m];
        notify_uart_data(binary, sizeof(binary), link.generation);
        size_t payload = mtus[m] - 3;
        if (payload > PACKET_SIZE) payload = PACKET_SIZE;
        assert(send_count == (sizeof(binary) + payload - 1) / payload);
        assert(notification_length == sizeof(binary));
        assert(memcmp(binary, notifications, sizeof(binary)) == 0);
        for (unsigned i = 0; i + 1 < send_count; ++i) {
            assert(notification_chunks[i] == payload);
        }
    }
    reset_link();
    notify_uart_data(binary, sizeof(binary), link.generation);
    assert(send_count == 0); /* No UART data before the client subscribes. */
    link.notify_enabled = true;
    link.congested = true;
    on_delay = unblock_notifications;
    notify_uart_data(binary, 35, link.generation);
    assert(delay_count == 1 && notification_length == 35);
    assert(memcmp(binary, notifications, 35) == 0);

    reset_link();
    link.notify_enabled = true;
    sendable = 0;
    on_delay = unblock_notifications;
    fail_sends = 1;
    notify_uart_data(binary, 35, link.generation);
    assert(delay_count == 2 && notification_length == 35);
    assert(memcmp(binary, notifications, 35) == 0);

    reset_link();
    link.notify_enabled = true;
    after_send = end_session;
    notify_uart_data(binary, 35, link.generation);
    assert(send_count == 1 && notification_length == 20);
    reset_link();
    link.notify_enabled = true;
    link.congested = true;
    on_delay = resubscribe;
    notify_uart_data(binary, 35, link.generation);
    assert(send_count == 0 && delay_count == 1);
    puts("PASS MTU fragmentation, congestion retry and stale-session cancellation");
}

static void test_uart_rx_poll(void)
{
    static const uint8_t binary[] = {0, 0xff, 0x80, 10, 0, 0x55};
    reset_link();
    link.notify_enabled = true;
    rx_input = binary;
    rx_length = sizeof(binary);
    if (setjmp(task_exit) == 0) {
        uart_rx_task(NULL);
    }
    assert(notification_length == sizeof(binary));
    assert(memcmp(notifications, binary, sizeof(binary)) == 0);
    assert(rx_flushes == 1 && event_resets == 1);

    reset_link();
    link.notify_enabled = true;
    after_send = end_session;
    if (setjmp(task_exit) == 0) {
        uart_rx_task(NULL);
    }
    assert(notification_length == sizeof(binary));
    assert(rx_flushes == 2 && event_resets == 2);
    puts("PASS UART ring polling without DATA events and RX flush at session boundaries");
}

static void test_connection_and_advertising(void)
{
    reset_link();
    link.connected = false;
    esp_ble_gatts_cb_param_t event = {0};
    event.connect.conn_id = 7;
    gatts_event_handler(ESP_GATTS_CONNECT_EVT, link.gatts_if, &event);
    assert(link.connected && !link.notify_enabled && link.mtu == 23);
    assert(requested_connection.min_int == 24 && requested_connection.max_int == 24);
    event.connect.conn_id = 8;
    gatts_event_handler(ESP_GATTS_CONNECT_EVT, link.gatts_if, &event);
    assert(link.conn_id == 7 && closed_connections == 1);
    event.disconnect.conn_id = 8;
    gatts_event_handler(ESP_GATTS_DISCONNECT_EVT, link.gatts_if, &event);
    assert(link.connected && advertising_starts == 0);
    event.disconnect.conn_id = 7;
    gatts_event_handler(ESP_GATTS_DISCONNECT_EVT, link.gatts_if, &event);
    assert(!link.connected && !link.notify_enabled && advertising_starts == 1);

    bool found_name = false, found_service = false, found_interval = false;
    for (size_t offset = 0; offset < sizeof(adv_data);) {
        size_t length = adv_data[offset];
        assert(length && offset + length + 1 <= sizeof(adv_data));
        uint8_t type = adv_data[offset + 1];
        if (type == ESP_BLE_AD_TYPE_NAME_CMPL) {
            assert(length == 10 && memcmp(adv_data + offset + 2, "bletransp", 9) == 0);
            found_name = true;
        } else if (type == ESP_BLE_AD_TYPE_16SRV_CMPL) {
            assert(length == 3 && adv_data[offset + 2] == 0xff && adv_data[offset + 3] == 0);
            found_service = true;
        } else if (type == 0x12) {
            assert(length == 5 && adv_data[offset + 2] == 24 && adv_data[offset + 4] == 24);
            found_interval = true;
        }
        offset += length + 1;
    }
    assert(found_name && found_service && found_interval && sizeof(adv_data) <= 31);
    assert(adv_params.type == ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND);
    puts("PASS connection defaults, extra-peer rejection, reconnect advertising and AD fields");
}

int main(void)
{
    test_binary_writes_and_backpressure();
    test_cccd();
    test_notifications();
    test_uart_rx_poll();
    test_connection_and_advertising();
    assert(lock_count == 0);
    puts("All bridge regression tests passed.");
    return 0;
}
