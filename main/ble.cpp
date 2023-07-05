#include "ble.h"
#include "freertos/semphr.h"

#define GATTS_TAG "BLE"

///Declare the static function
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_CAN      0x1FF8

#define GATTS_CHAR_UUID_CAN_MAIN    0x0001
#define GATTS_CHAR_UUID_CAN_FILTER  0x0002
#define GATTS_NUM_HANDLE_TEST_A     6

#define BLE_DEVICE_NAME_STR            "ESP_CAN_BLE"

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t rc_can_main_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t rc_can_main_property = 0;

static esp_attr_value_t gatts_rc_can_char_main_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(rc_can_main_str),
    .attr_value   = rc_can_main_str,
};

static uint8_t rc_can_filter_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t rc_can_filter_property = 0;

static esp_attr_value_t gatts_rc_can_char_filter_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(rc_can_filter_str),
    .attr_value   = rc_can_filter_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr          = {0, 0, 0, 0, 0, 0},
    .peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


struct rc_can_gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;    
    uint16_t char_filter_handle;
    esp_bt_uuid_t char_filter_uuid;
    esp_gatt_perm_t perm_filter;
    esp_gatt_char_prop_t filter_property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct rc_can_gatts_profile_inst rc_can_profile = {
    .gatts_cb = gatts_profile_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    .app_id = 0,
    .conn_id = 0,
    .service_handle = 0,
    .service_id = {.id = {.uuid = {.len = 0, .uuid = 0}, .inst_id = 0,}, .is_primary = false},
    .char_handle = 0,
    .char_uuid = {.len = 0, .uuid = 0},
    .perm = 0,
    .property = 0,
    .char_filter_handle = 0,
    .char_filter_uuid = {.len = 0, .uuid = 0},
    .perm_filter = 0,
    .filter_property = 0,
    .descr_handle = 0,
    .descr_uuid = {.len = 0, .uuid = 0},
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

uint8_t can_frame[32];
size_t can_frame_len;
SemaphoreHandle_t can_data_lock;

volatile bool should_notify = false;
volatile uint16_t notify_char_handle;
volatile uint16_t notify_conn_id;
volatile esp_gatt_if_t notify_gatt_if;

#define CAN_ALLOW_LIST_MAX_LENGTH 256
bool can_allow_list_enabled = false;
uint32_t can_allow_list[CAN_ALLOW_LIST_MAX_LENGTH];
size_t can_allow_list_index;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex("BLEWRITE_EXEC", prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        rc_can_profile.service_id.is_primary = true;
        rc_can_profile.service_id.id.inst_id = 0x00;
        rc_can_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        rc_can_profile.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_CAN;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(BLE_DEVICE_NAME_STR);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

        esp_ble_gatts_create_service(gatts_if, &rc_can_profile.service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    }
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));        
        rsp.attr_value.handle = param->read.handle;
        if (xSemaphoreTake(can_data_lock, 1) == pdTRUE) {
            rsp.attr_value.len = can_frame_len;
            memcpy(rsp.attr_value.value, can_frame, can_frame_len);
            xSemaphoreGive(can_data_lock);
        }
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex("BLEWRITE", param->write.value, param->write.len);
            if (rc_can_profile.descr_handle == param->write.handle && param->write.len == 2){
                // Write event to CCCD
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (rc_can_main_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        notify_char_handle = rc_can_profile.char_handle;
                        notify_conn_id = param->write.conn_id;
                        notify_gatt_if = gatts_if;
                        should_notify = true;
                    }
                }else if (descr_value == 0x0002){
                    if (rc_can_main_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        notify_char_handle = rc_can_profile.char_handle;
                        notify_conn_id = param->write.conn_id;
                        notify_gatt_if = gatts_if;
                        should_notify = true;
                    }
                }
                else if (descr_value == 0x0000){
                    should_notify = false;
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex("BLEWRITE", param->write.value, param->write.len);
                }
            }
            if (rc_can_profile.char_filter_handle == param->write.handle && param->write.len > 0) {
                // Write event to filter command.
                uint8_t cmd = param->write.value[0];;
                switch(cmd) {
                case 0:{ //deny all
                    can_allow_list_enabled = true;
                    ESP_LOGI(GATTS_TAG, "CAN Filter, Deny All");
                    break;
                }
                case 1:{ // allow all
                    can_allow_list_enabled = false;
                    can_allow_list_index = 0;
                    memset(can_allow_list, 0, sizeof(uint32_t)*CAN_ALLOW_LIST_MAX_LENGTH);
                    // ignore update interval advice.
                    ESP_LOGI(GATTS_TAG, "CAN Filter, Allow All");
                    break;
                }
                case 2:{ // allow one CAN ID
                    if (!can_allow_list_enabled)
                        break;
                    if (param->write.len < 7)
                        break;
                    // CAN_ID from buffer is big endian
                    uint8_t *buffer = param->write.value;
                    uint32_t can_id = buffer[3] << 24 | buffer[4] << 16 | buffer[5] << 8 | buffer[6];
                    // ignore update interval advice
                    can_allow_list[can_allow_list_index] = can_id;
                    can_allow_list_index++;
                    ESP_LOGI(GATTS_TAG, "CAN Filter, Allow %x", (unsigned int)can_id);
                    break;
                }
                }
            }
        }
        // Process long message and respond ACK.
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:{
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        rc_can_profile.service_handle = param->create.service_handle;
        rc_can_profile.char_uuid.len = ESP_UUID_LEN_16;
        rc_can_profile.char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_CAN_MAIN;
        rc_can_profile.char_filter_uuid.len = ESP_UUID_LEN_16;
        rc_can_profile.char_filter_uuid.uuid.uuid16 = GATTS_CHAR_UUID_CAN_FILTER;

        esp_ble_gatts_start_service(rc_can_profile.service_handle);
        rc_can_main_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(rc_can_profile.service_handle, &rc_can_profile.char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        rc_can_main_property,
                                                        &gatts_rc_can_char_main_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }

        rc_can_profile.descr_uuid.len = ESP_UUID_LEN_16;
        rc_can_profile.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(rc_can_profile.service_handle, &rc_can_profile.descr_uuid,
                                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        ESP_LOGI(GATTS_TAG, "esp_ble_gatts_add_char_descr called");
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }

        rc_can_filter_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
        add_char_ret = esp_ble_gatts_add_char(rc_can_profile.service_handle, &rc_can_profile.char_filter_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                rc_can_filter_property,
                                                &gatts_rc_can_char_filter_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        if (param->add_char.char_uuid.uuid.uuid16 == rc_can_profile.char_uuid.uuid.uuid16)
            rc_can_profile.char_handle = param->add_char.attr_handle;
        else if (param->add_char.char_uuid.uuid.uuid16 == rc_can_profile.char_filter_uuid.uuid.uuid16)
            rc_can_profile.char_filter_handle =  param->add_char.attr_handle;

        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x",i,prf_char[i]);
        }

        
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        rc_can_profile.descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params;
        memset(&conn_params, 0, sizeof(esp_ble_conn_update_params_t));
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 10;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x0A;    // min_int = 0x0A*1.25ms = 12.5ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        rc_can_profile.conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        // Advertising will stop as a client is connected.
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        if (should_notify && param->connect.conn_id == notify_conn_id) {
            should_notify = false;
        }
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        //ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
            esp_log_buffer_hex("BLECONFIRM", param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
        break;
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

void print_addr(uint8_t addr[]) {
    for (int i = 0; i < 6; i++) {
        printf("%x", addr[i]);
        if (i != 5)
            printf(":");
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            rc_can_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
        gatts_if == rc_can_profile.gatts_if) {
        if (rc_can_profile.gatts_cb) {
            rc_can_profile.gatts_cb(event, gatts_if, param);
        }
    }
}


void ble_init() {
    esp_err_t ret;
    can_allow_list_enabled = false;
    memset(can_allow_list, 0, sizeof(uint32_t)*256);
    can_allow_list_index = 0;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed", __func__);
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(0);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }
    can_data_lock = xSemaphoreCreateMutex();
    return;
}

bool ble_notify(void * data, size_t len) {
    if (len > 32)
        len = 32;
    if (xSemaphoreTake(can_data_lock, 10) == pdTRUE) {
        can_frame_len = len;
        memcpy(can_frame, data, len);
        xSemaphoreGive(can_data_lock);
    }
    if (should_notify) {
        bool can_id_allowed = !can_allow_list_enabled;
        if (can_allow_list_enabled) {
            uint32_t can_id = *(uint32_t*)data;
            for (int i = 0; i < can_allow_list_index; i++) {
                if (can_allow_list[i] == can_id) {
                    can_id_allowed = true;
                    break;
                }
            }
        }
        if (can_id_allowed) {
            esp_ble_gatts_send_indicate(notify_gatt_if, notify_conn_id, notify_char_handle,
                                                len, (uint8_t*)data, false);
            return true;
        }
    }
    return false;
}