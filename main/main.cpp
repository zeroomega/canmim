#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "mcp2515.h"
#include "led_strip.h"
#include "ble.h"
#include "driver/rmt.h"

// SPI PIN Map
#define MCP_SCK   7
#define MCP_MOSI 10
#define MCP_MISO  8
#define MCP_CS    9
#define MCP_INT   GPIO_NUM_6

// CAN Controller PIN Map
#define CAN_TX    GPIO_NUM_3
#define CAN_RX    GPIO_NUM_2

// LED PIN Map
#define LED_PIN   GPIO_NUM_4

// ADC PIN Map
#define ADC_OILP ADC1_CHANNEL_0
#define ADC_AUX  ADC1_CHANNEL_1

// CAN ID for OIL Pressure
#define OIL_PRESSURE_CAN_ID 0x662

// Resistors value in kOhm for the voltage divider.
#define OIL_PRESSURE_R1 10
#define OIL_PRESSURE_R2 50

// Characteristics of the oil pressure sensor.
#define OIL_PRESSURE_VL 500
#define OIL_PRESSURE_VH 4500
#define OIL_PRESSURE_PMAX 150


#define ADC_FRAME_SIZE 4 * 100

// By default ESP32 timer is 80,000,000 Hz (80Mhz)
// With a divider of 800, the divider counter ticks
// every 0.00001 sec.
#define TIMER_DIVIDER TIMER_BASE_CLK / 100000

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16         0xABF0

/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16     0xABF1


// Global SPI Handle for MCP2515 Can Controller
spi_device_handle_t spi_mcp2515;
adc_cali_handle_t adc1_cali_handle = NULL;
adc_continuous_handle_t adc1_cont_handle = NULL;

QueueHandle_t canOutputQueue;
QueueHandle_t bleOutputQueue;

volatile bool shouldUpdateADC = false;

static uint8_t own_addr_type;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    shouldUpdateADC = true;
    return true;
}

uint32_t kPassthroughIDs[] = {
    0x40,   // RPM, PPS
    0x138,  // Steering Angle
    0x139,  // Brake pressure.
    0x345,  // OilTemp, ECT
};

ble_uuid16_t ble_srv_chr_uuid = {
    .u = {
        .type = BLE_UUID_TYPE_16,
    },
    .value = {BLE_SVC_SPP_CHR_UUID16}
};

ble_uuid16_t ble_srv_uuid = {
    .u = {
        .type = BLE_UUID_TYPE_16,
    },
    .value = {BLE_SVC_SPP_UUID16}
};

ble_uuid16_t fields_uuid[] = {
    ble_srv_uuid,
};
// static ble_uuid_t *ble_srv_uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16);

int connection_handle[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

static uint8_t gatt_svr_chr_val;
static uint16_t ble_svc_gatt_read_val_handle;
static uint8_t gatt_svr_dsc_val;
bool should_ble_notify;

/* Define new custom service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ble_srv_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Support SPP service */
                .uuid = &ble_srv_chr_uuid.u,
                .access_cb = ble_svc_gatt_handler,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                .min_key_size = 0,
                .val_handle = &ble_svc_gatt_read_val_handle,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services. */
    },
};

void incomingCANTask(void * pvParameter);

void outgoingCANTask(void * pvParameter);

void bleTask(void *pvParameter);

// Test if CAN message should be pass through.
bool IsPassthroughID(twai_message_t *message);

// Helper function to cause system reboot when fatal error happens.
void fatal_error();

// Helper function to initiate ADC. Will reboot if error happens.
void init_adc();

// Helper function to initiate SPI Master controller. Will reboot if error
// happens.
void init_spi();

// Helper function to initiate ESP32 CAN Controller.
void init_can();

void init_mcu();

void init_timer();

void update_oilp();

void set_led();

void timer_callback(void* arg);

uint8_t get_normalized_oil_pressure();

uint32_t get_normalized_aux();

uint32_t get_immediate_aux();

void update_adc(uint32_t *adc1_mv, uint32_t *adc2_mv);

uint8_t calc_oil_pressure(uint32_t adc_mv);

void send_adc_data_to_can(uint32_t adc1_mv, uint32_t adc2_mv);

void handling_incoming_message();
// Helper function to send received CAN message to the output CAN bus.
// Errors will be ignored.
void sendTWAIMessageToCan(MCP2515 &mcpCan, twai_message_t *twaiMessage);

uint32_t adcSampleCounter = 0;
uint32_t canSampleCounter = 0;
volatile uint32_t canFrameRatePerSec = 0;
int64_t canMessageCounter = 0;
int64_t canMessageBeginTime = 0;

extern "C" void app_main(void) {
    gpio_set_pull_mode(MCP_INT, GPIO_FLOATING);
    init_mcu();
    init_can();
    init_spi();
    printf("SPI Init\n");
    init_adc();
    printf("ADC initialized\n");
    set_led();
    printf("led set\n");

    // Init NVS, which is used by WiFi and BLE stack.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    canOutputQueue = xQueueCreate(50, sizeof(can_frame));
    bleOutputQueue = xQueueCreate(50, sizeof(can_frame));

    xTaskCreate(incomingCANTask, "incomingCANTask", 4096, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(outgoingCANTask, "outgoingCANTask", 4096, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(bleTask, "bleTask", 4096, NULL, tskIDLE_PRIORITY, NULL);
}

void init_mcu() {
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %" PRIi16 ", ", chip_info.revision);

    uint32_t size_flash_chip;
    esp_flash_get_size(NULL, &size_flash_chip);
    printf("%ldMB %s flash\n", size_flash_chip / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size());
}

bool IsPassthroughID(twai_message_t *message) {
    // for (int i = 0; i < sizeof(kPassthroughIDs)/sizeof(uint32_t); i++) {
    //     if (kPassthroughIDs[i] == message->identifier)
    //         return true;
    // }
    // return false;
    // For GT86, since we don't have a full list of CAN ID that datalogger listens to
    // let all CAN IDs pass through.
    return true;
}

void fatal_error() {
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

void init_adc() {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t ret;
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "adc cali failed, code %d", ret);
        fatal_error();
    }

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4 * 5000,
        .conv_frame_size = ADC_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc1_cont_handle));

    adc_digi_pattern_config_t adc_pattern[2];
    memset(&adc_pattern, 0, sizeof(adc_digi_pattern_config_t)*2);
    for (int i = 0; i < 2; i++) {
      adc_pattern[i].atten = ADC_ATTEN_DB_11;
      adc_pattern[i].channel = i;
      adc_pattern[i].unit = ADC_UNIT_1;
      adc_pattern[i].bit_width = ADC_BITWIDTH_12;
    }
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 2,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 2 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    
    ESP_ERROR_CHECK(adc_continuous_config(adc1_cont_handle, &dig_cfg));
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
        .on_pool_ovf = NULL
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc1_cont_handle, &cbs, NULL));
}

void init_spi() {
    spi_bus_config_t s_bus_config;
    memset(&s_bus_config, 0, sizeof(spi_bus_config_t));
    s_bus_config.mosi_io_num = MCP_MOSI;
    s_bus_config.miso_io_num = MCP_MISO;
    s_bus_config.sclk_io_num = MCP_SCK;
    s_bus_config.quadwp_io_num = -1;
    spi_device_interface_config_t s_dev_config;
    memset(&s_dev_config, 0, sizeof(spi_device_interface_config_t));
    s_dev_config.clock_speed_hz = 8*1000*1000; // 8Mhz SPI
    s_dev_config.mode = 0;
    s_dev_config.spics_io_num = MCP_CS;
    s_dev_config.queue_size = 1;
    esp_err_t ret_code = spi_bus_initialize(SPI2_HOST, &s_bus_config, SPI_DMA_CH_AUTO);
    if (ret_code != ESP_OK) {
        ESP_LOGE("SPI", "Failed to initialize SPI Bus, Error: %s", esp_err_to_name(ret_code));
        fatal_error();
    } else {
        ESP_LOGW("SPI", "SPI bus initialized");
    }

    ret_code = spi_bus_add_device(SPI2_HOST, &s_dev_config, &spi_mcp2515);
    if (ret_code != ESP_OK) {
        ESP_LOGE("SPI", "Failed to initialize SPI Device, Error: %s", esp_err_to_name(ret_code));
        fatal_error();
    } else {
        ESP_LOGW("SPI", "SPI device added");
    }
}

void init_can() {
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX /* TX */, CAN_RX /* RX */, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGW("ESPCAN", "Can bus driver installed");
    } else {
        ESP_LOGE("ESPCAN", "Failed to install driver");
        fatal_error();
    }
    if (twai_start() == ESP_OK) {
        ESP_LOGW("ESPCAN", "Can bus started");
    } else {
        ESP_LOGE("ESPCAN", "Failed to start");
        fatal_error();
    }
}

uint8_t calc_oil_pressure(uint32_t adc_mv) {
    if (adc_mv < OIL_PRESSURE_VL)
        adc_mv = 0;
    else
        adc_mv -= OIL_PRESSURE_VL;
    adc_mv = adc_mv * OIL_PRESSURE_PMAX / (OIL_PRESSURE_VH - OIL_PRESSURE_VL);
    return (uint8_t)adc_mv;
}

void update_adc(uint32_t *adc1_mv, uint32_t *adc2_mv) {
    static uint8_t result[ADC_FRAME_SIZE * 2] = {0};
    
    uint32_t adc_sum[2];
    adc_sum[0] = 0;
    adc_sum[1] = 0;
    uint32_t adc_read_count[2];
    adc_read_count[0] = 0;
    adc_read_count[1] = 0;

    uint32_t ret_num = 0;
    esp_err_t ret;
    ret = adc_continuous_read(adc1_cont_handle, result, ADC_FRAME_SIZE * 2, &ret_num, 0);
    if (ret != ESP_OK) {
        ESP_LOGI("ADC", "ADC read return non OK: %d\n", ret);
    }
    for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
        uint32_t chan_num = p->type2.channel;
        if (chan_num != 0 && chan_num != 1) {
            ESP_LOGE("ADC", "Unknown ADC channel number %" PRIi32, chan_num);
            fatal_error();
        }
        uint32_t data = p->type2.data;        
        adc_sum[chan_num] += data;
        adc_read_count[chan_num]++;
    }
    for (int i = 0; i < 2; i++) {
        if (adc_read_count[i] > 0)
            adc_sum[i] /= adc_read_count[i];
        int mV = 0;
        adc_cali_raw_to_voltage(adc1_cali_handle, adc_sum[i], &mV);
        if (mV >= 0)
            adc_sum[i] = mV;
        else
            adc_sum[i] = 0;
    }
    *adc1_mv = adc_sum[0] * (OIL_PRESSURE_R1+OIL_PRESSURE_R2) / OIL_PRESSURE_R1;
    *adc2_mv = adc_sum[1] * (OIL_PRESSURE_R1+OIL_PRESSURE_R2) / OIL_PRESSURE_R1;
    adcSampleCounter++;
    if (adcSampleCounter >= 20) {
        adcSampleCounter = 0;
        ESP_LOGD("ADC", "Sample, %" PRIi32 " mV %" PRIi32 " mV", *adc1_mv, *adc2_mv);
        ESP_LOGD("ADC", "ret_num is %" PRIi32 " adc1 count is %" PRIi32 " adc2 count is %" PRIi32, ret_num, adc_read_count[0], adc_read_count[1]);
    }
}

void timer_callback(void* arg) {
    shouldUpdateADC = true;
}

void send_adc_data_to_can(uint32_t adc1_mv, uint32_t adc2_mv) {
    uint8_t pressure = calc_oil_pressure(adc1_mv);
    uint32_t canFrameRate = canFrameRatePerSec;
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = OIL_PRESSURE_CAN_ID;
    frame.can_dlc = 8;
    frame.data[0] = pressure;
    frame.data[1] = adc1_mv & 0xFF;
    frame.data[2] = (adc1_mv >> 8) & 0xFF;
    frame.data[3] = adc2_mv & 0xFF;
    frame.data[4] = (adc2_mv >> 8) & 0xFF;
    frame.data[5] = canFrameRate & 0xFF;
    frame.data[6] = (canFrameRate >> 8) & 0xFF;
    frame.data[7] = (canFrameRate >> 16) & 0xFF;
    if (xQueueSend(canOutputQueue, &frame, 0) != pdTRUE)
        ESP_LOGE("ADC", "Output CAN Queue Full");
    if (should_ble_notify) {
        if (xQueueSend(bleOutputQueue, &frame, 0) != pdTRUE) {
            ESP_LOGE("ADC", "Output BLE Queue Full");
        }
    }
}

void handling_incoming_message() {
    twai_message_t message;
    can_frame frame;
    esp_err_t ret_code;
    ret_code = twai_receive(&message, 1);
    switch(ret_code) {
        case ESP_OK:            
            canMessageCounter++;
            if (!IsPassthroughID(&message))
                break;
            if (!(message.rtr)) {
                // sendTWAIMessageToCan(mcpCan, &message);
                frame.can_id = message.identifier;
                frame.can_dlc = message.data_length_code;
                memcpy(frame.data, message.data, 8);
                xQueueSend(canOutputQueue, &frame, 0);
                if (should_ble_notify) {
                    xQueueSend(bleOutputQueue, &frame, 0);
                }
            }
            break;
        case ESP_ERR_TIMEOUT:
            // Timeout is normal when there is no CAN communication.
            // Ignore it.
            break;
        default:
            ESP_LOGE("ESPCAN", "Failed when receiving due to %s", esp_err_to_name(ret_code));
    }
}

void set_led() {
    static led_strip_t *pStrip_a;
    pStrip_a = led_strip_init(RMT_CHANNEL_0, GPIO_NUM_4, 1);
    pStrip_a->clear(pStrip_a, 50);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    pStrip_a->set_pixel(pStrip_a, 0, 6, 10, 6);
    pStrip_a->refresh(pStrip_a, 100);
}

void incomingCANTask(void * pvParameter) {
    // Start ADC measurement.
    canMessageBeginTime = esp_timer_get_time();
    ESP_ERROR_CHECK(adc_continuous_start(adc1_cont_handle));

    for(;;) {
        uint32_t adc1_mv, adc2_mv;
        if (shouldUpdateADC) {
            shouldUpdateADC = false;
            update_adc(&adc1_mv, &adc2_mv);
            send_adc_data_to_can(adc1_mv, adc2_mv);
        }
        handling_incoming_message();
        if (canMessageCounter >= 3000) {
            int64_t currentTime = esp_timer_get_time();
            canMessageCounter *= 1000000;
            canFrameRatePerSec = canMessageCounter / (currentTime - canMessageBeginTime);
            canMessageBeginTime = currentTime;
            canMessageCounter = 0;
            ESP_LOGD("CAN", "CAN FRAME RATE %" PRIi32, canFrameRatePerSec);
        }
    }
}

void outgoingCANTask(void * pvParameter) {
    MCP2515 mcpCan(&spi_mcp2515);
    printf("Created MCP2515 object\n");
    mcpCan.reset();
    printf("Reset MCP2515 \n");
    mcpCan.setBitrate(CAN_500KBPS, MCP_8MHZ);
    printf("Set bitrate MCP2515 \n");
    mcpCan.setOneShotMode(true);
    printf("Set OneShot MCP2515 \n");
    mcpCan.setNormalMode();
    printf("Set Normal mode MCP2515 \n");
    can_frame frame;
    for(;;) {
        if (xQueueReceive(canOutputQueue, &frame, portMAX_DELAY) == pdTRUE) {
            MCP2515::ERROR err = mcpCan.sendMessage(&frame);
            if (err != MCP2515::ERROR_OK) {
                //ESP_LOGE("MCPCAN", "Failed sending to CAN %d", err);
            }
        }
    }
}

/**
 * Logs information about a connection to the console.
 */
void ble_print_conn_desc(struct ble_gap_conn_desc *desc) {
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

void ble_server_on_reset(int reason) {
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

int ble_server_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    int rc;
    switch(event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            ble_print_conn_desc(&desc);
            // Save connection handle
            connection_handle[0] = event->connect.conn_handle;
        }
        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
            ble_server_advertise();
        }
        return 0;
    
    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        ble_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");
        ble_server_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        ble_server_advertise();
        return 0;
    
    case BLE_GAP_EVENT_CONN_UPDATE:
        DFLT_LOG_INFO("connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        ble_print_conn_desc(&desc);
        DFLT_LOG_INFO("\n");
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        DFLT_LOG_INFO("encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        ble_print_conn_desc(&desc);
        DFLT_LOG_INFO("\n");
        return 0;
    
    case BLE_GAP_EVENT_SUBSCRIBE:
        DFLT_LOG_INFO("subscribe event; conn_handle=%d attr_handle=%d "
                          "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        if (event->subscribe.cur_notify) {
            should_ble_notify = true;
        } else {
            should_ble_notify = false;
        }
        
        return 0;

    case BLE_GAP_EVENT_MTU:
        DFLT_LOG_INFO("mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;
    }

    return 0;
}

void ble_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    if(ble_gap_adv_active())
        return;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = fields_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_server_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
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

// void ble_store_config_init(void);

void ble_server_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    //print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
    ble_server_advertise();
}

int gatt_svr_init(void)
{
    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void ble_server_host_task(void *param)
{
    ESP_LOGI("MCU", "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* Callback function for custom service */
int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const ble_uuid_t *uuid;
    int rc;
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == ble_svc_gatt_read_val_handle) {
            rc = os_mbuf_append(ctxt->om,
                                &gatt_svr_chr_val,
                                sizeof(gatt_svr_chr_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        break;


    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic write by NimBLE stack; attr_handle=%d",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == ble_svc_gatt_read_val_handle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(gatt_svr_chr_val),
                                sizeof(gatt_svr_chr_val),
                                &gatt_svr_chr_val, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        break;
    
    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Descriptor read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Descriptor read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->dsc->uuid;
        //if (ble_uuid_cmp(uuid, ble_srv_uuid) == 0) {
            rc = os_mbuf_append(ctxt->om,
                                &gatt_svr_dsc_val,
                                sizeof(gatt_svr_dsc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        //}
        break;
    default:
        ESP_LOGI("BLE", "\nDefault Callback");
        break;
    }
    return 0;
}

int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len) {
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

void bleTask(void *pvParameter) {
    int rc;
    nimble_port_init();

    /* Initialize connection_handle array */
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        connection_handle[i] = -1;
    }

    ble_hs_cfg.reset_cb = ble_server_on_reset;
    ble_hs_cfg.sync_cb = ble_server_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = 3;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
#endif
#ifdef CONFIG_EXAMPLE_MITM
    ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
    ble_hs_cfg.sm_sc = 1;
#else
    ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
#endif
    /* Register custom service */
    rc = gatt_svr_init();
    assert(rc == 0);
    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-ble-svr");
    assert(rc == 0);
    gatt_svr_dsc_val = 0x99;

    /* XXX Need to have template for store */
    //ble_store_config_init();

    nimble_port_freertos_init(ble_server_host_task);

    can_frame frame;
    for (;;) {
        if (xQueueReceive(bleOutputQueue, &frame, portMAX_DELAY)) {
            if (should_ble_notify) {
                struct os_mbuf * txom;
                txom = ble_hs_mbuf_from_flat(&frame, sizeof(frame));
                if (txom == NULL) {
                    ESP_LOGW("BLE", "txom is NULL");
                    continue;
                }
                rc = ble_gatts_notify_custom(connection_handle[0],
                                            ble_svc_gatt_read_val_handle,
                                            txom);
                if (rc != 0) {
                    ESP_LOGW("BLE", "Error in sending notification rc = %d", rc);
                }
                //os_mbuf_free_chain(txom);
            }
        }
    }
}