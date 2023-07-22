#include "sdkconfig.h"
#include "board_config.h"
#include "esp_mac.h"
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
#include "driver/rmt.h"

#include <stdio.h>
#include <string.h>

#include "canmim.h"
#include "mcp2515.h"
#include "led_strip.h"
#include "ble.h"


#define LOG_CAN_TAG "CAN"
#define LOG_MCU_TAG "MCU"
#define LOG_CAN_SPI_TAG "SPI CAN"
#define LOG_BLE_TAG "BLE"

#define ADC_FRAME_SIZE 4 * 100

// Global SPI Handle for MCP2515 Can Controller
spi_device_handle_t spi_mcp2515;
adc_cali_handle_t adc1_cali_handle = NULL;
adc_continuous_handle_t adc1_cont_handle = NULL;

QueueHandle_t canOutputQueue;
QueueHandle_t bleOutputQueue;

volatile bool shouldUpdateADC = false;


#ifdef CANMIM_PRINT_METRICS

volatile size_t CAN_IN_COUNT = 0;
volatile size_t CAN_OUT_COUNT = 0;
volatile size_t BLE_OUT_COUNT = 0;
volatile size_t BLE_NOTIFIED_COUNT = 0;
volatile int64_t MCP_TIMER = 0;
volatile uint32_t ADC1_MV = 0;
volatile uint32_t ADC2_MV = 0;

#endif

bool can_isolation = false;

void gpio_init();

void incomingCANTask(void * pvParameter);

void outgoingCANTask(void * pvParameter);

void bleTask(void *pvParameter);

void set_ble_name_from_mac();

// Helper function to cause system reboot when fatal error happens.
void fatal_error();

// Helper function to initiate ADC. Will reboot if error happens.
void init_adc();

// Helper function to initiate SPI Master controller. Will reboot if error
// happens.
void init_spi();

// Helper function to initiate ESP32 CAN Controller.
void init_can();

void print_mcu_info();

void set_led();

void update_adc(uint32_t *adc1_mv, uint32_t *adc2_mv);

uint8_t calc_oil_pressure(uint32_t adc_mv);

void send_adc_data_to_can(uint32_t adc1_mv, uint32_t adc2_mv);

void handling_incoming_message();

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    shouldUpdateADC = true;
    return true;
}

#ifdef CANMIM_PRINT_METRICS

static void tick_timer_cb(void *arg) {
    size_t can_in_count = CAN_IN_COUNT;
    size_t can_out_count = CAN_OUT_COUNT;
    size_t ble_out_count = BLE_OUT_COUNT;
    size_t ble_notified_count = BLE_NOTIFIED_COUNT;
    int64_t mcp_timer  = MCP_TIMER;
    ESP_LOGI("TIMER", "can_in_count: %zu", can_in_count);
    ESP_LOGI("TIMER", "can_out_count: %zu", can_out_count);
    ESP_LOGI("TIMER", "ble_out_count: %zu", ble_out_count);
    ESP_LOGI("TIMER", "ble_notified_count: %zu", ble_notified_count);
    ESP_LOGI("TIMER", "MCP_TIMER: %zu microsec", (size_t)mcp_timer);
    ESP_LOGI("TIMER", "Sample, %" PRIi32 " mV %" PRIi32 " mV", ADC1_MV, ADC2_MV);
    ESP_LOGI("TIMER", "");
    CAN_IN_COUNT = 0;
    CAN_OUT_COUNT = 0;
    BLE_OUT_COUNT = 0;
    BLE_NOTIFIED_COUNT = 0;
    MCP_TIMER = 0;
}

#endif

extern "C" void app_main(void) {
    gpio_init();
    print_mcu_info();
    if (can_isolation)
        ESP_LOGI(LOG_CAN_TAG, "CAN isolation: enabled");
    else
        ESP_LOGI(LOG_CAN_TAG, "CAN isolation: disabled");
    init_can();
    if (can_isolation) {
        init_spi();
        ESP_LOGI(LOG_MCU_TAG, "SPI Init");
        printf("SPI Init\n");
    }
    init_adc();
    ESP_LOGI(LOG_MCU_TAG, "ADC Init");
    set_led();
    ESP_LOGI(LOG_MCU_TAG, "LED Init");

    // Init NVS, which is used by WiFi and BLE stack.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    set_ble_name_from_mac();
#ifdef CANMIM_PRINT_METRICS
    const esp_timer_create_args_t tick_timer_args = {
        .callback = &tick_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tick_timer",
        .skip_unhandled_events = false,
    };

    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000000));
#endif

    canOutputQueue = xQueueCreate(50, sizeof(can_frame_t));
    bleOutputQueue = xQueueCreate(50, sizeof(can_frame_t));

    xTaskCreate(incomingCANTask, "incomingCANTask", 4096, NULL, 1, NULL);
    xTaskCreate(outgoingCANTask, "outgoingCANTask", 4096, NULL, 3, NULL);
    xTaskCreate(bleTask, "bleTask", 4096, NULL, 2, NULL);
}

void set_ble_name_from_mac() {
    uint8_t ble_mac[8];
    memset(ble_mac, 0, sizeof(ble_mac));
    esp_err_t ret = esp_read_mac(ble_mac, ESP_MAC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_MCU_TAG, "Failed to retrieve BLE MAC address: %d", ret);
    }
    char ble_name[32];
    memset(ble_name, 0, sizeof(ble_name));
    size_t ble_name_len = strlen(BLE_DEVICE_NAME_BASE);
    if (ble_name_len > sizeof(ble_name)-4)
        ble_name_len = sizeof(ble_name)-4;
    memcpy(ble_name, BLE_DEVICE_NAME_BASE, ble_name_len);
    int ble_num = ble_mac[4] + ble_mac[5];
    ble_num = ble_num * 2621 %100;
    int char_count = sprintf(ble_name + ble_name_len, "%d", ble_num);
    ble_name_len += char_count;
    ble_name[ble_name_len] = 0;
    set_ble_name(ble_name, ble_name_len);
    ESP_LOGI(LOG_BLE_TAG, "BLE Device name: %s", ble_name);
}

void gpio_init() {
    // Set MCP2525 interrupt pin to float as we don't use it
    // for now.
    if (MCP_INT != -1)
        gpio_set_pull_mode((gpio_num_t)MCP_INT, GPIO_FLOATING);
    #ifndef ISO_OVERRIDE
    if (ISO_PIN != -1) {
        esp_err_t err = gpio_set_direction((gpio_num_t)ISO_PIN, GPIO_MODE_INPUT_OUTPUT);
        if (err != ESP_OK) {
            ESP_LOGE("GPIO", "ISO Pin failed to set to I/O mode, error code: %d", err);
        }
        err = gpio_set_pull_mode((gpio_num_t)ISO_PIN, GPIO_PULLUP_ONLY);
        if (err != ESP_OK) {
            ESP_LOGE("GPIO", "ISO Pin failed to set to Pull Up, error code: %d", err);
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
        int level = gpio_get_level((gpio_num_t)ISO_PIN);
        if (level)
            can_isolation = true;
        else
            can_isolation = false;
    }
    #else
    can_isolation = ISO_OVERRIDE;
    #endif
}

void print_mcu_info() {
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
    s_dev_config.queue_size = 10;
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
    twai_mode_t mode = TWAI_MODE_LISTEN_ONLY;
    if (!can_isolation)
        mode = TWAI_MODE_NO_ACK;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX /* TX */, (gpio_num_t)CAN_RX /* RX */, mode);
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

#ifdef CANMIM_PRINT_METRICS
    ADC1_MV = *adc1_mv;
    ADC2_MV = *adc2_mv;
#endif
}


void send_adc_data_to_can(uint32_t adc1_mv, uint32_t adc2_mv) {
    uint8_t pressure = calc_oil_pressure(adc1_mv);
    can_frame_t frame;
    memset(&frame, 0, sizeof(can_frame_t));
    frame.timestamp = esp_timer_get_time() / 1000;
    frame.can_id = OIL_PRESSURE_CAN_ID;
    frame.can_dlc = 8;
    frame.data[0] = pressure;
    frame.data[1] = adc1_mv & 0xFF;
    frame.data[2] = (adc1_mv >> 8) & 0xFF;
    frame.data[3] = adc2_mv & 0xFF;
    frame.data[4] = (adc2_mv >> 8) & 0xFF;
    if (xQueueSend(canOutputQueue, &frame, 0) != pdTRUE) {
        ESP_LOGE("ADC", "Output CAN Queue Full");
    }
    
    if (xQueueSend(bleOutputQueue, &frame, 0) != pdTRUE) {
        ESP_LOGE("ADC", "Output BLE Queue Full");
    }
}

void handling_incoming_message() {
    twai_message_t message;
    can_frame_t frame;
    esp_err_t ret_code;
    ret_code = twai_receive(&message, 1);
    switch(ret_code) {
        case ESP_OK:
            if (!(message.rtr)) {
                frame.timestamp = esp_timer_get_time()/1000;
                CAN_IN_COUNT++;
                frame.can_id = message.identifier;
                frame.can_dlc = message.data_length_code;
                memcpy(frame.data, message.data, 8);
                if (can_isolation)
                    xQueueSend(canOutputQueue, &frame, 0);
                xQueueSend(bleOutputQueue, &frame, 0);
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
    ESP_ERROR_CHECK(adc_continuous_start(adc1_cont_handle));

    for(;;) {
        uint32_t adc1_mv, adc2_mv;
        if (shouldUpdateADC) {
            shouldUpdateADC = false;
            update_adc(&adc1_mv, &adc2_mv);
            send_adc_data_to_can(adc1_mv, adc2_mv);
        }
        handling_incoming_message();
    }
}

void outgoingCANTask(void * pvParameter) {
    MCP2515* mcpCan = nullptr;
    if (can_isolation) {
        mcpCan = new MCP2515(&spi_mcp2515);
        printf("Created MCP2515 object\n");
        mcpCan->reset();
        printf("Reset MCP2515 \n");
        mcpCan->setBitrate(CAN_500KBPS, MCP_8MHZ);
        printf("Set bitrate MCP2515 \n");
        mcpCan->setNormalMode();
        printf("Set Normal mode MCP2515 \n");
        MCP2515::ERROR err = mcpCan->setOneShotMode(true);
        if (err != MCP2515::ERROR_OK) {
            ESP_LOGE("MCP2515", "Failed to set One Shot Mode");
        } else {
            ESP_LOGI("MCP2515", "Set One Shot Mode");
        }
    }
    can_frame_t frame;
    for(;;) {
        if (xQueueReceive(canOutputQueue, &frame, portMAX_DELAY) == pdTRUE) {
            // Unlikely to happen. If Output CAN is congested, drop a few CAN
            // frames to avoid buffer overfill.
            if (uxQueueMessagesWaiting(canOutputQueue) > 25) {
                for (int i = 0; i < 10; i++) {
                    xQueueReceive(bleOutputQueue, &frame, portMAX_DELAY);
                }
                ESP_LOGI("MCPCAN", "Skipped 10 messages");
            }

            if (can_isolation) {
                CAN_OUT_COUNT++;
                can_frame mcp_frame;
                mcp_frame.can_id = frame.can_id;
                mcp_frame.can_dlc = frame.can_dlc;
                memcpy(mcp_frame.data, frame.data, CAN_MAX_DATALEN);
                int64_t begin = esp_timer_get_time();
                MCP2515::ERROR err = mcpCan->sendMessageSkipStatus(&mcp_frame);
                int64_t end = esp_timer_get_time();
                MCP_TIMER = (end - begin);
                if (err != MCP2515::ERROR_OK) {
                    ESP_LOGE("MCPCAN", "Failed sending to CAN %d", err);
                    if (err == MCP2515::ERROR_ALLTXBUSY) {
                        ESP_LOGE("MCPCAN", "Failed sending to CAN, All TX busy");
                    }
                }
            } else {
                twai_message_t message;
                memset(&message, 0, sizeof(message));
                message.identifier = frame.can_id;
                message.data_length_code = frame.can_dlc;
                memcpy(message.data, frame.data, 8);
                esp_err_t ret_code = twai_transmit(&message, 1);
                if (ret_code != ESP_OK)
                     ESP_LOGE("ESPCAN", "Failed when transmitting due to %s", esp_err_to_name(ret_code));
            }
        }
    }
}

void bleTask(void *pvParameter) {
    ble_init();
    can_frame_t frame;
    while(1) {
        if (xQueueReceive(bleOutputQueue, &frame, portMAX_DELAY) == pdTRUE) {
            if (uxQueueMessagesWaiting(bleOutputQueue) > 25) {
                for (int i = 0; i < 10; i++) {
                    xQueueReceive(bleOutputQueue, &frame, portMAX_DELAY);
                }
                ESP_LOGI("BLE", "10 message skipped.");
            }
            BLE_OUT_COUNT++;
            if (ble_notify(&frame)) {
                BLE_NOTIFIED_COUNT++;
            }
        }        
    }
}








