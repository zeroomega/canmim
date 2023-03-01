#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "mcp2515.h"
#include "led_strip.h"
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


// Global SPI Handle for MCP2515 Can Controller
spi_device_handle_t spi_mcp2515;
adc_cali_handle_t adc1_cali_handle = NULL;
adc_continuous_handle_t adc1_cont_handle = NULL;

volatile bool shouldUpdateADC = false;

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

void send_adc_data_to_can(MCP2515& mcpCan, uint32_t adc1_mv, uint32_t adc2_mv);

void handling_incoming_message(MCP2515&mcpCan);
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
    init_adc();
    printf("ADC initialized\n");
    set_led();
    printf("led set\n");
    
    // Start ADC measurement.
    canMessageBeginTime = esp_timer_get_time();
    ESP_ERROR_CHECK(adc_continuous_start(adc1_cont_handle));
    while(1) {
        uint32_t adc1_mv, adc2_mv;
        if (shouldUpdateADC) {
            shouldUpdateADC = false;
            update_adc(&adc1_mv, &adc2_mv);
            send_adc_data_to_can(mcpCan, adc1_mv, adc2_mv);
        }
        handling_incoming_message(mcpCan);
        //canMessageCounter++;
        if (canMessageCounter >= 3000) {
            int64_t currentTime = esp_timer_get_time();
            canMessageCounter *= 1000000;
            canFrameRatePerSec = canMessageCounter / (currentTime - canMessageBeginTime);
            canMessageBeginTime = currentTime;
            canMessageCounter = 0;
            ESP_LOGD("CAN", "CAN FRAME RATE %"PRIi32, canFrameRatePerSec);
        }
    }
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

void sendTWAIMessageToCan(MCP2515 &mcpCan, twai_message_t *twaiMessage) {
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = twaiMessage->identifier;
    frame.can_dlc = twaiMessage->data_length_code;
    memcpy(frame.data, twaiMessage->data, 8);
    MCP2515::ERROR err = mcpCan.sendMessage(&frame);
    if (err != MCP2515::ERROR_OK)
        ESP_LOGE("MCPCAN", "Failed sending to CAN %d", err);
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

void send_adc_data_to_can(MCP2515& mcpCan, uint32_t adc1_mv, uint32_t adc2_mv) {
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
    MCP2515::ERROR err = mcpCan.sendMessage(&frame);
    if (err != MCP2515::ERROR_OK)
        ESP_LOGE("MCPCAN", "Failed sending ADC data to CAN %d", err);
}

void handling_incoming_message(MCP2515&mcpCan) {
    twai_message_t message;
    esp_err_t ret_code;
    ret_code = twai_receive(&message, 1);
    switch(ret_code) {
        case ESP_OK:
            canSampleCounter++;
            canMessageCounter++;
            if (canSampleCounter >= 5000) {
                canSampleCounter = 0;
                ESP_LOGD("ESPCAN", "Sampling Message ID: %" PRIi32 "", message.identifier);
            }
            if (!IsPassthroughID(&message))
                break;
            if (!(message.rtr))
                sendTWAIMessageToCan(mcpCan, &message);
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
