#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"
#include "mcp2515.h"

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

// By default ESP32 timer is 80,000,000 Hz (80Mhz)
// With a divider of 800, the divider counter ticks
// every 0.00001 sec.
#define TIMER_DIVIDER TIMER_BASE_CLK / 100000


// Global SPI Handle for MCP2515 Can Controller
spi_device_handle_t spi_mcp2515;
esp_adc_cal_characteristics_t adc1_chars;

volatile bool shouldUpdateADC = false;

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

bool IRAM_ATTR timer_callback(void* arg);

uint8_t get_normalized_oil_pressure();

uint32_t get_normalized_aux();

uint32_t get_immediate_aux();

void send_adc_data_to_can(MCP2515 &mcpCan, uint8_t pressure, uint32_t aux_data);

void handling_incoming_message(MCP2515&mcpCan);
// Helper function to send received CAN message to the output CAN bus.
// Errors will be ignored.
void sendTWAIMessageToCan(MCP2515 &mcpCan, twai_message_t *twaiMessage);

uint32_t adc1_level1[10];
uint32_t adc1_level1_pointer = 0;
uint32_t adc1_level1_rolling_sum = 0;
uint32_t adc1_level2[3];
uint32_t adc1_level2_pointer = 0;
uint32_t adc1_level2_rolling_sum = 0;
uint32_t adc1_read_counter = 0;
bool shouldSendOilPressure = false;

uint32_t adc2_level1[10];
uint32_t adc2_level1_pointer = 0;
uint32_t adc2_level1_rolling_sum = 0;
uint32_t adc2_level2[3];
uint32_t adc2_level2_pointer = 0;
uint32_t adc2_level2_rolling_sum = 0;
uint32_t adc2_read_counter = 0;
bool shouldSendADC2 = false;

uint32_t sampleCounter1 = 0;
uint32_t sampleCounter2 = 0;

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
    init_timer();
    printf("Timer initialized\n");
    
    while(1) {
        if (shouldUpdateADC) {
            shouldUpdateADC = false;
            update_oilp();
        }
        if (adc1_read_counter >= 5) {
            adc1_read_counter = 0;
            adc2_read_counter = 0;
            uint8_t pressure = get_normalized_oil_pressure();
            uint32_t aux_data = get_immediate_aux();
            send_adc_data_to_can(mcpCan, pressure, aux_data);
        }
        handling_incoming_message(mcpCan);
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

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    memset(&adc1_level1, 0, sizeof(adc1_level1));
    memset(&adc1_level2, 0, sizeof(adc1_level2));
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
    esp_err_t ret;
    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "Tow Point Calibration Data not available in FUSE, code %d", ret);
        fatal_error();
    }
    esp_adc_cal_value_t cal_ret = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 0, &adc1_chars);
    if (cal_ret == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGW("ADC", "Calibration Data TP");
    } else {
        ESP_LOGW("ADC", "Calibration Data Code: %d", cal_ret);
    }
    ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "ADC1 WIDTH Init failed %s", esp_err_to_name(ret));
        fatal_error();
    }
    ret = adc1_config_channel_atten(ADC_OILP, ADC_ATTEN_11db);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "OILP ATTEN Init failed %s", esp_err_to_name(ret));
        fatal_error();
    }
    ret = adc1_config_channel_atten(ADC_AUX, ADC_ATTEN_11db);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "AUX ATTEN Init failed %s", esp_err_to_name(ret));
        fatal_error();
    }
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

void init_timer() {
    timer_config_t timer_config;
    memset(&timer_config, 0, sizeof(timer_config_t));
    timer_config.alarm_en = TIMER_ALARM_EN;
    timer_config.auto_reload = TIMER_AUTORELOAD_EN;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.divider = TIMER_DIVIDER;
    esp_err_t ret_code = timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    if (ret_code != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to initialize timer, error: %s", esp_err_to_name(ret_code));
        fatal_error();
    }
    // Alarm triggers every 0.001sec
    ret_code = timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 100);
    if (ret_code != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to set alarm, error: %s", esp_err_to_name(ret_code));
        fatal_error();
    }
    ret_code = timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    if (ret_code != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to enable interrupt, error: %s", esp_err_to_name(ret_code));
        fatal_error();
    }
    ret_code = timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, &timer_callback, nullptr, 0);
    if (ret_code != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to add interrupt callback, error: %s", esp_err_to_name(ret_code));
        fatal_error();
    }
    ret_code = timer_start(TIMER_GROUP_0, TIMER_0);
    if (ret_code != ESP_OK) {
        ESP_LOGE("TIMER", "Failed to start the timer, error: %s", esp_err_to_name(ret_code));
        fatal_error();
    }
}

void update_oilp() {
    uint32_t raw = adc1_get_raw(ADC_OILP);
    adc1_read_counter++;
    adc1_level1_rolling_sum = adc1_level1_rolling_sum - adc1_level1[adc1_level1_pointer] + raw;
    adc1_level1[adc1_level1_pointer] = raw;
    adc1_level1_pointer++;
    if (adc1_level1_pointer == sizeof(adc1_level1)/sizeof(uint32_t))
        adc1_level1_pointer = 0;
    if (adc1_level1_pointer != 0) {
        uint32_t level2_raw = adc1_level1_rolling_sum / (sizeof(adc1_level1)/sizeof(uint32_t));
        adc1_level2_rolling_sum = adc1_level2_rolling_sum - adc1_level2[adc1_level2_pointer] + level2_raw;
        adc1_level2[adc1_level2_pointer] = level2_raw;
        adc1_level2_pointer++;
        if (adc1_level2_pointer == sizeof(adc1_level2)/sizeof(uint32_t))
            adc1_level2_pointer = 0;
    }

    raw = adc1_get_raw(ADC_AUX);
    adc2_read_counter++;
    adc2_level1_rolling_sum = adc2_level1_rolling_sum - adc2_level1[adc2_level1_pointer] + raw;
    adc2_level1[adc2_level1_pointer] = raw;
    adc2_level1_pointer++;
    if (adc2_level1_pointer == sizeof(adc2_level1)/sizeof(uint32_t))
        adc2_level1_pointer = 0;
    if (adc2_level1_pointer != 0) {
        uint32_t level2_raw = adc2_level1_rolling_sum / (sizeof(adc2_level1)/sizeof(uint32_t));
        adc2_level2_rolling_sum = adc2_level2_rolling_sum - adc2_level2[adc2_level2_pointer] + level2_raw;
        adc2_level2[adc2_level2_pointer] = level2_raw;
        adc2_level2_pointer++;
        if (adc2_level2_pointer == sizeof(adc2_level2)/sizeof(uint32_t))
            adc2_level2_pointer = 0;
    }
}

uint8_t get_normalized_oil_pressure() {
    uint32_t normalized_adc_raw = adc1_level2_rolling_sum / (sizeof(adc1_level2)/sizeof(uint32_t));
    uint32_t mV = esp_adc_cal_raw_to_voltage(normalized_adc_raw, &adc1_chars);
    //float actualV = ((float)mV)/OIL_PRESSURE_R1*(OIL_PRESSURE_R1+OIL_PRESSURE_R2)/1000;
    uint32_t actualV = mV * (OIL_PRESSURE_R1+OIL_PRESSURE_R2) / OIL_PRESSURE_R1;
    sampleCounter2++;
    if (sampleCounter2 >= 1000) {
        sampleCounter2  = 0;
        ESP_LOGD("ADC", "Sample Voltage: %d", actualV);
    }
    if (actualV < OIL_PRESSURE_VL)
        actualV = 0;
    else
        actualV -= OIL_PRESSURE_VL;
    actualV = actualV * OIL_PRESSURE_PMAX / (OIL_PRESSURE_VH - OIL_PRESSURE_VL);
    return (uint8_t)actualV;
}

uint32_t get_normalized_aux() {
    uint32_t normalized_adc_raw = adc2_level2_rolling_sum / (sizeof(adc2_level2)/sizeof(uint32_t));
    uint32_t mV = esp_adc_cal_raw_to_voltage(normalized_adc_raw, &adc1_chars);
    return mV;
}

uint32_t get_immediate_aux() {
    uint32_t raw = adc1_get_raw(ADC_AUX);
    uint32_t mV = esp_adc_cal_raw_to_voltage(raw, &adc1_chars);
     uint32_t actualV = mV * (OIL_PRESSURE_R1+OIL_PRESSURE_R2) / OIL_PRESSURE_R1;
     return actualV;
}

bool IRAM_ATTR timer_callback(void* arg) {
    shouldUpdateADC = true;
    return false;
}

void send_adc_data_to_can(MCP2515& mcpCan, uint8_t pressure, uint32_t aux_data) {
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = OIL_PRESSURE_CAN_ID;
    frame.can_dlc = 8;
    frame.data[0] = pressure;
    frame.data[1] = aux_data & 0xFF;
    frame.data[2] =  (aux_data >> 8) & 0xFF;
    frame.data[3] =  (aux_data >> 16) & 0xFF;
    frame.data[4] =  (aux_data >> 24) & 0xFF;
    MCP2515::ERROR err = mcpCan.sendMessage(&frame);
    if (err != MCP2515::ERROR_OK)
        ESP_LOGE("MCPCAN", "Failed sending oil pressure to CAN %d", err);
}

void handling_incoming_message(MCP2515&mcpCan) {
    twai_message_t message;
    esp_err_t ret_code;
    ret_code = twai_receive(&message, pdMS_TO_TICKS(1));
    switch(ret_code) {
        case ESP_OK:
            sampleCounter1++;
            if (sampleCounter1 >= 1000) {
                sampleCounter1 = 0;
                ESP_LOGD("ESPCAN", "Sampling Message ID: %d", message.identifier);
                for (int i = 0; i < message.data_length_code; i++) {
                    ESP_LOGD("ESPCAN", "Data byte %d = %d", i, message.data[i]);
                }
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