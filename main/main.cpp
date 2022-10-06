#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "mcp2515.h"

// SPI PIN Map
#define MCP_SCK   8
#define MCP_MOSI 10
#define MCP_MISO  9
#define MCP_CS    7

// ADC PIN Map
#define ADC_OILP  0

// Global SPI Handle for MCP2515 Can Controller
spi_device_handle_t spi_mcp2515;
esp_adc_cal_characteristics_t adc1_chars;

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
        printf("ADC Tow Point Calibration Data not available in FUSE %d\n", ret);
        fatal_error();
    }
    esp_adc_cal_value_t cal_ret = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 0, &adc1_chars);
    if (cal_ret == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Calibrate data TP\n");
    } else {
        printf("Calibrate data %d\n", cal_ret);
    }
    ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret != ESP_OK) {
        printf("ADC1 WIDTH Init failed %d\n", ret);
        fatal_error();
    }
    ret = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);
    if (ret != ESP_OK) {
        printf("ADC1 ATTEN Init failed %d\n", ret);
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
    printf("Prepared 2 struts\n");
    esp_err_t ret_code = spi_bus_initialize(SPI2_HOST, &s_bus_config, SPI_DMA_CH_AUTO);
    if (ret_code != ESP_OK) {
        printf("Failed to initialize SPI Bus, code %d\n", ret_code);
        fatal_error();
    } else {
        printf("SPI bus inited\n");
    }

    ret_code = spi_bus_add_device(SPI2_HOST, &s_dev_config, &spi_mcp2515);
    if (ret_code != ESP_OK) {
        printf("Failed to initialize SPI Device, code %d\n", ret_code);
        fatal_error();
    } else {
        printf("SPI device added\n");
    }
}

void sendTWAIMessageToCan(MCP2515 &mcpCan, twai_message_t *twaiMessage) {
    can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = twaiMessage->identifier;
    frame.can_dlc = twaiMessage->data_length_code;
    memcpy(frame.data, twaiMessage->data, 8);
    MCP2515::ERROR err = mcpCan.sendMessage(&frame);
    if (err != MCP2515::ERROR_OK) {
        printf("Failed sending to CAN %d\n", err);
    } else {
        printf("Can Message Sent\n");
    }
}

void init_can() {
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_3 /* TX */, GPIO_NUM_2 /* RX */, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("CAN bus Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        fatal_error();
    }
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        fatal_error();
    }
}

int rpm = 3500;
twai_message_t getMessage() {
    twai_message_t message;
    message.identifier = 0x40;
    if (rpm >=6000) {
        rpm = 3500;
    }
    message.data_length_code = 8;
    
    memset(&message.data, 0, 8);
    message.data[2] = rpm & 0xff;
    message.data[3] = (rpm & 0xff00)>>8;
    return message;
}

extern "C" void app_main(void)
{
    printf("Hello world!\n");

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
   
    
    while(1) {
        twai_message_t message;
        esp_err_t ret_code;
        ret_code = twai_receive(&message, pdMS_TO_TICKS(10000));
        switch(ret_code) {
            case ESP_OK:
                printf("ID is %d\n", message.identifier);
                if (!(message.rtr)) {
                    for (int i = 0; i < message.data_length_code; i++) {
                        printf("Data byte %d = %d\n", i, message.data[i]);
                    }
                    uint32_t adc_raw = adc1_get_raw(ADC1_CHANNEL_0);
                    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_raw, &adc1_chars);
                    float converted = ((float)voltage)/10*(10+22)/1000;

                    printf("Voltage: %.2f mv\n", converted);
                    sendTWAIMessageToCan(mcpCan, &message);
                }
                break;
            case ESP_ERR_INVALID_ARG:
                printf("Failed, Invalid Arg\n");
                break;
            case ESP_ERR_TIMEOUT:
                printf("Failed, Time Out\n");
                break;
            case ESP_FAIL:
                printf("Failed, ESP_FAIL\n");
                break;
            case ESP_ERR_INVALID_STATE:
                printf("Failed, Invalid State\n");
                break;
            case ESP_ERR_NOT_SUPPORTED:
                printf("Not supported\n");
                break;
        }
        
    }    
}
