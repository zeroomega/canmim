
#ifndef CANMIM_BLE_H
#define CANMIM_BLE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#include "canmim.h"

void ble_init();

bool ble_notify(can_frame_t *data);

void set_ble_name(char *name, size_t len);

size_t copy_can_frame_to_rc_frame(can_frame_t *frame, uint8_t *rc_data);

#endif
