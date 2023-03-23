#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "modlog/modlog.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

void print_addr(uint8_t addr[]);

void ble_server_on_reset(int reason);

static void ble_server_advertise(void);

void ble_print_conn_desc(struct ble_gap_conn_desc *desc);

void ble_server_on_reset(int reason);

int ble_server_gap_event(struct ble_gap_event *event, void *arg);

void ble_server_advertise(void);

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

void ble_server_on_sync(void);

int gatt_svr_init(void);

void ble_server_host_task(void *param);

int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len, void *dst, uint16_t *len);