#ifndef _UARM_SWIFT_H_
#define _UARM_SWIFT_H_

#include "uarm_common.h"

#define DEVICE_NAME "SwiftPro"
#define HARDWARE_VERSION hardware_version
#define SOFTWARE_VERSION "V4.10.0"
#define API_VERSION "V4.0.5"
#define BLE_UUID bt_mac_addr

void uarm_swift_init(void);
void uarm_swift_tick_run(void);

#endif
