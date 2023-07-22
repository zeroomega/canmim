#ifndef CANMIM_CANMIM_H
#define CANMIM_CANMIM_H

#include <stdint.h>


#define CAN_MAX_DATALEN 8


#define CAN_ID_MASK 0x1FFFFFFFUL /* omit EFF, RTR, ERR flags */

// CAN Frame format used in CANMIM project. Fixed 17 Bytes
typedef struct {
    uint32_t timestamp;                 // Time stamp this frame is generated
                                        //   in ms since MCU starts.
    uint32_t can_id;                    // 32 bit CAN_ID + EFF/RTR/ERR flags
    
    uint8_t  data[CAN_MAX_DATALEN];     // CAN data payload
    uint8_t  can_dlc;                   // frame payload length in byte
                                        //   (0 .. CAN_MAX_DATALEN)
} can_frame_t;

#endif