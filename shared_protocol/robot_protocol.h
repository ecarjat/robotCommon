#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ROBOT_PROTOCOL_VERSION              1U
#define ROBOT_PROTOCOL_FRAME_MAX_PAYLOAD    256U
#define ROBOT_CHANNEL_MAX                   8U
#define ROBOT_PROTOCOL_FRAME_MAX_SIZE   (sizeof(robot_frame_header_t) + ROBOT_PROTOCOL_FRAME_MAX_PAYLOAD + sizeof(uint32_t))

typedef enum
{
    ROBOT_CHANNEL_CMD   = 1U,
    ROBOT_CHANNEL_TELEM = 2U,
    ROBOT_CHANNEL_FILE  = 3U,
    ROBOT_CHANNEL_RPC   = 4U
} robot_channel_t;

typedef enum
{
    ROBOT_CMD_TYPE_TELEOP    = 0x01U,
    ROBOT_CMD_TYPE_MODE      = 0x02U,
    ROBOT_CMD_TYPE_ARM       = 0x03U,
    ROBOT_CMD_TYPE_DISARM    = 0x04U,
    ROBOT_CMD_TYPE_HEARTBEAT = 0x05U
} robot_cmd_type_t;

typedef struct
{
    uint8_t     version;
    uint8_t     channel;
    uint8_t     msg_type;
    uint8_t     reserved;
    uint16_t    payload_len;
    uint16_t    reserved_len;
    uint32_t    seq;
} robot_frame_header_t;

typedef struct
{
    float forward;
    float turn;
    uint16_t flags;
    uint16_t reserved;
} robot_cmd_teleop_t;

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROTOCOL_H */
