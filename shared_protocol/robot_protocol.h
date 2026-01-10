#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROBOT_FRAME_MAGIC          0x4B56U
#define ROBOT_FRAME_VERSION        0x01U
#define ROBOT_FRAME_MAX_PAYLOAD    240U
#define ROBOT_FRAME_HEADER_SIZE    10U
#define ROBOT_FRAME_MAX_DECODED    254U
#define ROBOT_FRAME_MAX_ENCODED    256U
#define ROBOT_FRAME_MAX_SIZE   (sizeof(robot_frame_header_t) + ROBOT_FRAME_MAX_PAYLOAD + sizeof(uint32_t))

#define ROBOT_FLAG_ACK_REQ 0x0001U
#define ROBOT_FLAG_IS_ACK  0x0002U

typedef enum
{
    ROBOT_CHANNEL_CMD   = 1U,
    ROBOT_CHANNEL_TELEM = 2U,
    ROBOT_CHANNEL_FILE  = 3U,
    ROBOT_CHANNEL_RPC   = 4U,
    ROBOT_CHANNEL_LOG   = 5U,
    ROBOT_CHANNEL_MAX   = 8U
} robot_channel_t;

typedef enum
{
    ROBOT_MSG_TELEM_FRAME        = 0x01U,
    ROBOT_MSG_TELEM_FRAME_V2     = 0x02U,
    ROBOT_MSG_CMD_TELEOP         = 0x10U,
    ROBOT_MSG_CMD_MODE           = 0x11U,
    ROBOT_MSG_CMD_ARM            = 0x12U,
    ROBOT_MSG_CMD_DISARM         = 0x13U,
    ROBOT_MSG_CMD_HEARTBEAT      = 0x14U,
    ROBOT_MSG_CMD_REBOOT         = 0x15U,  /* Reboot system (payload: 0=normal, 1=bootloader) */
    ROBOT_MSG_FILE_LIST_REQ      = 0x20U,
    ROBOT_MSG_FILE_LIST_RESP     = 0x21U,
    ROBOT_MSG_FILE_READ_REQ      = 0x22U,
    ROBOT_MSG_FILE_READ_RESP     = 0x23U,
    ROBOT_MSG_FILE_ERR           = 0x24U,
    ROBOT_MSG_RPC_REQ            = 0x30U,
    ROBOT_MSG_RPC_RESP           = 0x31U,
    ROBOT_MSG_ACK                = 0x7FU
} robot_msg_type_t;

#define ROBOT_RPC_METHOD_GET_PARAM       0x01U
#define ROBOT_RPC_METHOD_SET_PARAM       0x02U
#define ROBOT_RPC_METHOD_IMU_CALIB_FACE  0x10U
#define ROBOT_RPC_METHOD_MOTOR_ENABLE    0x20U
#define ROBOT_RPC_METHOD_MOTOR_DISABLE   0x21U
#define ROBOT_RPC_METHOD_MOTOR_RUN       0x22U
#define ROBOT_RPC_METHOD_BALANCE_ENABLE  0x23U
#define ROBOT_RPC_METHOD_BALANCE_DISABLE 0x24U

#define ROBOT_RPC_FLAG_SAVE        0x01U

#define ROBOT_TELEOP_FLAG_ARM        0x01U
#define ROBOT_TELEOP_FLAG_ESTOP      0x02U
#define ROBOT_TELEOP_FLAG_MODE_CYCLE 0x04U
#define ROBOT_TELEOP_FLAG_DUMP       0x08U  /* Trigger blackbox dump */

#define ROBOT_STATUS_ARMED   ROBOT_TELEOP_FLAG_ARM
#define ROBOT_STATUS_ESTOP   ROBOT_TELEOP_FLAG_ESTOP
#define ROBOT_STATUS_FAULT   0x04U
#define ROBOT_STATUS_LINK_OK 0x08U
#define ROBOT_STATUS_IMU_CAL 0x10U

#define ROBOT_FAULT_KILL_ANGLE    0x0001U
#define ROBOT_FAULT_IMU_TIMEOUT   0x0002U
#define ROBOT_FAULT_MOTOR_TIMEOUT 0x0004U

#define ROBOT_FAULT_KILL_ANGLE    0x0001U
#define ROBOT_FAULT_IMU_TIMEOUT   0x0002U
#define ROBOT_FAULT_MOTOR_TIMEOUT 0x0004U

#define ROBOT_RPC_STATUS_OK           0x00U
#define ROBOT_RPC_STATUS_BAD_LEN      0x01U
#define ROBOT_RPC_STATUS_BAD_OFFSET   0x02U
#define ROBOT_RPC_STATUS_STORAGE      0x03U
#define ROBOT_RPC_STATUS_BAD_METHOD   0x04U
#define ROBOT_RPC_STATUS_BAD_PARAM    0x05U
#define ROBOT_RPC_STATUS_NOT_READY    0x06U
#define ROBOT_RPC_STATUS_TIMEOUT      0x07U
#define ROBOT_RPC_STATUS_INCOMPLETE   0x08U

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
    uint16_t seq;
    uint16_t len;
    uint16_t flags;
} robot_frame_header_t;

typedef struct
{
    robot_frame_header_t hdr;
    uint8_t              payload[ROBOT_FRAME_MAX_PAYLOAD];
    uint32_t             crc32;
} robot_frame_t;

typedef struct
{
    float vx_mps;
    float wz_radps;
    uint8_t flags;   // bit0 arm, bit1 estop, bit2 mode_cycle, etc.
} robot_cmd_teleop_t;

typedef struct
{
    uint8_t  version;
    uint8_t  status;       // bit0 armed, bit1 estop, bit2 fault, bit3 link_ok
    uint16_t faults;       // bitfield of active faults
    uint32_t timestamp_ms; // system time
    float    pose_x_m;
    float    pose_y_m;
    float    yaw_rad;
    float    vx_mps;
    float    vy_mps;
    float    wz_radps;
    float    ax_mps2;
    float    ay_mps2;
    float    az_mps2;
    float    batt_v;
    float    batt_a;
    float    batt_pct;
    float    temp_c;
} robot_telem_v1_t;

/**
 * @brief Extended telemetry frame with EKF state and IMU health
 *
 * Sent at 500 Hz for real-time tuning visibility.
 * Per MainControl.md spec Section 14 and Code Review suggestion 4.5.
 */
typedef struct
{
    /* Base telemetry (same as v1) */
    uint8_t  version;
    uint8_t  status;       /* bit0 armed, bit1 estop, bit2 fault, bit3 link_ok */
    uint16_t faults;       /* bitfield of active faults */
    uint32_t timestamp_ms; /* system time */

    /* Motion mode */
    uint8_t  motion_mode;  /* motion_mode_t enum value */
    uint8_t  _reserved[3]; /* padding for alignment */

    /* EKF state estimate */
    float    theta_rad;    /* pitch angle (rad) */
    float    theta_dot;    /* pitch rate (rad/s) */
    float    x_m;          /* position estimate (m) */
    float    x_dot_mps;    /* velocity estimate (m/s) */
    float    gyro_bias;    /* EKF gyro bias estimate (rad/s) */
    uint8_t  estimate_valid;

    /* IMU health metrics */
    uint8_t  imu_active;       /* 0=BMI270, 1=ICM42688 */
    uint8_t  imu_gate_accel;   /* 1 if accel is gated due to vibration */
    uint8_t  _pad1;
    float    imu_gyro_diff_dps;    /* gyro disagreement (dps) */
    float    imu_vib_rms_g;        /* vibration RMS (g) */

    /* Wheel velocities */
    float    wheel_left_rps;   /* left wheel velocity (rad/s) */
    float    wheel_right_rps;  /* right wheel velocity (rad/s) */

    /* Control outputs */
    float    iq_left;          /* left motor Iq command */
    float    iq_right;         /* right motor Iq command */
    float    pitch_target_rad; /* pitch setpoint from velocity PID */

    /* Motor link diagnostics */
    uint32_t motor_left_ack_timeouts;
    uint32_t motor_right_ack_timeouts;

    /* ADC voltage (PC4) */
    float    adc_voltage; /* Voltage reading from ADC1 CH4 (PC4) with multiplier applied */
} robot_telem_v2_t;

typedef struct
{
    uint8_t  method;
    uint8_t  flags;   /* request: ROBOT_RPC_FLAG_* / response: ROBOT_RPC_STATUS_* */
    uint16_t offset;  /* byte offset into robot_params_t */
    uint16_t length;  /* bytes of data for SET/GET */
} robot_rpc_param_t;

typedef enum
{
    ROBOT_IMU_FACE_X_POS_UP = 0U,
    ROBOT_IMU_FACE_X_NEG_UP = 1U,
    ROBOT_IMU_FACE_Y_POS_UP = 2U,
    ROBOT_IMU_FACE_Y_NEG_UP = 3U,
    ROBOT_IMU_FACE_Z_POS_UP = 4U,
    ROBOT_IMU_FACE_Z_NEG_UP = 5U,
    ROBOT_IMU_FACE_COUNT = 6U
} robot_imu_face_t;

typedef enum
{
    ROBOT_MOTOR_SIDE_LEFT = 0U,
    ROBOT_MOTOR_SIDE_RIGHT = 1U
} robot_motor_side_t;

typedef struct
{
    uint8_t  imu;     /* 0 = primary (BMI270), 1 = secondary (ICM42688) */
    uint8_t  face;    /* robot_imu_face_t */
    uint16_t samples; /* 0 = default sample count */
} robot_rpc_imu_calib_face_t;

typedef struct
{
    uint8_t side;       /* robot_motor_side_t */
    uint8_t _reserved;
    float intensity;    /* [-1..1] normalized, scaled by IqMax */
} robot_rpc_motor_run_t;

/* File transfer protocol structures */
#define ROBOT_FILE_MAX_FILENAME 64U
#define ROBOT_FILE_CHUNK_SIZE   200U  /* Max chunk per frame (fits in 240B payload) */

typedef struct
{
    uint8_t _reserved;  /* Future: flags or path filtering */
} robot_file_list_req_t;

typedef struct
{
    char    filename[ROBOT_FILE_MAX_FILENAME];
    uint32_t size;      /* File size in bytes */
} robot_file_entry_t;

typedef struct
{
    uint8_t count;      /* Number of file entries in this response */
    uint8_t more;       /* 1 if more files exist (for pagination) */
} robot_file_list_resp_hdr_t;
/* Followed by robot_file_entry_t[count] */

typedef struct
{
    char     filename[ROBOT_FILE_MAX_FILENAME];
    uint32_t offset;    /* Byte offset to read from */
    uint16_t length;    /* Bytes to read (max ROBOT_FILE_CHUNK_SIZE) */
} robot_file_read_req_t;

typedef struct
{
    uint32_t offset;      /* Offset of this chunk */
    uint32_t total_size;  /* Total file size */
    uint16_t chunk_len;   /* Length of data in this response */
} robot_file_read_resp_t;
/* Followed by chunk_len bytes of file data */

typedef struct
{
    uint8_t error_code;  /* 1=not_found, 2=read_error, 3=busy, 4=invalid_offset */
    char    filename[ROBOT_FILE_MAX_FILENAME];
} robot_file_err_t;

#define ROBOT_FILE_ERR_NOT_FOUND    1U
#define ROBOT_FILE_ERR_READ_ERROR   2U
#define ROBOT_FILE_ERR_BUSY         3U
#define ROBOT_FILE_ERR_INVALID_REQ  4U

#pragma pack(pop)

robot_channel_t robot_channel_from_type(uint8_t type);
bool robot_msg_needs_ack(uint8_t type);

bool robot_frame_init(robot_frame_t *frame,
                      uint8_t type,
                      uint16_t seq,
                      uint16_t flags,
                      const uint8_t *payload,
                      uint16_t payload_len);

bool robot_frame_encode(const robot_frame_t *frame,
                        uint8_t *encoded_out,
                        size_t encoded_capacity,
                        size_t *encoded_len);

bool robot_frame_decode(const uint8_t *encoded,
                        size_t encoded_len,
                        robot_frame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROTOCOL_H */
