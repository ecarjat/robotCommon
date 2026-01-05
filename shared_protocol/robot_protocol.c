#include "robot_protocol.h"

#include <string.h>

#include "../crc32.h"
#include "../framing_cobs.h"

static bool validate_header(const robot_frame_header_t *hdr)
{
    if (hdr->magic != ROBOT_FRAME_MAGIC)
    {
        return false;
    }
    if (hdr->version != ROBOT_FRAME_VERSION)
    {
        return false;
    }
    if (hdr->len > ROBOT_FRAME_MAX_PAYLOAD)
    {
        return false;
    }
    return true;
}

robot_channel_t robot_channel_from_type(uint8_t type)
{
    if (type >= ROBOT_MSG_CMD_TELEOP && type <= ROBOT_MSG_CMD_HEARTBEAT)
    {
        return ROBOT_CHANNEL_CMD;
    }
    if (type == ROBOT_MSG_TELEM_FRAME)
    {
        return ROBOT_CHANNEL_TELEM;
    }
    if (type >= ROBOT_MSG_FILE_LIST_REQ && type <= ROBOT_MSG_FILE_ERR)
    {
        return ROBOT_CHANNEL_FILE;
    }
    if (type == ROBOT_MSG_RPC_REQ || type == ROBOT_MSG_RPC_RESP)
    {
        return ROBOT_CHANNEL_RPC;
    }
    if (type == ROBOT_MSG_ACK)
    {
        return ROBOT_CHANNEL_LOG; // treat ACK as meta/log channel
    }
    return 0;
}

bool robot_msg_needs_ack(uint8_t type)
{
    return (type >= ROBOT_MSG_FILE_LIST_REQ && type <= ROBOT_MSG_FILE_ERR) ||
           (type == ROBOT_MSG_RPC_REQ || type == ROBOT_MSG_RPC_RESP);
}

bool robot_frame_init(robot_frame_t *frame,
                      uint8_t type,
                      uint16_t seq,
                      uint16_t flags,
                      const uint8_t *payload,
                      uint16_t payload_len)
{
    if (frame == NULL)
    {
        return false;
    }
    if (payload_len > ROBOT_FRAME_MAX_PAYLOAD)
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->hdr.magic = ROBOT_FRAME_MAGIC;
    frame->hdr.version = ROBOT_FRAME_VERSION;
    frame->hdr.type = type;
    frame->hdr.seq = seq;
    frame->hdr.len = payload_len;
    frame->hdr.flags = flags;

    if (payload_len > 0U && payload != NULL)
    {
        memcpy(frame->payload, payload, payload_len);
    }

    const size_t crc_len = sizeof(frame->hdr) + payload_len;
    frame->crc32 = robot_crc32((const uint8_t *)&frame->hdr, crc_len);
    return true;
}

bool robot_frame_encode(const robot_frame_t *frame,
                        uint8_t *encoded_out,
                        size_t encoded_capacity,
                        size_t *encoded_len)
{
    if (frame == NULL || encoded_out == NULL || encoded_len == NULL)
    {
        return false;
    }

    const size_t decoded_len = sizeof(frame->hdr) + frame->hdr.len + sizeof(frame->crc32);
    if (decoded_len > ROBOT_FRAME_MAX_DECODED)
    {
        return false;
    }

    uint8_t decoded[ROBOT_FRAME_MAX_DECODED];
    memcpy(decoded, &frame->hdr, sizeof(frame->hdr));
    if (frame->hdr.len > 0U)
    {
        memcpy(decoded + sizeof(frame->hdr), frame->payload, frame->hdr.len);
    }
    memcpy(decoded + sizeof(frame->hdr) + frame->hdr.len, &frame->crc32, sizeof(frame->crc32));

    size_t cobs_len = robot_cobs_encode(decoded, decoded_len, encoded_out, encoded_capacity);
    if (cobs_len == 0U || cobs_len + 1U > encoded_capacity)
    {
        return false;
    }

    encoded_out[cobs_len] = 0x00; // delimiter
    *encoded_len = cobs_len + 1U;
    return true;
}

bool robot_frame_decode(const uint8_t *encoded,
                        size_t encoded_len,
                        robot_frame_t *out_frame)
{
    if (encoded == NULL || out_frame == NULL)
    {
        return false;
    }
    if (encoded_len < 2U) // must include at least code byte and delimiter
    {
        return false;
    }
    if (encoded[encoded_len - 1U] != 0x00U)
    {
        return false;
    }

    uint8_t decoded[ROBOT_FRAME_MAX_DECODED];
    size_t decoded_len = robot_cobs_decode(encoded, encoded_len - 1U, decoded, sizeof(decoded));
    if (decoded_len < sizeof(robot_frame_header_t) + sizeof(uint32_t))
    {
        return false;
    }

    robot_frame_header_t hdr;
    memcpy(&hdr, decoded, sizeof(hdr));
    if (!validate_header(&hdr))
    {
        return false;
    }
    if ((size_t)hdr.len + sizeof(hdr) + sizeof(uint32_t) != decoded_len)
    {
        return false;
    }

    const size_t crc_offset = sizeof(hdr) + hdr.len;
    uint32_t crc_rx;
    memcpy(&crc_rx, decoded + crc_offset, sizeof(crc_rx));

    const uint32_t crc_calc = robot_crc32(decoded, crc_offset);
    if (crc_calc != crc_rx)
    {
        return false;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->hdr = hdr;
    if (hdr.len > 0U)
    {
        memcpy(out_frame->payload, decoded + sizeof(hdr), hdr.len);
    }
    out_frame->crc32 = crc_rx;
    return true;
}
