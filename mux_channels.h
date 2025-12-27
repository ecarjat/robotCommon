#ifndef ROBOT_MUX_CHANNELS_H
#define ROBOT_MUX_CHANNELS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "shared_protocol/robot_protocol.h"

typedef void (*robot_mux_handler_t)(uint8_t msg_type,
                                    const uint8_t *payload,
                                    size_t payload_len,
                                    void *ctx);

typedef struct
{
    robot_mux_handler_t handler;
    void               *ctx;
} robot_mux_entry_t;

typedef struct
{
    robot_mux_entry_t entries[ROBOT_CHANNEL_MAX + 1U];
} robot_mux_t;

void robot_mux_init(robot_mux_t *mux);
bool robot_mux_register(robot_mux_t *mux, uint8_t channel, robot_mux_handler_t handler, void *ctx);
void robot_mux_dispatch(robot_mux_t *mux, uint8_t channel, uint8_t msg_type, const uint8_t *payload, size_t payload_len);

#endif /* ROBOT_MUX_CHANNELS_H */
