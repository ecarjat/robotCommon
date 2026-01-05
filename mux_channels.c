#include "mux_channels.h"

#include <string.h>

void robot_mux_init(robot_mux_t *mux)
{
    if (mux != NULL)
    {
        memset(mux, 0, sizeof(*mux));
    }
}

bool robot_mux_register(robot_mux_t *mux, robot_channel_t channel, robot_mux_handler_t handler, void *ctx)
{
    if (mux == NULL || handler == NULL)
    {
        return false;
    }

    if (channel == 0U || channel > ROBOT_CHANNEL_MAX)
    {
        return false;
    }

    mux->entries[channel].handler = handler;
    mux->entries[channel].ctx = ctx;
    return true;
}

void robot_mux_dispatch(robot_mux_t *mux, uint8_t msg_type, const uint8_t *payload, size_t payload_len)
{
    if (mux == NULL)
    {
        return;
    }

    robot_channel_t channel = robot_channel_from_type(msg_type);
    if (channel == 0U || channel > ROBOT_CHANNEL_MAX)
    {
        return;
    }

    robot_mux_handler_t handler = mux->entries[channel].handler;
    if (handler != NULL)
    {
        handler(msg_type, payload, payload_len, mux->entries[channel].ctx);
    }
}
