# Shared Protocol Skeleton

This folder hosts protocol artifacts shared between the STM32 firmware (`firmware/`) and the ESP32 bridge (`esp32/`):

- `robot_protocol.h` — versioned frame header definition, channel IDs, and starter command payload layouts.

Channel assignments follow `Migration.md` / `MIGRATION_AGENT.md`:

| Channel | ID | Purpose |
|---------|----|---------|
| CMD     | 1  | Teleop commands, mode control, heartbeats |
| TELEM   | 2  | State snapshots and one-shot events |
| FILE    | 3  | Log/file metadata, chunked transfers |
| RPC     | 4  | Request/response (calibration, system info, settings) |

`robot_protocol.h` is intentionally minimal right now—additional framing helpers (COBS/CRC, mux glue, etc.) will be layered in the `common/` tree as the migration progresses. All firmware code should include these shared headers instead of duplicating protocol constants.
