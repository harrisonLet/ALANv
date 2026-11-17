/*
 * fakeCommands.h - Scripted ground station commands for testing the CM4 -> CM7 pipeline
 *
 * Only built when CM4_FAKE_COMMANDS is defined in shared_configuration.h.
 * Every CM4_FAKE_CMD_PERIOD_S seconds the TIM6 callback sets fake_rx_flag and the main
 * loop feeds the next packet into the same parse path a real LoRa packet takes, so the
 * radio only has to transmit telemetry.
 */

#ifndef FAKE_COMMANDS_H
#define FAKE_COMMANDS_H

#include <stdint.h>

// Seconds between fake commands. Keep it under the CM7's COMMANDER_LINK_TIMEOUT_MS (5 s),
// or manual and semi-auto modes drop to SAFE between commands.
#define CM4_FAKE_CMD_PERIOD_S (3U)

#define FAKE_CMD_PACKET_LEN (21U) // LORA_HDR_LEN_BYTES + IPC_CMD_DATA_SIZE, the longest packet main.c accepts

/**
  * @brief  Build the next scripted command as a full LoRa packet, then advance the script.
  * @param  buf  Destination, at least FAKE_CMD_PACKET_LEN bytes.
  * @retval Packet length: 5 with no payload, 7 for an angle, the same sizes the ground station sends.
  * @note   Wraps to the start of the script after the last step. Sequence numbers keep counting.
  */
uint8_t fakeCmd_next(uint8_t *buf);

#endif
