#ifndef _RC_RADIO_PACKETS_H
#define _RC_RADIO_PACKETS_H

typedef struct {
  uint8_t x_left;
  uint8_t y_left;
  uint8_t x_right;
  uint8_t y_right;
  uint8_t flags;
} Packet;

typedef struct {
  signed int rssi;
} PacketRSSI;

#endif
