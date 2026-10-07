// =====================================================================
//  v2v_protocol.h  --  Wire format for the V2V (ESP-NOW) link
//  V2V-Assisted Predictive Adaptive Cruise Control / BECE302L
//
//  MASTER COPY lives in firmware/common/. Do not edit the copies inside
//  the sketch folders -- run `python tools/sync_common.py` to propagate.
//
//  Design notes (ask-me-in-the-viva material):
//   * Fixed-size packed struct, little-endian, memcpy'd straight into the
//     ESP-NOW payload. No parsing, no allocation -> deterministic timing,
//     which is what a real-time control loop needs.
//   * seq lets the receiver MEASURE packet loss instead of assuming it.
//   * crc16 catches corruption. ESP-NOW already CRCs at the radio layer,
//     but this also catches struct-layout mismatch between two boards
//     built with different core versions -- a real failure we can detect.
//   * version + magic make a mismatched build fail loudly instead of
//     silently decoding garbage into a braking command.
// =====================================================================
#ifndef V2V_PROTOCOL_H
#define V2V_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define V2V_MAGIC            0x3256u   // 'V2' little-endian
#define V2V_PROTO_VERSION    1

// --- node identifiers -------------------------------------------------
#define V2V_NODE_LEAD        1
#define V2V_NODE_FOLLOW      2

// --- flag bits --------------------------------------------------------
#define V2V_FLAG_BRAKING     (1u << 0)  // lead is actively decelerating
#define V2V_FLAG_HAZARD      (1u << 1)  // lead declares emergency stop
#define V2V_FLAG_SIM         (1u << 2)  // sender is running a simulated plant
#define V2V_FLAG_MOVING      (1u << 3)  // lead speed above standstill threshold

// ---------------------------------------------------------------------
//  The broadcast packet. 26 bytes. ESP-NOW allows up to 250.
// ---------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint16_t magic;        // V2V_MAGIC
  uint8_t  version;      // V2V_PROTO_VERSION
  uint8_t  node_id;      // V2V_NODE_*
  uint32_t seq;          // monotonic, increments every transmission
  uint32_t t_ms;         // sender's millis() at time of send
  float    speed_mps;    // lead's own measured forward speed
  float    accel_mps2;   // lead's filtered acceleration (negative = slowing)
  uint8_t  flags;        // V2V_FLAG_*
  uint8_t  _reserved[3]; // keeps float alignment sane + room to grow
  uint16_t crc;          // CRC16-CCITT over all preceding bytes
} V2VPacket;

#define V2V_PACKET_SIZE   (sizeof(V2VPacket))
#define V2V_CRC_SPAN      (V2V_PACKET_SIZE - sizeof(uint16_t))

// ---------------------------------------------------------------------
//  CRC16-CCITT (poly 0x1021, init 0xFFFF). Small, no table, plenty fast
//  for a 24-byte span at 20 Hz.
// ---------------------------------------------------------------------
static inline uint16_t v2v_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// Stamp magic/version/crc into an otherwise-filled packet, ready to send.
static inline void v2v_seal(V2VPacket *p, uint8_t node_id) {
  p->magic   = V2V_MAGIC;
  p->version = V2V_PROTO_VERSION;
  p->node_id = node_id;
  p->_reserved[0] = p->_reserved[1] = p->_reserved[2] = 0;
  p->crc = v2v_crc16((const uint8_t *)p, V2V_CRC_SPAN);
}

// Returns 1 if the received buffer is a packet we should trust.
static inline int v2v_validate(const uint8_t *buf, int len, V2VPacket *out) {
  if (len != (int)V2V_PACKET_SIZE) return 0;
  V2VPacket tmp;
  memcpy(&tmp, buf, V2V_PACKET_SIZE);
  if (tmp.magic != V2V_MAGIC)             return 0;
  if (tmp.version != V2V_PROTO_VERSION)   return 0;
  if (tmp.crc != v2v_crc16((const uint8_t *)&tmp, V2V_CRC_SPAN)) return 0;
  if (out) *out = tmp;
  return 1;
}

// ---------------------------------------------------------------------
//  Arduino-core compatibility shim.
//  ESP32 Arduino core 3.x changed the ESP-NOW receive callback signature.
//  Both sketches use V2V_RECV_CB_ARGS / V2V_RECV_PEER_MAC so one source
//  file compiles on either core.
// ---------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define V2V_RECV_CB_ARGS   const esp_now_recv_info_t *info, \
                             const uint8_t *data, int len
  #define V2V_RECV_PEER_MAC  (info->src_addr)
#else
  #define V2V_RECV_CB_ARGS   const uint8_t *mac, const uint8_t *data, int len
  #define V2V_RECV_PEER_MAC  (mac)
#endif

#endif // V2V_PROTOCOL_H
