#pragma once
#include "comms_sim_pkg/hrcp_codec.hpp"
namespace comms_sim {
struct HrcpMac {
  unsigned type=0,ack_policy=0,pairnet=0x3101,destination=255,source=0;
  unsigned subframes=0,last_type=0,last_sequence=1023;
  bool buffer_empty=true;
};
struct HrcpMacBody {
  bool ok=false;
  unsigned sequence=0,command=0;
  std::vector<uint8_t> payload;
  std::string reason;
};
struct HrcpBeacon {
  unsigned slots=4,slot_tenths_us=105,period_us=140,atp_ms=500,next_id=1;
  int expected_rssi_dbm=-80;
  std::array<uint8_t,6> address{{2,0,0,0,0,2}};
};
std::array<uint8_t,10> hrcp_mac_encode(const HrcpMac &mac);
bool hrcp_mac_decode(const std::array<uint8_t,10>&bytes,HrcpMac &mac);
std::vector<uint8_t> hrcp_subframe(unsigned sequence,const std::vector<uint8_t>&payload);
std::vector<uint8_t> hrcp_command(unsigned sequence,unsigned command,const std::vector<uint8_t>&payload);
HrcpMacBody hrcp_body_decode(const HrcpMac&mac,const std::vector<uint8_t>&bytes);
std::vector<uint8_t> hrcp_beacon_encode(const HrcpBeacon&beacon);
bool hrcp_beacon_decode(const std::vector<uint8_t>&body,HrcpBeacon&beacon);
std::vector<uint8_t> hrcp_request_payload(unsigned atp_ms);
std::vector<uint8_t> hrcp_response_payload(unsigned devid,unsigned atp_ms);
bool hrcp_request_valid(const std::vector<uint8_t>&payload,unsigned &atp_ms);
bool hrcp_response_valid(const std::vector<uint8_t>&payload,unsigned devid,unsigned &atp_ms);
}
