#include "comms_sim_pkg/hrcp_mac.hpp"
#include <algorithm>
#include <stdexcept>
namespace comms_sim {
namespace {
void put(std::vector<uint8_t>&b,unsigned n,unsigned size){for(unsigned i=0;i<size;++i)b.push_back(n>>(8*i));}
unsigned get(const std::vector<uint8_t>&b,size_t p,unsigned n){unsigned v=0;for(unsigned i=0;i<n;++i)v|=unsigned(b.at(p+i))<<(8*i);return v;}
uint8_t reverse(uint8_t v){uint8_t r=0;for(unsigned i=0;i<8;++i)r=(r<<1)|((v>>i)&1);return r;}
const std::vector<uint8_t> address_a={2,0,0,0,0,1};
const std::vector<uint8_t> capability={101,0,128,1,0,0,0}; // SC, SIFS2.5us, pilot,16QAM; bondingなし。
const std::vector<uint8_t> operation={101,0,0,1,0,0,0};
bool ies(const std::vector<uint8_t>&b,size_t p,unsigned id,const std::vector<uint8_t>*expected=nullptr){
  bool found=false;while(p<b.size()){if(p+2>b.size()||p+2+b[p+1]>b.size())return false;
    if(b[p]==id){if(found)return false;found=true;if(expected&&(b[p+1]!=expected->size()||!std::equal(expected->begin(),expected->end(),b.begin()+p+2)))return false;}p+=2+b[p+1];}return found;
}
}
std::array<uint8_t,10> hrcp_mac_encode(const HrcpMac&m){
  if((m.type!=0&&m.type!=3&&m.type!=4)||m.last_sequence>1023||m.subframes>256||m.source>255||m.destination>255||m.pairnet>65535||m.last_type>1||(m.ack_policy!=0&&m.ack_policy!=2))throw std::invalid_argument("MAC header範囲外");
  std::vector<uint8_t>b;put(b,1|(m.type<<3)|(m.ack_policy<<7),2);put(b,m.pairnet,2);put(b,m.destination,1);put(b,m.source,1);
  put(b,m.subframes|(m.last_type<<9)|(m.last_sequence<<10)|(unsigned(m.buffer_empty)<<21),3);put(b,0,1);
  std::array<uint8_t,10>out;std::copy(b.begin(),b.end(),out.begin());return out;
}
bool hrcp_mac_decode(const std::array<uint8_t,10>&bytes,HrcpMac&m){
  std::vector<uint8_t>b(bytes.begin(),bytes.end());unsigned f=get(b,0,2),t=get(b,6,3);
  if((f&7)!=1||(f&64)||((f>>9)&1))return false;
  m.type=(f>>3)&7;m.ack_policy=(f>>7)&3;m.pairnet=get(b,2,2);m.destination=b[4];m.source=b[5];
  m.subframes=t&511;m.last_type=(t>>9)&1;m.last_sequence=(t>>10)&1023;m.buffer_empty=(t>>21)&1;
  return (m.type==0||m.type==3||m.type==4)&&(m.ack_policy==0||m.ack_policy==2)&&m.subframes<=1;
}
std::vector<uint8_t> hrcp_subframe(unsigned sequence,const std::vector<uint8_t>&payload){
  if(sequence>1023||payload.empty()||payload.size()>8192)throw std::invalid_argument("MAC subframe範囲外");
  std::vector<uint8_t>b;put(b,1|(sequence<<1)|((payload.size()-1)<<11),3);
  b.push_back(reverse(HrcpCodec::SubheaderCrc({b[0],b[1],b[2]})));
  auto checked=payload;HrcpCodec::AppendFcs(checked);b.insert(b.end(),checked.begin(),checked.end());return b;
}
std::vector<uint8_t> hrcp_command(unsigned sequence,unsigned command,const std::vector<uint8_t>&payload){
  std::vector<uint8_t>b;put(b,command,2);put(b,payload.size(),2);b.insert(b.end(),payload.begin(),payload.end());return hrcp_subframe(sequence,b);
}
HrcpMacBody hrcp_body_decode(const HrcpMac&m,const std::vector<uint8_t>&b){
  HrcpMacBody out;if(!m.subframes){out.ok=b.empty();out.reason=out.ok?"HEADER_ONLY":"UNEXPECTED_BODY";return out;}
  if(b.size()<9){out.reason="MAC_LENGTH";return out;}
  if(b[3]!=reverse(HrcpCodec::SubheaderCrc({b[0],b[1],b[2]}))){out.reason="SUBHEADER_HCS";return out;}
  unsigned info=get(b,0,3);out.sequence=(info>>1)&1023;unsigned length=(info>>11)+1;
  if(!(info&1)||b.size()!=length+8){out.reason="SUBFRAME_LENGTH";return out;}
  std::vector<uint8_t>checked(b.begin()+4,b.end());if(!HrcpCodec::CheckFcs(checked)){out.reason="PAYLOAD_FCS";return out;}
  out.payload.assign(checked.begin(),checked.end()-4);
  if(m.type==3){if(out.payload.size()<4||get(out.payload,2,2)!=out.payload.size()-4){out.reason="COMMAND_LENGTH";return out;}
    out.command=get(out.payload,0,2);out.payload.erase(out.payload.begin(),out.payload.begin()+4);}
  out.ok=true;out.reason="MAC_VALID";return out;
}
std::vector<uint8_t> hrcp_beacon_encode(const HrcpBeacon&v){
  if(v.slots<1||v.slots>6||v.slot_tenths_us>255||v.expected_rssi_dbm>30||v.expected_rssi_dbm< -225||v.next_id<1||v.next_id>236||v.period_us>65535||!v.atp_ms||v.atp_ms>65535)throw std::invalid_argument("Beacon値範囲外");
  std::vector<uint8_t>b;put(b,v.slots,1);put(b,v.slot_tenths_us,1);put(b,v.period_us,2);put(b,v.atp_ms,2);put(b,v.next_id,1);put(b,0,1);
  // ユーザー承認の暫定符号化:0.1us単位、ExpectedRSSI=30-code。
  put(b,30-v.expected_rssi_dbm,1);b.insert(b.end(),v.address.begin(),v.address.end());
  b.insert(b.end(),{1,6,'D','R','O','N','E','S',10,7});b.insert(b.end(),capability.begin(),capability.end());HrcpCodec::AppendFcs(b);return b;
}
bool hrcp_beacon_decode(const std::vector<uint8_t>&body,HrcpBeacon&v){
  if(body.size()<19||!HrcpCodec::CheckFcs(body))return false;
  std::vector<uint8_t>b(body.begin(),body.end()-4);
  if(!ies(b,15,10,&capability)||!ies(b,15,1))return false;
  v.slots=b[0];v.slot_tenths_us=b[1];v.period_us=get(b,2,2);v.atp_ms=get(b,4,2);v.next_id=b[6];v.expected_rssi_dbm=30-int(b[8]);std::copy_n(b.begin()+9,6,v.address.begin());
  return v.slots>=1&&v.slots<=6&&v.slot_tenths_us&&v.period_us&&v.atp_ms&&v.next_id>=1&&v.next_id<=236&&!(b[7]&16);
}
std::vector<uint8_t> hrcp_request_payload(unsigned atp){auto p=address_a;p.insert(p.end(),capability.begin(),capability.end());put(p,atp,2);return p;}
std::vector<uint8_t> hrcp_response_payload(unsigned id,unsigned atp){auto p=address_a;put(p,id,1);put(p,atp,2);put(p,0,1);p.insert(p.end(),{0x25,7});p.insert(p.end(),operation.begin(),operation.end());return p;}
bool hrcp_request_valid(const std::vector<uint8_t>&p,unsigned &atp){
  if(p.size()!=15||!std::equal(address_a.begin(),address_a.end(),p.begin())||!std::equal(capability.begin(),capability.end(),p.begin()+6))return false;
  atp=get(p,13,2);return atp>0;
}
bool hrcp_response_valid(const std::vector<uint8_t>&p,unsigned id,unsigned &atp){
  if(p.size()<10||!std::equal(address_a.begin(),address_a.end(),p.begin())||p[6]!=id||p[9]!=0||!ies(p,10,0x25,&operation))return false;
  atp=get(p,7,2);return atp>0;
}
}
