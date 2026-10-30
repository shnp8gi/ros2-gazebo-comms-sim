#include "comms_sim_pkg/hrcp_ldpc.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace comms_sim {
namespace {
constexpr std::array<std::array<unsigned, 3>, 15> rate11{{
  {{96,193,4}},{{34,320,135}},{{352,70,270}},{{104,306,287}},{{31,234,150}},
  {{159,364,91}},{{302,45,286}},{{126,239,371}},{{17,158,272}},{{28,336,178}},
  {{214,60,369}},{{219,145,372}},{{7,245,173}},{{19,140,373}},{{6,238,363}}
}};
constexpr std::array<std::array<unsigned, 3>, 15> rate14{{
  {{0,1,4}},{{32,34,39}},{{64,70,78}},{{8,18,95}},{{31,42,54}},
  {{63,76,91}},{{14,45,94}},{{30,47,83}},{{17,62,80}},{{28,48,82}},
  {{22,60,81}},{{27,49,84}},{{7,53,77}},{{19,44,85}},{{6,46,75}}
}};
constexpr double limit = 80.;
double bounded(double v) { return std::clamp(v, -limit, limit); }
}
HrcpLdpc::HrcpLdpc(unsigned k) : k_(k) {
  if (k != 1056 && k != 1344) throw std::invalid_argument("HRCP LDPCの情報長は1056または1344です");
  const auto &seed = k == 1056 ? rate11 : rate14;
  checks_.resize(N-k);
  parity_equations_.resize(N-k);
  for (unsigned j=0; j<N; ++j) {
    for (unsigned s : seed[j%15]) {
      const unsigned i = (s/96)*96 + (s%96+96-j/15)%96;
      checks_[i].push_back(j);
      parity_equations_[i].set(j);
    }
  }
  // GF(2)でH=[A|P]のPを単位行列へ変換。残ったAが系統符号のparity式。
  for (unsigned c=0; c<N-k; ++c) {
    unsigned pivot=c;
    while (pivot<N-k && !parity_equations_[pivot][k+c]) ++pivot;
    if (pivot==N-k) throw std::logic_error("LDPC parity行列が特異です");
    std::swap(parity_equations_[c],parity_equations_[pivot]);
    for (unsigned i=0; i<N-k; ++i)
      if (i!=c && parity_equations_[i][k+c]) parity_equations_[i]^=parity_equations_[c];
  }
}
std::vector<uint8_t> HrcpLdpc::Encode(const std::vector<uint8_t> &message) const {
  if (message.empty() || message.size()>k_) throw std::invalid_argument("LDPC情報bit数が範囲外です");
  std::bitset<N> info;
  std::vector<uint8_t> out(N,0);
  for (unsigned i=0; i<message.size(); ++i) {
    if (message[i]>1) throw std::invalid_argument("LDPC入力がbitではありません");
    out[i]=message[i]; info[i]=message[i];
  }
  for (unsigned i=0; i<N-k_; ++i) out[k_+i]=(parity_equations_[i]&info).count()%2;
  return out;
}
bool HrcpLdpc::SyndromeZero(const std::vector<uint8_t> &word) const {
  if (word.size()!=N) throw std::invalid_argument("LDPC符号長が1440bitではありません");
  for (auto b:word) if (b>1) throw std::invalid_argument("LDPC符号語がbitではありません");
  return SyndromeZeroUnchecked(word);
}
bool HrcpLdpc::SyndromeZeroUnchecked(const std::vector<uint8_t> &word) const {
  for (const auto &row:checks_) {
    unsigned s=0; for (auto j:row) s^=word[j];
    if (s) return false;
  }
  return true;
}
HrcpLdpc::Decoded HrcpLdpc::Decode(const std::vector<double> &llr,unsigned actual,
                                  unsigned max_iterations,double normalization) const {
  if (llr.size()!=N || actual==0 || actual>k_ || max_iterations==0 ||
      max_iterations>1000 || !std::isfinite(normalization) || normalization<=0 || normalization>1)
    throw std::invalid_argument("LDPC復号設定が不正です");
  std::vector<double> belief(N);
  for (unsigned i=0;i<N;++i) {
    if (!std::isfinite(llr[i])) throw std::invalid_argument("LDPC LLRに非有限値があります");
    belief[i]=(i>=actual && i<k_)?limit:bounded(llr[i]);
  }
  // 各bitの次数は3。行ごとの数百回のallocationを連続領域1個へまとめる。
  std::vector<double> messages(N*3,0.);
  std::vector<uint8_t> hard(N);
  Decoded result;
  for (unsigned iter=0;iter<=max_iterations;++iter) {
    for (unsigned i=0;i<N;++i) hard[i]=belief[i]<0;
    if (SyndromeZeroUnchecked(hard)) {result.converged=true;result.iterations=iter;break;}
    if (iter==max_iterations) {result.iterations=iter;break;}
    size_t offset=0;
    for (unsigned row=0;row<checks_.size();++row) {
      const auto &indices=checks_[row];double *old=messages.data()+offset;offset+=indices.size();
      double first=std::numeric_limits<double>::infinity(),second=first;
      size_t smallest=0; bool negative=false;
      // 最大row次数は45。固定配列で反復ごとのallocationを避ける。
      std::array<double,64> q{};
      if (indices.size()>q.size()) throw std::logic_error("LDPC row次数が想定を超えました");
      for (size_t e=0;e<indices.size();++e) {
        const unsigned j=indices[e];
        q[e]=(j>=actual && j<k_)?limit:bounded(belief[j]-old[e]);
        negative^=q[e]<0;
        const double a=std::abs(q[e]);
        if(a<first) {second=first;first=a;smallest=e;} else if(a<second) second=a;
      }
      for (size_t e=0;e<indices.size();++e) {
        const double mag=normalization*(e==smallest?second:first);
        old[e]=(negative^(q[e]<0))?-mag:mag;
        const unsigned j=indices[e];
        belief[j]=(j>=actual && j<k_)?limit:bounded(q[e]+old[e]);
      }
    }
  }
  result.information.assign(hard.begin(),hard.begin()+actual);
  return result;
}
}
