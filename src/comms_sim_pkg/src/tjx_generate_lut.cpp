#include "comms_sim_pkg/tjx_theoretical_phy.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

int main(int argc,char**argv){
 try{
  if(argc!=9)throw std::invalid_argument("使い方: tjx_generate_lut output.yaml trials seed data_bytes control_mcs esno_min esno_max step");
  const std::string output=argv[1];const unsigned trials=std::stoul(argv[2]),bytes=std::stoul(argv[4]),control=std::stoul(argv[5]);
  const uint64_t seed=std::stoull(argv[3]);double low=std::stod(argv[6]),high=std::stod(argv[7]),step=std::stod(argv[8]);
  if(!trials||trials>10000000||!std::isfinite(low)||!std::isfinite(high)||!std::isfinite(step)||low>=high||step<=0||low< -100||high>100||(high-low)/step>10000)
    throw std::invalid_argument("LUT生成範囲または試行数が不正です");
  if(std::filesystem::exists(output)||std::filesystem::exists(output+".partial"))throw std::invalid_argument("LUT出力先が既に存在します");
  auto keys=comms_sim::tjx_required_blocks(bytes,control);
  struct Job{int mcs;unsigned bits;double esno;};std::vector<Job> jobs;
  for(auto [mcs,bits]:keys)for(unsigned i=0;low+i*step<=high+1e-9;++i)jobs.push_back({mcs,bits,low+i*step});
  std::vector<std::string> rows(jobs.size());
  // 各セルの乱数系列を固定。スレッド数を変えてもLUTは同一になる。
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for(size_t i=0;i<jobs.size();++i){const auto&j=jobs[i];
    comms_sim::TjxBlockSimulator simulator(seed^(uint64_t(i+1)*0x9e3779b97f4a7c15ULL));uint64_t errors=0,bit_errors=0;
    for(unsigned n=0;n<trials;++n){auto result=simulator.Run(j.mcs,j.bits,j.esno);errors+=result.error;bit_errors+=result.bit_errors;}
    std::ostringstream row;row<<std::setprecision(17)<<"  - {mcs: "<<j.mcs<<", information_bits: "<<j.bits<<", es_no_db: "<<j.esno
      <<", trials: "<<trials<<", block_errors: "<<errors<<", bit_errors: "<<bit_errors<<"}\n";rows[i]=row.str();
  }
  std::ofstream out(output+".partial");if(!out)throw std::runtime_error("LUT出力先を開けません");
  out<<"model: tjx_ldpc_awgn_v1\nchip_rate_hz: 1760000000\ndecoder_iterations: 50\ndecoder_normalization: 0.8\n"
     <<"channel: ideal_sync_known_channel_awgn\nseed: "<<seed<<"\ndata_bytes: "<<bytes<<"\ncontrol_mcs: "<<control
     <<"\ntrials_per_point: "<<trials<<"\nprobability_estimator: raw_counts_no_smoothing\n"
     <<"length_policy: exact_shortened_ldpc_independent_codewords\nheader_model: EH_8_4_spreading_4\npoints:\n";
  for(const auto&row:rows)out<<row;
  out.close();if(!out)throw std::runtime_error("LUT書込み失敗");
  comms_sim::TjxTheoryConfig c;c.table=output+".partial";c.control_mcs=control;
  comms_sim::TjxTheoreticalPhy check(c);check.ValidateLengths(bytes);
  std::filesystem::rename(output+".partial",output);
  std::cout<<"LUT生成完了: "<<jobs.size()<<"点、各"<<trials<<"試行、"<<output<<'\n';
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
