#include <limits>
#include <list>
#include <random>
#include <string>
#include <tuple>
#include <vector>
#include <unordered_map>
#include <utility>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <bits/stdc++.h>
#include <cstdlib>
#include <filesystem>
#include <cstdlib>


#include <sdf/sdf.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/msgs/dataframe.pb.h>
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "../../include/gazebo/systems/RFComms_custom.hh"

using namespace gz;
using namespace sim;
using namespace systems;

struct FileConfiguration
{
  std::string GetCurrentDateTime() 
  {
    std::time_t t = std::time(nullptr);
    std::tm now;

    localtime_r(&t, &now);

    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H-%M-%S", &now);
    return buffer;
  }

  std::string GetDirPath(std::string )
  {
    const char* userDirPath = std::getenv("USER");
    return std::string(userDirPath);
  }

  
  // std::string commsAnalysisDirPath = "/home/" + std::string(userDirPath) + "/ws/gazebo/comms_analysis/";
  // // std::string commsAnalysisDirPath = "./comms_analysis/";
  // std::string antennaGainsDirPath = "/home/" + std::string(userDirPath) + "ws/gazebo/antenna_gains/";
  // // std::string antennaGainsDirPath = "./antenna_gains/";
  
  // friend std::ostream &operator<<(std::ostream &_oss,
  //                                 const FileConfiguration &_config)
  // {
  //   _oss << "Comms Analysis Directory Path: " << _config.commsAnalysisDirPath << std::endl;
  //       //  << "Antenna Gain Directory Path: " << _config.antennaGainsDirPath << std::endl;

  //   return _oss;
  // }
};


/// \brief Parameters for simple log-normal fading model.
struct RangeConfiguration
{
  /// \brief Hard limit on range. 通信可能な最大距離
  double maxRange = 50.0;

  /// \brief Fading exponent.
  double fadingExponent = 2.5;

  /// \brief Received power at 1m (in dBm).
  double l0 = 40;

  /// \brief Standard deviation for received power.
  double sigma = 10;

  /// Output stream operator.
  /// \param[out] _oss Stream.
  /// \param[in] _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RangeConfiguration &_config)
  {
    _oss << "RF Configuration (range-based)" << std::endl
         << "-- max_range: " << _config.maxRange << std::endl
         << "-- fading_exponent: " << _config.fadingExponent << std::endl
         << "-- l0: " << _config.l0 << std::endl
         << "-- sigma: " << _config.sigma << std::endl;

    return _oss;
  }
};

/// \brief Radio configuration parameters.
///
/// Static parameters such as channel capacity and transmit power.
struct RadioConfiguration
{
  /// \brief Center Frequency (in GHz).
  double centerFrequency = 60.0;
  
  /// \brief Capacity of radio in bits-per-second.
  double capacity = 2.16 * pow(10.0, 9.0);

  /// \brief Default transmit power in dBm. Default is 27dBm or 500mW.
  double txPower = -10.0;

  /// \brief Modulation scheme, e.g., QPSK (Quadrature Phase Shift Keyring).
  std::string modulation = "QPSK";

  /// \brief Noise floor of the radio in dBm.
  double thermalNoiseDensity = -147.0;

  /// \brief File Path of E-Plane antenna gains.
  std::string ePlaneFileName = "e_plane.csv";

  /// \brief Vertical antenna direction gain.
  std::vector<std::vector<double>> ePlane;

  /// \brief File Path of E-Plane antenna gains.
  std::string hPlaneFileName = "h_plane.csv";

  /// \brief holizontal antenna direction gain.
  std::vector<std::vector<double>> hPlane;

  static constexpr int NumberOfAntenna = 3;

  /// \brief 各アンテナの回転角度を保持する構造体
  struct AntennaRotation
  {
    gz::math::Quaterniond No[3];
  };

  /// \brief 送信アンテナの回転角度
  AntennaRotation txAntennaRot = {
    {gz::math::Quaterniond(0., 0., -3*M_PI/4),
     gz::math::Quaterniond(0., 0., -M_PI/2),
     gz::math::Quaterniond(0., 0., -M_PI/4)}
  };

  /// \brief 受信アンテナの回転角度
  AntennaRotation rxAntennaRot = {
    {gz::math::Quaterniond(0., 0., 3*M_PI/4),
     gz::math::Quaterniond(0., 0., M_PI/2),
     gz::math::Quaterniond(0., 0., M_PI/4)}
  };

  /// \brief Communication duration from launching association.
  double associateDuration = 1.;

  /// \brief Threshold of rx power to launching setup.
  double powerThresholdForAssociation = -65.5;
  
  /// \brief Max antenna gain calculated by 3D antenna patterns.
  double maxAntennaGain;

  /// \brief Max antenna attenuation.
  double maxAntennaAttenuation = 30.;

  /// \brief Delay from expired comms to launching new setup.
  double switchingDelay;

  /// \brief Interval of broadcasting beacons (s)
  double beaconInterval;

  /// \brief Interval of plotting (s)
  double plotInterval = 0.001;

  /// \brief Whether communication with AP (like ground station) is sequential. 
  bool isScheduled = false;

  /// Output stream operator.
  /// \param _oss Stream.
  /// \param _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RadioConfiguration &_config)
  {
    _oss << "Radio Configuration" << std::endl
         << "-- center_frequency: " << _config.centerFrequency << std::endl
         << "-- capacity: " << _config.capacity << std::endl
         << "-- tx_power: " << _config.txPower << std::endl
         << "-- thermal_noise_density: " << _config.thermalNoiseDensity << std::endl
         << "-- modulation: " << _config.modulation << std::endl;

    return _oss;
  }
};

/// \brief Store radio state
///
/// Structure to hold radio state including the pose and book-keeping
/// necessary to implement bitrate limits.
struct RadioState
{
  /// \brief Timestamp of last update.
  double timeStamp;

  /// \brief Pose of the radio.
  gz::math::Pose3<double> pose;

  /// \brief Pose of the radio.
  gz::math::Vector3d antennaPos;

  /// \brief Angle of the radio.
  gz::math::Quaterniond antennaRot;

  /// \brief Recent sent packet history.filePath
  std::list<std::pair<double, uint64_t>> bytesSent;

  /// \brief Accumulation of bytes sent in an epoch.
  uint64_t bytesSentThisEpoch = 0;

  /// \brief Recent received packet history.
  std::list<std::pair<double, uint64_t>> bytesReceived;

  /// \brief Accumulation of bytes received in an epoch.
  uint64_t bytesReceivedThisEpoch = 0;

  /// \brief 
  double scheduledPlotTime = 0.;

  /// \brief
  double scheduledBeaconTransmittionTime = 0.;
  
  /// \brief
  std::string scheduledAssociateWith = "";

  /// \brief
  int scheduledAssociationIndex = 0;

  /// \brief Received Sgzal Strngth Indicator.
  std::unordered_map<std::string, double> rssi;

  /// \brief 
  std::string currentSetupWith = "";

  /// \brief Time setup phase completed.
  double timeSetupCompleted = 0;

  /// \brief Time expired association.
  double timeout = 0.;

  double timeStartAssociation;

  /// \brief
  std::string currentAssociateWith = "";

  /// \brief 
  double totalDataTransferred = 0.;

  /// \brief .
  // std::vector<std::pair<double, double>> totalTimeAndDataTransmittionInItsLevels = {{-39., 0.},
  //                                                             {-45., 0.},
  //                                                             {-49., 0.},
  //                                                             {-51., 0.},
  //                                                             {-55., 0.},
  //                                                             {-58., 0.},
  //                                                             {-61., 0.}};
  std::vector<std::vector<double>> totalTimeAndDataTransmittionInItsLevels = {{-39., 13.1413, 0., 0.},
                                                                              {-45., 9.856, 0., 0.},
                                                                              {-49., 7.744, 0., 0.},
                                                                              {-51., 6.5707, 0., 0.},
                                                                              {-55., 5.1627, 0., 0.},
                                                                              {-58., 3.2853, 0., 0.},
                                                                              {-61., 2.5813, 0., 0.}};

  // std::vector<std::pair<double, double>> totalTimeAndDataTransmittionInItsLevels = {{-39., 0.},
  //                                                             {-45., 0.},
  //                                                             {-49., 0.},
  //                                                             {-51., 0.},
  //                                                             {-55., 0.},
  //                                                             {-58., 0.},
  //                                                             {-61., 0.}};

  /// \brief
  double previousTimeStamp = 0.;

  /// \brief Name of the model associated with the radio.
  std::string name;
};

/// \brief Type for holding RF power as a Normally distributed random variable.
struct RFPower
{
  /// \brief Expected value of RF power.
  double mean;

  /// \brief Variance of RF power.
  double variance;

  /// \brief double operator.
  /// \return the RFPower as a double.
  operator double() const
  {
    return mean;
  }
};

/// \brief Private RFComms_custom data class.
class gz::sim::systems::RFComms_custom::Implementation
{
  /// \brief Attempt communication between two nodes.
  ///
  /// The radio configuration, transmitter and receiver state, and
  /// packet size are all used to compute the probability of successful
  /// communication (i.e., based on both SNR and bitrate
  /// limitations). This probability is then used to determine if the
  /// packet is successfully communicated.
  ///
  /// \param[in out] _txState Current state of the transmitter.
  /// \param[in out] _rxState Current state of the receiver.
  /// \param[in] _numBytes Size of the packet.
  /// \return std::tuple<bool, double> reporting if the packet should be
  /// delivered and the received sgzal strength (in dBm).

  public: double RssiToSinr(std::string address, RadioState _rxState);

  public: std::vector<double> MeasureRssi(RadioState &_txState,
                                         RadioState &_rxState) const;

  public: int judgmentOfUseAntenna(RadioState &_txState, 
                                   RadioState &_rxState) const;

  /// \brief Convert from e_plane.csv and h_plane.csv to two dimensional array.
  public: std::vector<std::vector<double>> FileToArray(std::string _filePath) const;

  private: double PoseToGain(const RadioState &_initialPointState,
                             const RadioState &_terminalPointState,
                             const gz::math::Quaterniond &_currentAntennaRot) const;

  /// \brief Convert degree to radian.
  public: double DegreeToRadian(double _degree) const;

  /// \brief Convert degree to radian.
  public: double RadianToDegree(double _radian) const;

  /// \brief Convert from dBm to power.
  /// \param[in] _dBm Input in dBm.
  /// \return Power in watts (W).
  private: double DbmToPow(double _dBm) const;

  /// \brief Compute the bit error rate (BER).
  /// \param[in] _power Rx power (dBm).
  /// \param[in] _noise Noise value (dBm).
  /// \return Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER).
  private: double QPSKPowerToBER(double _power,
                                 double _noise) const;

  private: double SinrToBer(double _sinr,
                             std::string _modulation) const;
  
  

  /// \brief Function to compute the pathloss between two antenna poses.
  /// \param[in] _txPower Tx power.
  /// \param[in] _txState Radio state of the transmitter.
  /// \param[in] _rxState Radio state of the receiver.
  /// \return The RFPower pathloss distribution of the two antenna poses. https://en.wikipedia.org/wiki/Log-distance_path_loss_model
  private: RFPower FreeSpaceReceivedPower(const double &_txPower,
                                          const RadioState &_txState,
                                          const RadioState &_rxState) const;
  
  private: RFPower LogNormalReceivedPower(const double &_txPower,
                                          const RadioState &_txState,
                                          const RadioState &_rxState) const;

  /// \brief Convert RSSI to throuput based on examination NICT operated.
  /// \param[in] _rssi received sgzal strength indicator（RSSI）.
  public: double RssiToThrouputInGbps(double _rssi) const;

  public: void RssiToTotalTimeAndDataTransmittionInItsLevels(const double _rssi,
                                                             const double _timeStampDuration,
                                                             std::vector<std::vector<double>>& _totalTimeAndDataTransmittionInItsLevels) const;
                      
  public: double TotalTimeAndDataTransmittionInItsLevelsToTotalDataTransmittion(const std::vector<std::vector<double>> _totalTimeAndDataTransmittionInItsLevels) const;

  // public: double GetAssociateDuration(std::string _srcAddress);

  /// \brief Range configuration.
  public: RangeConfiguration rangeConfig;

  /// \brief Radio configuration.
  public: RadioConfiguration radioConfig;

  /// \brief File configuration.
  public: FileConfiguration fileConfig;

  /// \brief A map where the key is the address and the value its radio state.
  public: std::unordered_map<std::string, RadioState> radioStates;

  /// \brief Duration of an epoch (seconds).
  public: double epochDuration = 1.0;

  /// \brief Random device to seed random engine
  public: std::random_device rd{};

  /// \brief Random number generator.
  public: mutable std::default_random_engine rndEngine{rd()};

  /// \brief Writing file.
  public: std::ofstream writing_file;

};

/////////////////////////////////////////////
std::vector<std::vector<double>> RFComms_custom::Implementation::FileToArray(std::string _filePath) const
{ 
  std::ifstream file(_filePath);
  std::vector<std::vector<double>> array;

  if (file)
  {
    std::string line;
    while (getline(file, line))
    {
      std::vector<double> row;
      std::stringstream ss(line);

      std::string value;
      while (std::getline(ss, value, ','))
      {
        row.push_back(std::stod(value));
      }
      array.push_back(row);
    }
  }
  else
    gzerr << "Cannot read " << _filePath << std::endl;
  
  return array;
}

/////////////////////////////////////////////
double RFComms_custom::Implementation::DegreeToRadian(double _degree) const
{
  return _degree * M_PI/180.0;
}

double RFComms_custom::Implementation::RadianToDegree(double _radian) const
{
  return _radian * 180.0/M_PI; 
}

/////////////////////////////////////////////
// double RFComms_custom::Implementation::DbmToPow(double _dBm) const
// {
//   return 0.001 * pow(10., _dBm / 10.);
// }

double RFComms_custom::Implementation::DbmToPow(double _dBm) const
{
  return pow(10., _dBm / 10.);
}


///////////////////////////////////////////// 
double RFComms_custom::Implementation::PoseToGain(
  const RadioState &_initialPointState,
  const RadioState &_terminalPointState,
  const gz::math::Quaterniond &_currentAntennaRot) const
{

  // (送信側/受信側)から見た(受信側/送信側)のベクトル(_terminalPointState.pose.Pos() - _initialPointState.pose.Pos())を \
  (送信側/受信側)アンテナの回転だけ逆回転させる(_currentAntennaRot.RotateVectorReverse)ことで \
  実質的に(送信側/受信側)アンテナから見た(受信側/送信側)のベクトルを算出する

  // 例えば、送信側が(x y z)(r p y)=(0 0 0)(0 0 0)でアンテナ角が(r p y)=(0 0 1/4*pi) \
  受信側が(x y z)(r p y)=(3 3 0)(0 0 0)でアンテナ角が(r p y)=(0 0 -3/4*pi)の場合 \
  得られるベクトル direction は送信側/受信側でいずれも(x y z)=(3√2 0 0)となる

  auto direction = _currentAntennaRot.RotateVectorReverse(_terminalPointState.pose.Pos() - _initialPointState.pose.Pos());
  
  igndbg << "direction: " << direction << " [" << _initialPointState.name << "]" << std::endl;

  // E面における(放射角/到来角）、thetaを空間ベクトルの内積の公式より算出する
  auto eDirection = direction;
  eDirection.Y(0.0);

  double theta = round((this->RadianToDegree(acos(eDirection.Normalized().Dot(gz::math::Vector3d::UnitX))))*10.)/10.;
  if (eDirection.Z() < 0.0)
  {
    theta = -theta;
  }

  // 算出したthetaをE面のアンテナパターンにあてはめる
  auto itEPlane = std::find_if(
    std::begin(this->radioConfig.ePlane), std::end(this->radioConfig.ePlane),
    [&](const auto& row) {
      return row.at(0) == theta;
    }
  );

  // H面における(放射角/到来角）、varphiを空間ベクトルの内積の公式より算出する
  auto hDirection = direction;
  hDirection.Z(0.0);

  double varphi = round(this->RadianToDegree(acos(hDirection.Normalized().Dot(gz::math::Vector3d::UnitX)))*10.)/10.;
  if (hDirection.Y() < 0.0)
  {
    varphi = -varphi;
  }

  // 算出したvarphiをH面のアンテナパターンにあてはめる
  auto itHPlane = std::find_if(
    std::begin(this->radioConfig.hPlane), std::end(this->radioConfig.hPlane),
    [&](const auto& row) {
      return row.at(0) == varphi;
    }
  );

  // gzdbg << "[" << _initialPointState.name << "]" << std::endl;
  // gzdbg << "theta: " << theta << " varphi: " << varphi << std::endl;

  // いずれかでもあてはまらなかった場合、アンテナ利得は-infになる
  if (itEPlane == std::end(this->radioConfig.ePlane) || itHPlane == std::end(this->radioConfig.hPlane)) 
    return -std::numeric_limits<double>::infinity();

  // アンテナ素子の指向性利得　= アンテナ素子の最大指向性利得(dBi) - E面のアンテナ素子の放射パターンによる減衰(dB)  - H面のアンテナ素子の放射パターンによる減衰(dB)で算出する \
  プログラム上はアンテナ素子の最大指向性利得(dBi) - E面のアンテナ素子の放射パターンによる減衰(dB) + アンテナ素子の最大指向性利得(dBi) - H面のアンテナ素子の放射パターンによる減衰(dB) - アンテナ素子の最大指向性利得(dBi)
  double antennaGainEPlane = this->radioConfig.ePlane[std::distance(std::begin(this->radioConfig.ePlane), itEPlane)][1];
  double antennaAttenuationEPlane = this->radioConfig.maxAntennaGain - antennaGainEPlane;
  if (antennaAttenuationEPlane > this->radioConfig.maxAntennaAttenuation)
  {
    antennaAttenuationEPlane = this->radioConfig.maxAntennaAttenuation;
    // gzwarn << "E面で下限値を超えました" << std::endl;
  }
  
  double antennaGainHPlane = this->radioConfig.hPlane[std::distance(std::begin(this->radioConfig.hPlane), itHPlane)][1];
  double antennaAttenuationHPlane = this->radioConfig.maxAntennaGain - antennaGainHPlane;
  if (antennaAttenuationHPlane > this->radioConfig.maxAntennaAttenuation)
  {
    antennaAttenuationHPlane = this->radioConfig.maxAntennaAttenuation;
    // gzwarn << "H面で下限値を超えま���た" << std::endl;
  }

  double antennaGain = this->radioConfig.maxAntennaGain - antennaAttenuationEPlane - antennaAttenuationHPlane;

  double minAntennaGain = this->radioConfig.maxAntennaGain - this->radioConfig.maxAntennaAttenuation;

  if (antennaGain < minAntennaGain)
  {
    antennaGain = minAntennaGain;
    // gzwarn << "3Dで下限値を超えました" << std::endl;
  }

  // gzwarn << "AntennaGain(E-Plane): " << antennaGainEPlane << std::endl;
  // gzwarn << "AntennaGain(H-Plane): " << antennaGainHPlane << std::endl;
  // gzwarn << "AntennaGain3d: " << antennaGain << std::endl;
                      
  return antennaGain;
}

/////////////////////////////////////////////
RFPower RFComms_custom::Implementation::FreeSpaceReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double speedOfLight = 3.0 * pow(10.0, 8.0);
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange) // 通信可能範囲外の場合
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = 20. * log10(4. * M_PI * kRange * this->radioConfig.centerFrequency/speedOfLight);

  return {_txPower - kPL, pow(this->rangeConfig.sigma, 2.)};
}

/////////////////////////////////////////////
RFPower RFComms_custom::Implementation::LogNormalReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange) // 通信可能範囲外の場合
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = this->rangeConfig.l0 +
    10 * this->rangeConfig.fadingExponent * log10(kRange); // 対数距離経路損失モデル https://en.wikipedia.org/wiki/Log-distance_path_loss_model

  return {_txPower - kPL, pow(this->rangeConfig.sigma, 2.)};
}

std::vector<double> RFComms_custom::Implementation::MeasureRssi(
    RadioState &_txState, RadioState &_rxState) const
{
    // アンテナの数を取得
    int NumberOfAntennas = this->radioConfig.NumberOfAntenna;

    // 送信アンテナの回転を計算
    std::vector<gz::math::Quaterniond> txCurrentAntennaRot(NumberOfAntennas);
    for (int i = 0; i < NumberOfAntennas; ++i) {
        txCurrentAntennaRot[i] = _txState.antennaRot * this->radioConfig.txAntennaRot.No[i];
    }

    // 送信アンテナの利得を計算
    std::vector<double> txAntennaGain(NumberOfAntennas);
    std::vector<RFPower> rxPowerDist(NumberOfAntennas);
    for (int i = 0; i < NumberOfAntennas; ++i) {
        txAntennaGain[i] = this->PoseToGain(_txState, _rxState, txCurrentAntennaRot[i]);
    }

    for (int i = 0; i < NumberOfAntennas; i ++) {
      gzdbg << "gain of transmitting antenna " << i << ":" << rxPowerDist[i] << std::endl; 
    }

    // 受信電力を計算
    for (int i = 0; i < NumberOfAntennas; ++i) {
        rxPowerDist[i] = this->LogNormalReceivedPower(
            this->radioConfig.txPower + txAntennaGain[i], _txState, _rxState);
    }

    // 雑音を加味した実受信電力の計算
    std::vector<double> rxPower(NumberOfAntennas);
    for (int i = 0; i < NumberOfAntennas; ++i) {
        rxPower[i] = rxPowerDist[i].mean;
        if (rxPowerDist[i].variance > 0.0) {
            std::normal_distribution<> d{rxPowerDist[i].mean, sqrt(rxPowerDist[i].variance)};
            rxPower[i] = d(this->rndEngine);
        }
    }

    for (int i = 0; i < NumberOfAntennas; i ++) {
      gzdbg << "power of recieving antenna " << i << ":" << rxPower[i] << std::endl; 
    }

    // 受信アンテナの回転を計算
    std::vector<gz::math::Quaterniond> rxCurrentAntennaRot(NumberOfAntennas);
    for (int i = 0; i < NumberOfAntennas; ++i) {
        rxCurrentAntennaRot[i] = _rxState.antennaRot * this->radioConfig.rxAntennaRot.No[i];
    }

    // 受信アンテナの利得を計算
    std::vector<double> rxAntennaGain(NumberOfAntennas);
    for (int i = 0; i < NumberOfAntennas; ++i) {
        rxAntennaGain[i] = this->PoseToGain(_rxState, _txState, rxCurrentAntennaRot[i]);
    }

    for (int i = 0; i < NumberOfAntennas; i ++) {
      gzdbg << "gain of recieving antenna " << i << ":" << rxPowerDist[i] << std::endl; 
    }

    // RSSIの計算
    std::vector<double> rssi(NumberOfAntennas * NumberOfAntennas);
    for (int i = 0; i < NumberOfAntennas; ++i) {
        for (int j = 0; j < NumberOfAntennas; ++j) {
            rssi[i * NumberOfAntennas + j] = rxPower[j] + rxAntennaGain[i];
        }
    }

    return rssi;
}

double RFComms_custom::Implementation::RssiToSinr(
  std::string _address, RadioState _rxState)
{
  double rssi;
  double interferencePower = 0.;
  for (const auto& pair : _rxState.rssi)
  {
    double pow = this->DbmToPow(pair.second);
    if (_address == pair.first)
      rssi = pow;
    else
      interferencePower += pow;
  }

  double noiseFloor= this->DbmToPow(this->radioConfig.thermalNoiseDensity)*this->radioConfig.capacity;

  // gzdbg << _address << "'s rssi[mW]: " << rssi << std::endl;
  // gzdbg << "interference_power[mW]: " << interferencePower << std::endl;
  // gzdbg << "noiseFloor[mW]: " << noiseFloor << std::endl;

  return 10*log10(rssi/(interferencePower + noiseFloor));
}

////////////////////////////////////////////
double RFComms_custom::Implementation::QPSKPowerToBER(
  double _power, 
  double _noise) const
{
  return erfc(sqrt(_power / _noise)); // これあってる？？
}

double RFComms_custom::Implementation::SinrToBer(
  double _sinr,
  std::string _modulation) const
{ 
  if (_modulation.compare("QPSK") == 0)
    return 0.5 * erfc(sqrt(_sinr/2.0));
  if (_modulation.compare("16QAM"))
    return 0.375 * erfc(sqrt(_sinr/10.0)); 
  if (_modulation.compare("64QAM"))
    return (sqrt(64.0) - 1.0)/(log2(64.0) * pow(2.0, log2(64.0)/2.0 - 1.0)) * erfc(sqrt(_sinr * (3.0/(2.0 * (64.0 - 1.0)))));
  if (_modulation.compare("256QAM"))
    return (sqrt(256.0) - 1.0)/(log2(256.0) * pow(2.0, log2(256.0)/2.0 - 1.0)) * erfc(sqrt(_sinr * (3.0/(2.0 * (256.0 - 1.0)))));
  return std::numeric_limits<double>::lowest();
}

// NICTから提供いただいたテーブルを元に作成
double RFComms_custom::Implementation::RssiToThrouputInGbps(
  double _rssi) const
{
  if (_rssi > -51.0)
    return 6.0;
  else if (_rssi > -55.0)
    return 4.7;
  else if (_rssi > -58.5)
    return 2.7;
  else if (_rssi > -61.5)
    return 2.15;
  else if (_rssi > -63.5)
    return 1.1;
  else if (_rssi > -65.5)
    return 0.5;
  else
    return 0.0;
}

void RFComms_custom::Implementation::RssiToTotalTimeAndDataTransmittionInItsLevels(
  const double _rssi,
  const double _timeStampDuration,
  std::vector<std::vector<double>>& _totalTimeAndDataTransmittionInItsLevels) const
{
  for (auto& totalTimeAndDataTransmittionInItsLevel : _totalTimeAndDataTransmittionInItsLevels)
  {
    if (_rssi > totalTimeAndDataTransmittionInItsLevel.at(0))
    {
      totalTimeAndDataTransmittionInItsLevel.at(2) += _timeStampDuration;
      totalTimeAndDataTransmittionInItsLevel.at(3) = totalTimeAndDataTransmittionInItsLevel.at(1)*totalTimeAndDataTransmittionInItsLevel.at(2);
      break;
    }
  }
}

double RFComms_custom::Implementation::TotalTimeAndDataTransmittionInItsLevelsToTotalDataTransmittion(
  const std::vector<std::vector<double>> _totalTimeAndDataTransmittionInItsLevels) const
{
  double totalDataTransmittionInGb = 0.;
  for (const auto& _totalTimeAndDataTransmittionInItsLevel: _totalTimeAndDataTransmittionInItsLevels)
  {
    totalDataTransmittionInGb += _totalTimeAndDataTransmittionInItsLevel.at(3);
  }
  return totalDataTransmittionInGb;
}

// 一時的に使う関数
// double RFComms_custom::Implementation::GetAssociateDuration(
//   std::string _srcAddress
// )
// {
//   if (_srcAddress == "ground_station_antenna_0")
//     return 4;
//   if (_srcAddress == "ground_station_antenna_1")
//     return 4;
//   if (_srcAddress == "ground_station_antenna_2")
//     return 4;
//   gzerr << "Unexpected model exists '" << _srcAddress << "'. Associate duration for its model was default value." << std::endl;
//   return this->radioConfig.associateDuration;
// }

int gz::sim::systems::RFComms_custom::Implementation::judgmentOfUseAntenna(
  RadioState &_txState, 
  RadioState &_rxState) const {

  if (this->radioConfig.NumberOfAntenna <= 1) {
    gzerr << "Invalid antenna count." << std::endl;
    return -1;
  }
  
  // 送信側の隣り合うアンテナの中間（角度）を切り替え位置に設定し，x軸上でその角度となる座標を算出している．
  std::vector<double> SwitchingPosition(this->radioConfig.NumberOfAntenna - 1);
  for (int i = 0; i < this->radioConfig.NumberOfAntenna - 1; i++) {
    double SwitchingAngle = (this->radioConfig.txAntennaRot.No[i].Yaw() + this->radioConfig.txAntennaRot.No[i+1].Yaw()) / 2;
    SwitchingPosition[i] = _txState.pose.Pos().X() - (_txState.pose.Pos().Y()-_rxState.pose.Pos().Y()) * std::tan(-SwitchingAngle - M_PI/2);
  }

    // 通信を行うときに使用する受信側のアンテナ番号を受信側の座標によって取得している．
    // 直進のみのシナリオでは送信側のアンテナ番号は (NumberOfAntennas - 1) - CommuAntenna で計算できるためここでは返り値を受信側のアンテナ番号のみとしている．
  double rxPosX = _rxState.pose.Pos().X();
  int CommuAntenna = -1;

  if (rxPosX <= SwitchingPosition[0]) {
    CommuAntenna = this->radioConfig.NumberOfAntenna - 1;
  } else {
    for (int i = 0; i < this->radioConfig.NumberOfAntenna - 2; i++) {
      if (SwitchingPosition[i] < rxPosX && rxPosX <= SwitchingPosition[i + 1]) {
        CommuAntenna = this->radioConfig.NumberOfAntenna - (i + 2);
        break;
      }
    }
    if (rxPosX > SwitchingPosition.back()) {
      CommuAntenna = 0;
    }
  }

  if (CommuAntenna == -1) {
    gzerr << "Unexpected value for rxPosX: " << rxPosX << std::endl;
  }

  return CommuAntenna;
}

//////////////////////////////////////////////////
RFComms_custom::RFComms_custom()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void RFComms_custom::Load(const Entity &/*_entity*/,
    std::shared_ptr<const sdf::Element> _sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{  
  // Generate  files
  const char* userDirPath = std::getenv("HOME");
  if (!userDirPath)
  {
    throw std::runtime_error("HOME environment variable not found.");
  }

  std::string currentDataTime = this->dataPtr->fileConfig.GetCurrentDateTime();
  std::string commsAnalysisFilePath = std::string(userDirPath) + "/ws/src/gazebo/comms_analysis/" + currentDataTime + ".csv";
  std::vector<std::string> analysis_header_contents = {
    "time[s]",
    "dstName",
    "dstX[m]",
    "dstY[m]",
    "dstZ[m]",
    "srcName",
    "srcX[m]",
    "srcY[m]",
    "srcZ[m]",
    "txAnttennaNo",
    "rxAnttennaNo",
    "RSSI[dBm]",
    "SINR[dB]",
    "-39dBm",
    "-45dBm",
    "-49dBm",
    "-51dBm",
    "-55dBm",
    "-58dBm",
    "-61dBm",
    "TotalDataTransmittion[Gb]",
    "TotalDataTransmittion[GB]"
  };

  this->dataPtr->writing_file.open(commsAnalysisFilePath, std::ios::out);
  for(size_t analysis_index=0; analysis_index < analysis_header_contents.size(); ++analysis_index)
  {
    if(analysis_index != 0)
      this->dataPtr->writing_file << ",";
    this->dataPtr->writing_file << analysis_header_contents[analysis_index];
  }
    
  
  if (_sdf->HasElement("range_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("range_config");

    this->dataPtr->rangeConfig.maxRange =
      elem->Get<double>("max_range", this->dataPtr->rangeConfig.maxRange).first;

    this->dataPtr->rangeConfig.fadingExponent =
      elem->Get<double>("fading_exponent",
        this->dataPtr->rangeConfig.fadingExponent).first;

    this->dataPtr->rangeConfig.l0 =
      elem->Get<double>("l0", this->dataPtr->rangeConfig.l0).first;

    this->dataPtr->rangeConfig.sigma =
      elem->Get<double>("sigma", this->dataPtr->rangeConfig.sigma).first;
  }

  if (_sdf->HasElement("radio_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("radio_config");

    this->dataPtr->radioConfig.centerFrequency =
      elem->Get<double>("center_frequency", this->dataPtr->radioConfig.centerFrequency).first;
    
    this->dataPtr->radioConfig.capacity =
      elem->Get<double>("capacity", this->dataPtr->radioConfig.capacity).first;

    this->dataPtr->radioConfig.txPower =
      elem->Get<double>("tx_power", this->dataPtr->radioConfig.txPower).first;

    this->dataPtr->radioConfig.modulation =
      elem->Get<std::string>("modulation",
        this->dataPtr->radioConfig.modulation).first;

    this->dataPtr->radioConfig.thermalNoiseDensity =
      elem->Get<double>("thermal_noise_density",
        this->dataPtr->radioConfig.thermalNoiseDensity).first;
    
    this->dataPtr->radioConfig.associateDuration = 
      elem->Get<double>("associate_duration",
        this->dataPtr->radioConfig.associateDuration).first;    

    this->dataPtr->radioConfig.powerThresholdForAssociation =
      elem->Get<double>("power_threshold_for_association",
          this->dataPtr->radioConfig.powerThresholdForAssociation).first;
    
    this->dataPtr->radioConfig.beaconInterval =
      elem->Get<double>("beacon_interval",
          this->dataPtr->radioConfig.beaconInterval).first;

    this->dataPtr->radioConfig.plotInterval =
      elem->Get<double>("plot_interval",
          this->dataPtr->radioConfig.plotInterval).first;
    
    // if (this->dataPtr->radioConfig.plotInterval < this->dataPtr->radioConfig.beaconInterval)
    // {
    //   gzwarn << "Plot interval (" << this->dataPtr->radioConfig.plotInterval << ") is changed to beacon interval (" <<  this->dataPtr->radioConfig.beaconInterval << ")";
    //   this->dataPtr->radioConfig.plotInterval = this->dataPtr->radioConfig.beaconInterval; 
    // }
      
    this->dataPtr->radioConfig.switchingDelay =
      elem->Get<double>("switching_delay",
          this->dataPtr->radioConfig.switchingDelay).first;

    this->dataPtr->radioConfig.isScheduled =
      elem->Get<bool>("is_scheduled",
          this->dataPtr->radioConfig.isScheduled).first;
    
    gzdbg << "isScheduled: " << this->dataPtr->radioConfig.isScheduled << std::endl;
    

    std::string antennaGainDirPath = std::string(userDirPath) + "/ws/src/gazebo/antenna_gains/";
    this->dataPtr->radioConfig.ePlaneFileName =
      elem->Get<std::string>("e_plane",
          this->dataPtr->radioConfig.ePlaneFileName).first;
    std::string ePlaneFilePath = antennaGainDirPath + this->dataPtr->radioConfig.ePlaneFileName;
    this->dataPtr->radioConfig.ePlane = this->dataPtr->FileToArray(ePlaneFilePath);
    double maxGainEPlane = -std::numeric_limits<double>::infinity();
    for (const auto& row : this->dataPtr->radioConfig.ePlane)
    {
      maxGainEPlane = std::max(maxGainEPlane, row[1]);
    }

    std::string hPlaneFilePath = antennaGainDirPath + this->dataPtr->radioConfig.hPlaneFileName;
    this->dataPtr->radioConfig.hPlaneFileName =
      elem->Get<std::string>("h_plane",
          this->dataPtr->radioConfig.hPlaneFileName).first;
    this->dataPtr->radioConfig.hPlane = this->dataPtr->FileToArray(hPlaneFilePath);
    double maxGainHPlane = -std::numeric_limits<double>::infinity();
    for (const auto& row : this->dataPtr->radioConfig.hPlane)
    {
      maxGainHPlane = std::max(maxGainHPlane, row[1]);
    }

    // 最大のアンテナ利得は各面の最大利得を足して2で割ったものとする
    this->dataPtr->radioConfig.maxAntennaGain = (maxGainHPlane + maxGainEPlane)/2.; // 各面の最大値の平均

    // 減衰の下限値
    // this->dataPtr->radioConfig.maxAntennaAttenuation = 30.
  }

  gzdbg << "Range configuration:" << std::endl
         << this->dataPtr->rangeConfig << std::endl;

  gzdbg << "Radio configuration:" << std::endl
         << this->dataPtr->radioConfig << std::endl;
}

//////////////////////////////////////////////////
void RFComms_custom::Step(
      const UpdateInfo &_info,
      const comms::Registry &_currentRegistry,
      comms::Registry &_newRegistry,
      EntityComponentManager &_ecm)
{
  // 1ステップ時間を取得する（結果の出力はpublisher.ccの送信間隔にも依存するが、この送信間隔が1ステップ時間より短くなると処理が追いつかなくなるので注意）
  std::chrono::steady_clock::duration timestep = _info.dt;
  double timestepInSeconds = std::chrono::duration<double>(timestep).count();
  if (timestepInSeconds > this->dataPtr->radioConfig.beaconInterval)
    gzerr << "Timestep is longer than beacon interval." << std::endl;

  for (auto & [address, content] : _currentRegistry)
  {
    // エンティティの関連付けが必要な場合
    // Associate entity if needed.
    if (content.entity == kNullEntity)
    {
      // エンティティの関連付け処理
      auto entities = sim::entitiesFromScopedName(content.modelName, _ecm);
      if (entities.empty())
        continue;

      auto entityId = *(entities.begin());
      if (entityId == kNullEntity)
        continue;

      _newRegistry[address].entity = entityId;

      // ワールドポーズコンポーネントの有効化
      enableComponent<components::WorldPose>(_ecm, entityId);
    }
    else
    {
      // ラジオ状態の更新
      const auto kPose = sim::worldPose(content.entity, _ecm);
      this->dataPtr->radioStates[address].pose = kPose;
      this->dataPtr->radioStates[address].antennaPos = kPose.Pos();
      this->dataPtr->radioStates[address].antennaRot = kPose.Rot();
      this->dataPtr->radioStates[address].timeStamp =
        std::chrono::duration<double>(_info.simTime).count();
      this->dataPtr->radioStates[address].name = content.modelName;
    }
    
  }

  for (auto & [address, content] : _currentRegistry)
  {
    // アウトバウンドメッセージキューへの参照
    auto &outbound = content.outboundMsgs;
    
    // 送信元アドレスがロボットにアタッチされている必要がある
    auto itSrc = this->dataPtr->radioStates.find(address);

    // 送信元アドレスが見つかった場合

    if (itSrc != this->dataPtr->radioStates.end())
    {
      // アウトバウンドメッセージの処理
      for (const auto &msg : outbound)
      {
        std::string srcAddress = itSrc->first;
        std::string srcAddressBase = srcAddress.substr(0, srcAddress.length()-1);
        
        // 送信先アドレスがロボットにアタッチされている必要がある
        auto itDst = this->dataPtr->radioStates.find(msg->dst_address());
        if (itDst == this->dataPtr->radioStates.end())
          continue;
        
        std::string dstAddress = itDst->first;
        std::string dstAddressBase = dstAddress.substr(0, dstAddress.length()-1);

        itSrc->second.scheduledAssociateWith = dstAddressBase + std::to_string(itSrc->second.scheduledAssociationIndex);
        itDst->second.scheduledAssociateWith = srcAddressBase + std::to_string(itDst->second.scheduledAssociationIndex);

        // Setupフェーズの経過時間を表示
        bool isSetup = itSrc->second.currentSetupWith == dstAddress && itDst->second.currentSetupWith == srcAddress;
        if (isSetup)
        {
          gzdbg << "Setup time: " << srcAddress << " <=== " << itSrc->second.timeStamp << "/" << itSrc->second.timeSetupCompleted << " ===> " << dstAddress << std::endl;
        }

        // Associatedフェーズの経過時間を表示
        bool isAssociated = itSrc->second.currentAssociateWith == dstAddress && itDst->second.currentAssociateWith == srcAddress;
        if (isAssociated)
        {
          gzdbg << "Associated time: " << srcAddress << " <=== " << itSrc->second.timeStamp << "/" << itSrc->second.timeout << " ===> " << dstAddress << std::endl;
        }
        
        // RSSIの算出
        int NumberOfAntennas = this->dataPtr->radioConfig.NumberOfAntenna;
        std::vector<double> rssi(NumberOfAntennas * NumberOfAntennas);
        rssi = this->dataPtr->MeasureRssi(itSrc->second, itDst->second);
        int ReceivingAntenna = this->dataPtr->judgmentOfUseAntenna(itSrc->second, itDst->second);
        int TransmissionAntenna = NumberOfAntennas - 1 - ReceivingAntenna;

        // gzwarn << "RSSI: " << srcAddress << " <===> " << dstAddress << " -> " << rssi << " dBm" << std::endl;

        // Setupフェーズ、Associatedフェーズのいずれでもない場合、ビーコンの送信間隔で処理を行う
        if (!isSetup && !isAssociated)
        {
          // ビーコン送信感覚の処理
          if (itSrc->second.timeStamp >= itSrc->second.scheduledBeaconTransmittionTime)
            itSrc->second.scheduledBeaconTransmittionTime = itSrc->second.timeStamp + this->dataPtr->radioConfig.beaconInterval;
          else
            continue;
        }

        // 他のデバイスと接続中でない場合のみ処理を行う
        bool isSrcBusy = (itSrc->second.currentSetupWith != "" && itSrc->second.currentSetupWith != dstAddress) ||
                         (itSrc->second.currentAssociateWith != "" && itSrc->second.currentAssociateWith != dstAddress);
        bool isDstBusy = (itDst->second.currentSetupWith != "" && itDst->second.currentSetupWith != srcAddress) ||
                         (itDst->second.currentAssociateWith != "" && itDst->second.currentAssociateWith != srcAddress);
        if (isSrcBusy || isDstBusy)
          continue;
        
        // Associationの要求が可能な場合
        bool isCommsAnalysisPlottable = (itDst->second.scheduledAssociateWith == srcAddress || itDst->second.currentSetupWith == srcAddress || itDst->second.currentAssociateWith == srcAddress) &&
                                        itDst->second.timeStamp > itDst->second.scheduledPlotTime;
        if (isCommsAnalysisPlottable)
        {
          itDst->second.scheduledPlotTime = itDst->second.timeStamp + this->dataPtr->radioConfig.plotInterval; 
          gz::math::Pose3 dstPose = itDst->second.pose;
          this->dataPtr->writing_file << std::endl 
                                      << itDst->second.timeStamp << ","
                                      << dstAddress << ","
                                      << dstPose.Pos().X() << ","
                                      << dstPose.Pos().Y() << ","
                                      << dstPose.Pos().Z();
          
          // for (auto pair : itDst->second.rssi)
          // {
          //   gzwarn << pair.first << ": " << pair.second << " dBm" << std::endl;
          // }
        }

        // Associatedフェーズかつ、タイムアウトもしくはRSSIが閾値を下回った場合、Disassociationを行う
        bool isDisassociation = isAssociated && (rssi[ReceivingAntenna * NumberOfAntennas + TransmissionAntenna] < this->dataPtr->radioConfig.powerThresholdForAssociation 
        // || itSrc->second.timeStamp > itSrc->second.timeout
        );
        if (isDisassociation)
        {
          // Associationの解除
          itSrc->second.currentAssociateWith = "";
          itDst->second.currentAssociateWith = "";

          // Associatedフェーズの時間を算出
          double timeAssociated = itSrc->second.timeStamp - itSrc->second.timeStartAssociation;
          std::cout << srcAddress << " was disassociated with " << dstAddress << " at " << itSrc->second.timeStamp << " s (for " << timeAssociated << " s) " << std::endl;

          // 次の接続先を定めている場合はその設定を行う
          if (this->dataPtr->radioConfig.isScheduled)
          {
            itSrc->second.scheduledAssociationIndex += 1;
            auto itScheduledSrc = this->dataPtr->radioStates.find(dstAddressBase + std::to_string(itSrc->second.scheduledAssociationIndex));
            if (itScheduledSrc == this->dataPtr->radioStates.end())
              itSrc->second.scheduledAssociationIndex = 0;
            std::cout << srcAddress << " will associate with " << dstAddressBase << itSrc->second.scheduledAssociationIndex << std::endl;

            itDst->second.scheduledAssociationIndex += 1;
            auto itScheduledDst = this->dataPtr->radioStates.find(srcAddressBase + std::to_string(itDst->second.scheduledAssociationIndex));
            if (itScheduledDst == this->dataPtr->radioStates.end())
              itDst->second.scheduledAssociationIndex = 0;
            std::cout << dstAddress << " will associate with " << srcAddressBase << itDst->second.scheduledAssociationIndex << std::endl;
            }
        }

        // SetupフェーズかつSetup完了時刻を超過している場合、Association Requestに応答し、Associatedフェーズに移行する
        bool isAssociationRespondable = isSetup && itSrc->second.timeStamp > itSrc->second.timeSetupCompleted;

        bool isAssociationRequestable = !isAssociated && 
                                        !isSetup &&
                                        (!this->dataPtr->radioConfig.isScheduled || itSrc->second.scheduledAssociateWith == dstAddress) &&
                                        (!this->dataPtr->radioConfig.isScheduled || itDst->second.scheduledAssociateWith == srcAddress) &&
                                        rssi[ReceivingAntenna * NumberOfAntennas + TransmissionAntenna] > this->dataPtr->radioConfig.powerThresholdForAssociation;
        
        if (isAssociationRequestable)
        {
          // Setupの設定
          itSrc->second.currentSetupWith = dstAddress;
          itDst->second.currentSetupWith = srcAddress;

          // Setup完了時刻の設定
          itSrc->second.timeSetupCompleted = itSrc->second.timeStamp + this->dataPtr->radioConfig.switchingDelay;
          std::cout << srcAddress << " is being setup with " << dstAddress << " from " << itSrc->second.timeStamp << " to " << itSrc->second.timeSetupCompleted << std::endl;
        }

        // SetupフェーズかつSetup完了時刻を超過している場合、Association Requestに応答し、Associatedフェーズに移行する
        if (isAssociationRespondable)
        {
          // Setupの解除
          itSrc->second.currentSetupWith = "";
          itDst->second.currentSetupWith = "";

          // Associationの設定
          itSrc->second.currentAssociateWith = dstAddress;
          itDst->second.currentAssociateWith = srcAddress;

          // Timeoutの設定
          // double associateDuration = this->dataPtr->GetAssociateDuration(srcAddress);
          itSrc->second.timeout = itSrc->second.timeStamp;
          //  + associateDuration;

          // Association開始時刻の設定
          itSrc->second.timeStartAssociation = itSrc->second.timeStamp;
          std::cout << srcAddress << " will be associated with " << dstAddress << " from " << itSrc->second.timeStamp << " to " << itSrc->second.timeout << std::endl;
        }

        // Associatedフェーズかつ、RSSIが発散していない
        if (isAssociated && std::isfinite(ReceivingAntenna * NumberOfAntennas + TransmissionAntenna))
        {
          // タイムスタンプの間隔を算出
          // タイムスタンプの間隔を算出
          double timeStampDuration = itDst->second.timeStamp - itDst->second.previousTimeStamp; // publishの間隔が短くなるほど小さくなる
          
          // SINRの算出
          double sinr = this->dataPtr->RssiToSinr(srcAddress, itDst->second);
          gzdbg << "SINR: " << srcAddress << " <===> " << dstAddress << " -> " << sinr << " dB" << std::endl;

          // Throughputの算出
          double throughputInGbps = this->dataPtr->RssiToThrouputInGbps(rssi[ReceivingAntenna * NumberOfAntennas + TransmissionAntenna]);
          gzdbg << "Throuput: " << srcAddress << " <===> " << dstAddress << " -> " << throughputInGbps << " Gbps" << std::endl;

          // 累積データ転送量の算出
          double dataTransferredInTimestep = throughputInGbps*timeStampDuration;
          itDst->second.totalDataTransferred += dataTransferredInTimestep;
          gzdbg << "Total data transferred: " << srcAddress << " <===> " << dstAddress << " -> " << itDst->second.totalDataTransferred << " Gb" << std::endl;

          // 累積データ転送時間の算出
          // for (auto &pair: itDst->second.totalTimeAndDataTransmittionInItsLevels)
          // {
          //   if (rssi > pair.first)
          //   {
          //     pair.second += timeStampDuration;
          //     break;
          //   }
          // }

          this->dataPtr->RssiToTotalTimeAndDataTransmittionInItsLevels(rssi[ReceivingAntenna * NumberOfAntennas + TransmissionAntenna],
                                                                       timeStampDuration,
                                                                       itDst->second.totalTimeAndDataTransmittionInItsLevels);
          double totalDataTransmittionInGb = this->dataPtr->TotalTimeAndDataTransmittionInItsLevelsToTotalDataTransmittion(itDst->second.totalTimeAndDataTransmittionInItsLevels);
          double totalDataTransmittionInGB = totalDataTransmittionInGb/8.;

          if (isCommsAnalysisPlottable)
          {
            gz::math::Pose3 srcPose = itSrc->second.pose;       
            this->dataPtr->writing_file << ","
                                        << srcAddress << ","
                                        << srcPose.Pos().X() << ","
                                        << srcPose.Pos().Y() << ","
                                        << srcPose.Pos().Z() << ","
                                        << ReceivingAntenna << ","
                                        << TransmissionAntenna << ","
                                        << rssi[ReceivingAntenna * NumberOfAntennas + TransmissionAntenna] << ","
                                        << sinr << ",";
            for (auto &totalTimeAndDataTransmittionInItsLevel: itDst->second.totalTimeAndDataTransmittionInItsLevels)
            {
              this->dataPtr->writing_file << totalTimeAndDataTransmittionInItsLevel.at(2) << ",";
            }
            this->dataPtr->writing_file << totalDataTransmittionInGb << ","
                                        << totalDataTransmittionInGB;

          }

          // 送信メッセージをターミナルに表示
          // auto inboundMsg = std::make_shared<gz::msgs::Dataframe>(*msg);
          // auto *commsAnalysisPtr = inboundMsg->mutable_header()->add_data();
          // commsAnalysisPtr->set_key("RSSI/SINR: " + address);
          // commsAnalysisPtr->add_value(std::to_string(rssi) + "/" + std::to_string(sinr));
          // _newRegistry[msg->dst_address()].inboundMsgs.push_back(inboundMsg);
        }
        
        // SetupフェーズまたはAssociatedフェーズにおける累積時間算出のためにpreviousTimeStampを算出
        bool isTimeStampRenewable = isSetup || isAssociated;
        if (isTimeStampRenewable)
          itDst->second.previousTimeStamp = itDst->second.timeStamp;
      }
    }
    // Clear the outbound queue.
    _newRegistry[address].outboundMsgs.clear();
  }
}

GZ_ADD_PLUGIN(RFComms_custom,
              gz::sim::System,
              comms::ICommsModel::ISystemConfigure,
              comms::ICommsModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RFComms_custom,
                    "gz::sim::systems::RFComms_custom")