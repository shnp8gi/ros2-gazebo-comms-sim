#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace comms_sim
{
    /**
     * IRateModel (本番仕様 §4.3)
     * --------------------------
     * 責務: RSSI [dBm] → 到達可能レート [Gbps] の写像のみ (プラグインのデータ会計は Gbps 前提)。
     * CONNECTED 判定閾値 (MinRssiDbm) と飽和上限 (MaxRssiDbm) も写像の
     * 一部として本インターフェースが提供する (呼び出し側の rssi_min/rssi_max)。
     */
    class IRateModel {
    public:
        virtual ~IRateModel() = default;
        virtual double RateGbps(double rssi_dbm) const = 0;
        virtual double MinRssiDbm() const = 0;
        virtual double MaxRssiDbm() const = 0;
    };

    /**
     * McsTableRateModel
     * -----------------
     * 既存の MCS テーブル (RSSI 閾値 → スループット段階) 互換の写像。既定。
     * 挙動・既定テーブルは旧 CommsCalculator 内蔵実装と同一。
     */
    class McsTableRateModel : public IRateModel {
    public:
        explicit McsTableRateModel(const std::string& csv_path = "") {
            if (!csv_path.empty()) {
                this->LoadTable(csv_path);
            }
            if (this->rssi_.empty()) {
                this->rssi_ = {-61, -58, -55, -51, -45, -39};
                this->throughput_ = {2.5813, 3.2853, 5.1627, 6.5707, 9.856, 13.1413};
            }
        }

        double RateGbps(double rssi_dbm) const override {
            if (rssi_dbm <= this->rssi_.front()) return 0.0;
            if (rssi_dbm >= this->rssi_.back()) return this->throughput_.back();
            auto it = std::upper_bound(this->rssi_.begin(), this->rssi_.end(), rssi_dbm);
            std::size_t idx = std::distance(this->rssi_.begin(), it) - 1;
            return this->throughput_[idx];
        }

        double MinRssiDbm() const override { return this->rssi_.front(); }
        double MaxRssiDbm() const override { return this->rssi_.back(); }

    private:
        void LoadTable(const std::string& path) {
            std::ifstream file(path);
            if (!file.is_open()) {
                std::cerr << "Failed to open MCS table: " << path << std::endl;
                return;
            }
            std::vector<std::pair<double, double>> data;
            std::string line;
            while (std::getline(file, line)) {
                line.erase(0, line.find_first_not_of(" \r\n\t"));
                if (line.empty() || line[0] == '#' ||
                    (line.size() >= 2 && line[0] == '/' && line[1] == '/')) {
                    continue;
                }
                std::stringstream ss(line);
                std::string t1, t2;
                if (std::getline(ss, t1, ',') && std::getline(ss, t2, ',')) {
                    try {
                        data.emplace_back(std::stod(t1), std::stod(t2));
                    } catch (...) {
                        // ignore
                    }
                }
            }
            if (data.empty()) return;
            std::sort(data.begin(), data.end());
            for (const auto& p : data) {
                this->rssi_.push_back(p.first);
                this->throughput_.push_back(p.second);
            }
        }

        std::vector<double> rssi_, throughput_;
    };

    /**
     * ShannonRateModel
     * ----------------
     * 雑音床を基準線とした連続レート写像: rate = η·BW·log2(1 + SNR)。
     * SNR = RSSI − noise_floor。snr_min_db 未満は 0 (= CONNECTED 不能)、
     * snr_cap_db で飽和 (実装上の変調上限相当)。
     * 60GHz / BW 100MHz / 雑音床 −95dBm が本番初期値【実測で校正】。
     */
    class ShannonRateModel : public IRateModel {
    public:
        ShannonRateModel(double bandwidth_hz, double noise_floor_dbm,
                         double efficiency = 1.0, double snr_min_db = 0.0,
                         double snr_cap_db = 50.0)
            : bandwidth_hz(bandwidth_hz), noise_floor_dbm(noise_floor_dbm),
              efficiency(efficiency), snr_min_db(snr_min_db),
              snr_cap_db(snr_cap_db) {}

        double RateGbps(double rssi_dbm) const override {
            double snr_db = rssi_dbm - this->noise_floor_dbm;
            if (snr_db < this->snr_min_db) return 0.0;
            snr_db = std::min(snr_db, this->snr_cap_db);
            double snr = std::pow(10.0, snr_db / 10.0);
            return this->efficiency * this->bandwidth_hz * std::log2(1.0 + snr) / 1.0e9;
        }

        double MinRssiDbm() const override { return this->noise_floor_dbm + this->snr_min_db; }
        double MaxRssiDbm() const override { return this->noise_floor_dbm + this->snr_cap_db; }

    private:
        double bandwidth_hz;
        double noise_floor_dbm;
        double efficiency;
        double snr_min_db;
        double snr_cap_db;
    };

    /**
     * RateModelFactory
     * ----------------
     * 責務: yaml `rate_model:` セクションからの組立のみ。
     *   rate_model: { type: shannon, bandwidth_hz: 100.0e6, noise_floor_dbm: -95.0,
     *                 efficiency: 1.0, snr_min_db: 0.0, snr_cap_db: 50.0 }
     * セクションが無い/type: mcs のときは MCS テーブル (旧構成と完全互換)。
     * mcs_table_path の解決 (パス展開) は呼び出し側の責務。
     */
    class RateModelFactory {
    public:
        static std::unique_ptr<IRateModel> Create(const YAML::Node& rate_model,
                                                  const std::string& resolved_mcs_path) {
            if (rate_model && rate_model["type"].as<std::string>("mcs") == "shannon") {
                return std::make_unique<ShannonRateModel>(
                    rate_model["bandwidth_hz"].as<double>(100.0e6),
                    rate_model["noise_floor_dbm"].as<double>(-95.0),
                    rate_model["efficiency"].as<double>(1.0),
                    rate_model["snr_min_db"].as<double>(0.0),
                    rate_model["snr_cap_db"].as<double>(50.0));
            }
            return std::make_unique<McsTableRateModel>(resolved_mcs_path);
        }
    };
} // namespace comms_sim
