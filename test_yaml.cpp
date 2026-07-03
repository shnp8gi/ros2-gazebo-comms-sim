#include <iostream>
#include <yaml-cpp/yaml.h>

int main() {
    YAML::Node node = YAML::Load("rssi_threshold: -95");
    try {
        double val = node["rssi_threshold"].as<double>();
        std::cout << "Success: " << val << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    return 0;
}
