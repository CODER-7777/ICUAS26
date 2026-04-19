#include <cstdlib>
#include <string>
#include <yaml-cpp/yaml.h>


int get_num_robots() {
    const char *env_p = std::getenv("NUM_ROBOTS");
    if (env_p) {
        return std::stoi(env_p);
    } else {
        return 5;
    }
}

std::string get_charging_file() {
    const char *env_p = std::getenv("CHARGING_FILE");
    if (env_p) {
        return std::string(env_p);
    } else {
        return "5";
    }
}

double get_comm_range() {
    const char *env_p = std::getenv("COMM_RANGE");
    if (env_p) {
        return std::stod(env_p);
    } else {
        return 70;
    }
}

std::string config_path = "/root/ros2_ws/src/icuas26_competition/config/";
YAML::Node config = YAML::LoadFile(config_path + get_charging_file());

std::vector<double> charging_area_upper_left() {
    return {
        config["charging_area"]["upper_left"][0].as<double>(), config["charging_area"]["upper_left"][1].as<double>()
    };
}

std::vector<double> charging_area_down_right() {
    return {
        config["charging_area"]["down_right"][0].as<double>(), config["charging_area"]["down_right"][1].as<double>()
    };
}


// int main(){
//    std::cout<<"NUM_ROBOTS: " << get_num_robots() << std::endl;
//    std::cout<<"CHARGING_FILE: " << get_charging_file() << std::endl;
//    std::cout<<"COMM_RANGE: " << get_comm_range() << std::endl;
//    std::cout<<"Charging Area Upper Left: " << charging_area_upper_left()[0] << ", " << charging_area_upper_left()[1] << std::endl;
//    std::cout<<"Charging Area Lower Right: " << charging_area_lower_right()[0] << ", " << charging_area_lower_right()[1] << std::endl;
// }
