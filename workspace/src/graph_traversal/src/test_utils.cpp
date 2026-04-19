#include "utils.hpp"
#include <iostream>

int main(){
   std::cout<<"NUM_ROBOTS: " << get_num_robots() << std::endl;
   std::cout<<"CHARGING_FILE: " << get_charging_file() << std::endl;
   std::cout<<"COMM_RANGE: " << get_comm_range() << std::endl;
   std::cout<<"Charging Area Upper Left: " << charging_area_upper_left()[0] << ", " << charging_area_upper_left()[1] << std::endl;
   std::cout<<"Charging Area Lower Right: " << charging_area_down_right()[0] << ", " << charging_area_down_right()[1] << std::endl;
}

