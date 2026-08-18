#include "common/types.h"

#include <sstream>

namespace vehicle {
namespace {

// Token viết hoa → VehicleClass; -1 nếu không khớp loại xe nào.
int classFromToken(const std::string& token) {
  if (token == "CAR") return kClassCar;
  if (token == "MOTOBIKE") return kClassMotorbike;
  if (token == "TRUCK") return kClassTruck;
  if (token == "BUS") return kClassBus;
  return -1;
}

}  // namespace

const char* vehicleClassName(int cls) {
  switch (cls) {
    case kClassCar: return "car";
    case kClassMotorbike: return "motorbike";
    case kClassTruck: return "truck";
    case kClassBus: return "bus";
    default: return "unknown";
  }
}

std::set<int> laneAllowedClasses(const std::string& zone_name) {
  std::vector<std::string> tokens;
  std::stringstream ss(zone_name);
  std::string token;
  while (std::getline(ss, token, '_')) tokens.push_back(token);

  if (tokens.empty() || tokens.back() != "LANE") return {};

  std::set<int> allowed;
  for (size_t i = 0; i + 1 < tokens.size(); ++i) {
    const int cls = classFromToken(tokens[i]);
    if (cls >= 0) allowed.insert(cls);
  }
  return allowed;
}

}  // namespace vehicle
