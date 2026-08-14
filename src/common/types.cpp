#include "common/types.h"

namespace vehicle {

const char* vehicleClassName(int cls) {
  switch (cls) {
    case kClassCar: return "car";
    case kClassMotorbike: return "motorbike";
    case kClassTruck: return "truck";
    case kClassBus: return "bus";
    default: return "unknown";
  }
}

}  // namespace vehicle
