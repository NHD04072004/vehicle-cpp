#include "business/violation/kinds.h"

#include "business/violation/constants.h"

namespace vehicle {
namespace business {
namespace violation {

const char* eventKindCode(EventKind k) {
  switch (k) {
    case EventKind::kNoHelmet: return kNoHelmet;
    case EventKind::kWrongLane: return kWrongLane;
    case EventKind::kWrongWay: return kWrongWay;
    default: return nullptr;  // kPlate không phải vi phạm
  }
}

const char* eventKindName(EventKind k) {
  switch (k) {
    case EventKind::kPlate: return "PLATE";
    case EventKind::kNoHelmet: return kNoHelmet;
    case EventKind::kWrongLane: return kWrongLane;
    case EventKind::kWrongWay: return kWrongWay;
    default: return "?";
  }
}

}  // namespace violation
}  // namespace business
}  // namespace vehicle
