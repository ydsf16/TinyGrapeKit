#pragma once
#include <VWO/State.h>

namespace VWO {

void AugmentState(const double timestamp, const long int frame_id, State* state);

}  // namespace VWO