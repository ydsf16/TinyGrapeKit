#pragma once

#include <VWO/State.h>

namespace VWO {

class PlaneUpdater {
public:
    struct Config {
        double plane_rot_noise = 0.01;
        double plane_trans_noise = 0.01;
    };

    PlaneUpdater(const Config& config);

    bool UpdateState(State* state);

private:
    Config config_;
};

}  // namespace VWO