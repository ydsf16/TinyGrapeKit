#pragma once

#include <memory>

#include <TGK/WheelProcessor/WheelPropagator.h>
#include <VWO/State.h>

namespace VWO {

class Propagator {
public:
    Propagator(const double kl, const double kr,const double b, const double noise_factor);

    void Propagate(const double begin_wl, const double begin_wr,
                   const double end_wl, const double end_wr,
                   State* state);
private:
    std::unique_ptr<TGK::WheelProcessor::WheelPropagator> wheel_propagator_;

    const double klkr_process_noise_ = 1e-18;
    const double b_process_noise_ = 1e-12;
};

}  // namespace VWO