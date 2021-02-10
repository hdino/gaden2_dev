#ifndef GADEN2_GAS_DISPERSION_MODEL_HPP_INCLUDED
#define GADEN2_GAS_DISPERSION_MODEL_HPP_INCLUDED

#include "simulation_element.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2 {

class GasDispersionModel : public SimulationElement
{
public:
    GasDispersionModel(rl::Logger &parent_logger)
        : logger(parent_logger.getChild("GasDispersionModel"))
    {}

    virtual ~GasDispersionModel() {}

    virtual double getConcentrationAt(const Eigen::Vector3d &position) = 0; // returns [ppm]

protected:
    rl::Logger logger;
};

} // namespace gaden2

#endif // GADEN2_GAS_DISPERSION_MODEL_HPP_INCLUDED
