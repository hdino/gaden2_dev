#ifndef GADEN2_WIND_MODELS_WIND_MODEL_BASE_HPP_INCLUDED
#define GADEN2_WIND_MODELS_WIND_MODEL_BASE_HPP_INCLUDED

#include "../simulation_element.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2::wind_model {

class WindModelBase : public SimulationElement
{
public:
    WindModelBase(rl::Logger &parent_logger)
        : logger(parent_logger.getChild("WindModel"))
    {}

    virtual ~WindModelBase() {}

    virtual Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position) = 0;

    virtual Eigen::Vector3d getEnvironmentMin() const = 0;
    virtual Eigen::Vector3d getEnvironmentMax() const = 0;

protected:
    virtual void performIncrement(double time_step, double total_sim_time)
    {
        (void)time_step;
        (void)total_sim_time;
    }

    rl::Logger logger;
};

} // namespace gaden2::wind_model

#endif // GADEN2_WIND_MODELS_WIND_MODEL_BASE_HPP_INCLUDED
