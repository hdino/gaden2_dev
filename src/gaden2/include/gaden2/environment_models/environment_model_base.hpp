#ifndef GADEN2_ENVIRONMENT_MODELS_ENVIRONMENT_MODEL_BASE_HPP_INCLUDED
#define GADEN2_ENVIRONMENT_MODELS_ENVIRONMENT_MODEL_BASE_HPP_INCLUDED

#include <gaden2/simulation_element.hpp>

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2::environment {

enum class Occupancy : int32_t { Free = 0, Occupied = 1, Outlet = 2, OutOfWorld = 3 };

class EnvironmentModelBase : public SimulationElement
{
public:
    EnvironmentModelBase(rl::Logger parent_logger)
        : logger(parent_logger.getChild("EnvironmentModel"))
    {}

    virtual ~EnvironmentModelBase() {}

    virtual Eigen::Vector3d getEnvironmentMin() const = 0;
    virtual Eigen::Vector3d getEnvironmentMax() const = 0;

    virtual Occupancy getOccupancy(const Eigen::Vector3d &p) const = 0;

protected:
    virtual void performIncrement(double time_step, double total_sim_time)
    {
        (void)time_step;
        (void)total_sim_time;
    }

    rl::Logger logger;
};

} // namespace gaden2::environment

#endif // GADEN2_ENVIRONMENT_MODELS_ENVIRONMENT_MODEL_BASE_HPP_INCLUDED
