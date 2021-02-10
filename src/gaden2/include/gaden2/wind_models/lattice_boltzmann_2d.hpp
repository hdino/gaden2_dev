#ifndef GADEN2_WIND_MODELS_LATTICE_BOLTZMANN_2D_HPP_INCLUDED
#define GADEN2_WIND_MODELS_LATTICE_BOLTZMANN_2D_HPP_INCLUDED

#include "../logger.hpp"
#include "wind_model_base.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

#include <memory>

namespace gaden2 {

namespace environment {
class EnvironmentModelBase;
} // namespace environment

namespace wind_model {

class LatticeBoltzmann2D : public WindModelBase
{
public:
    static constexpr double DEFAULT_GRID_CELL_SIZE = 1.0; // [m]

    LatticeBoltzmann2D(const std::shared_ptr<environment::EnvironmentModelBase> &environment_model,
                       double grid_cell_size = DEFAULT_GRID_CELL_SIZE, // [m]
                       rl::Logger parent_logger = getStandardLogger());

    ~LatticeBoltzmann2D();

    Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position);

    Eigen::Vector3d getEnvironmentMin() const;
    Eigen::Vector3d getEnvironmentMax() const;

private:
    void performIncrement(double time_step, double total_sim_time);
};

}} // namespace gaden2::wind_model

#endif // GADEN2_WIND_MODELS_LATTICE_BOLTZMANN_2D_HPP_INCLUDED
