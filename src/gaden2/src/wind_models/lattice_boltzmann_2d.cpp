#include <gaden2/environment_model.hpp>
#include <gaden2/wind_models/lattice_boltzmann_2d.hpp>

namespace gaden2::wind_model {

LatticeBoltzmann2D::LatticeBoltzmann2D(const std::shared_ptr<EnvironmentModel> &environment_model,
                                       double grid_cell_size,
                                       rl::Logger parent_logger)
    : WindModelBase(parent_logger)
{
    //
}

LatticeBoltzmann2D::~LatticeBoltzmann2D()
{
    //
}

Eigen::Vector3d LatticeBoltzmann2D::getWindVelocityAt(const Eigen::Vector3d &position)
{
    return Eigen::Vector3d(0, 0, 0);
}

Eigen::Vector3d LatticeBoltzmann2D::getEnvironmentMin() const
{
    //
}

Eigen::Vector3d LatticeBoltzmann2D::getEnvironmentMax() const
{
    //
}

void LatticeBoltzmann2D::performIncrement(double time_step, double total_sim_time)
{
    //
}

} // namespace gaden2::wind_model
