#ifndef GADEN2_WIND_MODELS_FARRELL_HPP_INCLUDED
#define GADEN2_WIND_MODELS_FARRELL_HPP_INCLUDED

#include "../logger.hpp"
#include "wind_model_base.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

#include <memory>
#include <string>

namespace gaden2 {

namespace environment {
class EnvironmentModelBase;
} // namespace environment

namespace wind_model {

class FarrellColouredNoiseGenerator;

struct FarrellsWindModelConfiguration
{
    double u0;              // [m/s] mean wind velocity in x-direction
    double v0;              // [m/s] mean wind velocity in y-direction
    double k_x;             // [m2/s] diffusivity term in x-direction, recommended range: [1, 30]
    double k_y;             // [m2/s] diffusivity term in y-direction, recommended range: [1, 30]
    double noise_gain;      // [] Input gain constant for boundary condition noise generation.
    double noise_damp;      // [] Damping ratio for boundary condition noise generation.
    double noise_bandwidth; // [] Bandwidth for boundary condition noise generation.
};
std::string toString(const FarrellsWindModelConfiguration &config, size_t indention = 0);

class Farrell : public WindModelBase
{
public:
    // define default variables, so that pybind11 can also use them
    static constexpr double DEFAULT_GRID_CELL_SIZE = 5.0;   // [m] recommended range: [5, 10]
    static constexpr double DEFAULT_U0 = 1.0;               // [m/s] mean wind velocity in x-direction
    static constexpr double DEFAULT_V0 = 0.0;               // [m/s] mean wind velocity in y-direction
    static constexpr double DEFAULT_KX = 10.0;              // [m2/s] diffusivity term in x-direction, recommended range: [1, 30]
    static constexpr double DEFAULT_KY = 10.0;              // [m2/s] diffusivity term in y-direction, recommended range: [1, 30]
    static constexpr double DEFAULT_NOISE_GAIN = 2.0;       // [] Input gain constant for boundary condition noise generation.
    static constexpr double DEFAULT_NOISE_DAMP = 0.1;       // [] Damping ratio for boundary condition noise generation.
    static constexpr double DEFAULT_NOISE_BANDWIDTH = 0.2;  // [] Bandwidth for boundary condition noise generation.

    Farrell(const std::shared_ptr<environment::EnvironmentModelBase> &environment_model,
            double grid_cell_size = DEFAULT_GRID_CELL_SIZE, // [m] recommended range: [5, 10]
            double u0 = DEFAULT_U0, // [m/s] mean wind velocity in x-direction
            double v0 = DEFAULT_V0, // [m/s] mean wind velocity in y-direction
            double kx = DEFAULT_KX, // [m2/s] diffusivity term in x-direction, recommended range: [1, 30]
            double ky = DEFAULT_KY, // [m2/s] diffusivity term in y-direction, recommended range: [1, 30]
            double noise_gain = DEFAULT_NOISE_GAIN, // [] Input gain constant for boundary condition noise generation.
            double noise_damp = DEFAULT_NOISE_DAMP, // [] Damping ratio for boundary condition noise generation.
            double noise_bandwidth = DEFAULT_NOISE_BANDWIDTH, // [] Bandwidth for boundary condition noise generation.
            rl::Logger parent_logger = getStandardLogger());
    ~Farrell();

    void startRecord(const std::string &file);
    void stopRecord();

    Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position);

    Eigen::Vector3d getEnvironmentMin() const;
    Eigen::Vector3d getEnvironmentMax() const;

private:
    void performIncrement(double time_step, double total_sim_time);

    void applyBoundaryConditions(double dt);
    std::tuple<Eigen::ArrayXXd, Eigen::ArrayXXd> getCentred1stDifferences(const Eigen::ArrayXXd &f);
    std::tuple<Eigen::ArrayXXd, Eigen::ArrayXXd> getCentred2ndDifferences(const Eigen::ArrayXXd &f);

    FarrellsWindModelConfiguration config_;
    std::unique_ptr<FarrellColouredNoiseGenerator> noise_generator_;

    Eigen::Vector3d environment_min_;
    Eigen::Vector3d environment_max_;

    double dx_, dy_; // grid point spacing in x/y direction
    Eigen::Array3d delta_grid; // (dx, dy, dz), dz has no meaning

    Eigen::ArrayXXd u_, v_; // wind velocity field in x/y direction

    Eigen::Array<double, 8, 1> corner_means_;
    Eigen::ArrayXd ramp_x_, ramp_y_;
};

}} // namespace gaden2::wind_model

#endif // GADEN2_WIND_MODELS_FARRELL_HPP_INCLUDED
