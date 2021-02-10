#ifndef GADEN2_ENVIRONMENT_MODELS_PLANE_HPP_INCLUDED
#define GADEN2_ENVIRONMENT_MODELS_PLANE_HPP_INCLUDED

#include "environment_model_base.hpp"
#include <gaden2/logger.hpp>

#include <Eigen/Core>

#include <string>

namespace gaden2::environment {

class Plane : public EnvironmentModelBase
{
public:
    static constexpr double DEFAULT_X_MIN = -50.0;
    static constexpr double DEFAULT_X_MAX = 50.0;
    static constexpr double DEFAULT_Y_MIN = -25.0;
    static constexpr double DEFAULT_Y_MAX = 25.0;
    static constexpr double DEFAULT_Z_MAX = 50.0;

    Plane(double x_min = DEFAULT_X_MIN,
          double x_max = DEFAULT_X_MAX,
          double y_min = DEFAULT_Y_MIN,
          double y_max = DEFAULT_Y_MAX,
          double z_max = DEFAULT_Z_MAX,
          rl::Logger parent_logger = getStandardLogger());

    Plane(const std::string &file,
          rl::Logger parent_logger = getStandardLogger());

    ~Plane();

    void startRecord(const std::string &file);
    void stopRecord();

    Eigen::Vector3d getEnvironmentMin() const;
    Eigen::Vector3d getEnvironmentMax() const;

    Occupancy getOccupancy(const Eigen::Vector3d &p) const;

    Eigen::Vector3d getPlaneCenterCoordinates() const;
    Eigen::Vector3d getPlaneDimensions() const;

private:
    Eigen::Vector3d world_min_; // [m] minimum in world coordinates
    Eigen::Vector3d world_max_; // [m] maximum in world coordinates
    Eigen::Vector3d plane_min_; // [m]
    Eigen::Vector3d plane_max_; // [m]
};

} // namespace gaden2::environment

#endif // GADEN2_ENVIRONMENT_MODELS_PLANE_HPP_INCLUDED
