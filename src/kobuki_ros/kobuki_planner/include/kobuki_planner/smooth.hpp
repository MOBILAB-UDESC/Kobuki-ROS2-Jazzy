#include "kobuki_planner/utils.hpp"
#include <iostream>
#include "Eigen/Dense"

using Eigen::Vector2d;

class SmoothPath {

    public:
        SmoothPath(const std::vector<std::tuple<float, float, float>> path_poses, float step_size);

        std::vector<std::tuple<float, float, float>> Darpa_curve();

    // private:

        std::vector<std::tuple<float, float, float>> path_poses;
        double wsmooth = 0.2;
        double csmooth = 0.2;
        double Kappa   = 45;
        double step_size;

        Vector2d smoothnessTerm(const Vector2d& xim2, const Vector2d& xim1, const Vector2d& xi, const Vector2d& xip1, const Vector2d& xip2);
        Vector2d curvatureTerm(const Vector2d& xim1, const Vector2d& xi, const Vector2d& xip1);

    private:
        Vector2d ort(const Vector2d& a, const Vector2d& b) {
            return a - b * a.dot(b) / (b.squaredNorm());
        }

        double clamp(double value, double min, double max) {
            return std::max(min, std::min(value, max));
        }
};