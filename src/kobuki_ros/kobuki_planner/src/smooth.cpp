#include "kobuki_planner/smooth.hpp"

SmoothPath::SmoothPath(const std::vector<std::tuple<float, float, float>> path_poses, float step_size)
    : path_poses(path_poses), step_size(step_size) {}

std::vector<std::tuple<float, float, float>> SmoothPath::Darpa_curve()
{
    int iterations       = 0;
    int max_iterations   = 500;
    int waypoints_number = this->path_poses.size();

    if (waypoints_number < 5) {
        return this->path_poses;
    }

    double total_weight = this->wsmooth + this->csmooth;

    while(iterations < max_iterations)
    {
        for (int i = 2; i < waypoints_number-2; ++i)
        {

            if (std::isnan(std::get<0>(this->path_poses[i + 2])))
                break;

            Vector2d xim2(std::get<0>(this->path_poses[i - 2]), std::get<1>(this->path_poses[i - 2]));
            Vector2d xim1(std::get<0>(this->path_poses[i - 1]), std::get<1>(this->path_poses[i - 1]));
            Vector2d xi  (std::get<0>(this->path_poses[i    ]), std::get<1>(this->path_poses[i    ]));
            Vector2d xip1(std::get<0>(this->path_poses[i + 1]), std::get<1>(this->path_poses[i + 1]));
            Vector2d xip2(std::get<0>(this->path_poses[i + 2]), std::get<1>(this->path_poses[i + 2]));

            Vector2d correction = {0.0, 0.0};

            correction = correction - this->smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            correction = correction - this->curvatureTerm(xim1, xi, xip1);

            float alpha = 0.1;
            xi = xi + alpha * correction / total_weight;
            this->path_poses[i] = std::make_tuple(xi.x(), xi.y(), std::get<2>(this->path_poses[i]));

            Vector2d Dxi = xi - xim1;
            double newTheta = std::atan2(Dxi.y(), Dxi.x());
            std::get<2>(this->path_poses[i - 1]) = newTheta;
        }

        iterations++;

    }

    return this->path_poses;

}

Vector2d SmoothPath::smoothnessTerm(const Vector2d& xim2, const Vector2d& xim1, const Vector2d& xi, const Vector2d& xip1, const Vector2d& xip2)
{
    return this->wsmooth * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

Vector2d SmoothPath::curvatureTerm(const Vector2d& xim1, const Vector2d& xi, const Vector2d& xip1)
{

    Vector2d gradient(0, 0), p1(0, 0), p2(0, 0);

    Vector2d Dxi = xi - xim1;
    Vector2d DxiP1 = xip1 - xi;
    double absDxi = Dxi.norm();
    double absDxiP1 = DxiP1.norm();

    if (absDxi > 0 && absDxiP1 > 0)
    {
        auto temp    = this->clamp(Dxi.dot(DxiP1) / (absDxi * absDxiP1), -1.0, 1.0);
        double Dphi  = std::acos(temp);
        double kappa = Dphi / absDxi;

        if (kappa <= this->Kappa)
        {
            return Vector2d(0.0, 0.0);
        }else{
            double absDxi1Inv     = 1 / absDxi;
            double PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi),2));
            double u              = -absDxi1Inv * PDphi_PcosDphi;

            p1 = this->ort(xi, -xip1) / (absDxi * absDxiP1);
            p2 = this->ort(xip1, xi) / (absDxi * absDxiP1);

            double s = Dphi / (absDxi * absDxi);

            Vector2d ki, kim1, kip1;

            ki   = u * (-p1 - p2) - (s * Vector2d::Ones());
            kim1 = u * p2 - (s * Vector2d::Ones());
            kip1 = u * p1;

            double wCurvature = 0.001;
            // calculate the gradient
            gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

            if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
            {
                return Vector2d(0.0, 0.0);
            }else
            {
                return gradient;
            }
        }
    }else{
        return Vector2d(0.0, 0.0);
    }

    return gradient;
}