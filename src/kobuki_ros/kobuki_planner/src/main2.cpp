#include "kobuki_planner/smooth.hpp"
#include <iostream>

int main()
{
    std::vector<std::tuple<double, double, double>> waypoints = {
        {0.0, 0.0, 0.0},
        {1.0, 0.5, 0.0},
        {2.0, 1.0, 0.0},
        {3.0, 1.5, 0.0},
        {4.0, 2.0, 0.0},
        {5.0, 2.5, 0.0},
        {6.0, 3.0, 0.0}
    };

    // SmoothPath smoother(waypoints, 0.1);
    // auto smoothed = smoother.Darpa_curve();

    // for (const auto& [x, y, theta] : smoothed) {
    //     std::cout << "x: " << x << ", y: " << y << ", theta: " << theta << '\n';
    // }

    return 0;
}
