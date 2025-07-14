#include "kobuki_planner/utils.hpp"

std::vector<int> MtoP(const std::vector<double> coord_meters, const std::vector<double> origin, int height, double map_resolution)
{
    std::vector<int> coord_pixels(2);

    // coord_pixels[0] = static_cast<int>((width / 2) + (coord_meters[0] / map_resolution));
    // coord_pixels[1] = static_cast<int>((height / 2) - (coord_meters[1] / map_resolution));
    coord_pixels[0] =              -static_cast<int>((-coord_meters[0] + origin[0]) / map_resolution);
    coord_pixels[1] = height + static_cast<int>((-coord_meters[1] + origin[1])/ map_resolution);
    //static_cast<int>((coord_meters[0] + (width * map_resolution / 2)) / map_resolution);
    //static_cast<int>((coord_meters[1] + (height * map_resolution / 2)) / map_resolution);

    return coord_pixels;
}

std::vector<double> PtoM(const std::vector<int> coord_pixels, const std::vector<double> origin, int height, double map_resolution)
{
    std::vector<double> coord_meters(2);

    // coord_meters[0] = (coord_pixels[0] * map_resolution) - (width * map_resolution / 2);
    // coord_meters[1] = (coord_pixels[1] * map_resolution) - (height * map_resolution / 2);
    // coord_meters[0] = (coord_pixels[0] - (width / 2)) * map_resolution;
    // coord_meters[1] = -1 * (coord_pixels[1] - (height / 2)) * map_resolution;
    coord_meters[0] = (coord_pixels[0] - 0) * map_resolution + origin[0];
    coord_meters[1] = -((coord_pixels[1] - height) * map_resolution - origin[1]);

    return coord_meters;
}

double normalize_angle(double angle) {
    while (angle >= M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double euclidean_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}