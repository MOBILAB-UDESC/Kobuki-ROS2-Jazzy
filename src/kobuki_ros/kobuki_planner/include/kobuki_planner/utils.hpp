#include <vector>
#include <cmath>
/*
        ################################### (+, +)
        #                                 #
        #                                 #
        #           x                     #                 Meters
        #        (0.,0.)                  #
        #                                 #
        #                                 #
        ###################################
    Origin
    (-, -)

     (0, 0)                         (width, 0)
        ################################### 
        #                                 #
        #                                 #
        #                                 #                 Pixels
        #                                 #
        #                                 #
        #                                 #
        ###################################
  (0, height)                       (width, height)
*/

std::vector<int> MtoP(const std::vector<double> coord_meters, const std::vector<double> origin, int height, double map_resolution);

std::vector<double> PtoM(const std::vector<int> coord_pixels, const std::vector<double> origin, int height, double map_resolution);

double normalize_angle(double angle);

double euclidean_distance(double x1, double y1, double x2, double y2);