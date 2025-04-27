#ifndef UTIL_HPP
#define UTIL_HPP

#include <vector>
#include <array>

namespace util{

    class Util{

        public:

            static std::array<double, 9> getK(double fx, double fy, double cx, double cy){
                std::array<double, 9> K = {
                    fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 1.0
                };

                return K;
            }

            static std::array<double, 9> getK(std::vector<double> dist_coeffs){
                return getK(dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3]);
            }

            static std::array<double, 9> getR(double fx, double fy, double cx, double cy){
                std::array<double, 9> R = {
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                };

                return R;
            }

            static std::array<double, 9> getR(std::vector<double> dist_coeffs){
                return getR(dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3]);
            }

            static std::array<double, 12> getP(double fx, double fy, double cx, double cy){
                std::array<double, 12>  P = {
                    fx, 0.0, cx, 0.0,
                    0.0, fy, cy, 0.0,
                    0.0, 0.0, 1.0, 0.0
                };

                return P;
            }

            static std::array<double, 12> getP(std::vector<double> dist_coeffs){
                return getP(dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3]);
            }
    };

} // namespace util

#endif