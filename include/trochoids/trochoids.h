/*********************************************************************
    The Clear BSD License

    Copyright (c) 2023, AirLab
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted (subject to the limitations in the disclaimer
    below) provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

        * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from this
        software without specific prior written permission.

    NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
    THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
    IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Sagar Sachdev, Brady Moon, Jay Patrikar */

#ifndef TROCHOIDS_TROCHOIDS_H
#define TROCHOIDS_TROCHOIDS_H

#define EPSILON 1e-5
#include <math.h>
#include <vector>
#include <tuple>
#include <utility>
#include <algorithm>
#include <iostream>
#include <trochoids/DubinsStateSpace.h>
#include "trochoids/trochoid_utils.h"


namespace trochoids
{

class Trochoid
{
    double del1, del2, v, w, vw, phi1, phi2, psi_w;
    double xt10, yt10, xt20, yt20, E, G;
    double waypoint_distance = 0;

public:
    typedef std::vector<std::tuple<double, double, double>> Path;

    struct Problem
    {
        std::vector<double> X0;  // x, y, psi
        std::vector<double> Xf;  // x, y, psi
        std::vector<double> wind;  // x, y
        double max_kappa;
        double v;  // m/s
    };
    Problem problem;
    bool use_trochoid_classification = true;

    Trochoid() {}

    Path getTrochoidNumerical(bool exhaustive_solve_only = false,
                                double waypoint_distance = 0);

    Path getTrochoid(double waypoint_distance = 0);

    static double get_length(Path path);

    Path get_path(double t1, double t2);

    static std::vector<double> decision_pts(double x0, double y0,
                                            double xf, double yf,
                                            double psi1_trochoidal,
                                            double psi2_trochoidal);

private:
    double func(double t, double k);

    double derivfunc(double t, double k);

    double newtonRaphson(double x, double k, int idx_max = 100);

    void exhaustive_numerical_solve(double &del1, double &del2,
                                    double &phi1, double &phi2,
                                    double &vw, double &step_size,
                                    double &xt10, double &xt20,
                                    double &yt10, double &yt20,
                                    double &best_time, Path &final_path);

    void dubins_solve(double &phi1, double &phi2,
                        double &x0, double &xf,
                        double &y0, double &yf,
                        Path &final_path);

    void check_roots(double &del1, double &w,
                        double &phi1, double &F,
                        double &root1, double &root2, int &index);

    std::vector<std::pair<double, double>> trochoid_classification(double x0,
                                                                    double y0,
                                                                    double xf,
                                                                    double yf);

    bool check_within_four_r(double d2, double d1, double x0, double y0, double xf, double yf);

    bool check_within_four_r(double d, double x0, double y0, double xf, double yf);
};
}  // namespace trochoids

#endif  // TROCHOIDS_TROCHOIDS_H
