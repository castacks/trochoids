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

#include "trochoids/trochoid_utils.h"

typedef std::vector<std::tuple<double, double, double>> Path;

double trochoids::WrapTo2Pi(double a1)
{
  return a1 - 2*M_PI * floor(a1 / (2*M_PI));
}

double trochoids::WrapToPi(double a1)
{
  int m = static_cast<int>(a1 / (2*M_PI));
  a1 = a1 - m*2*M_PI;
  if (a1 > M_PI)
    a1 -= 2.0*M_PI;
  else if (a1 < -M_PI)
    a1 +=2.0*M_PI;
  return a1;
}

bool trochoids::get_trochoid_path_numerical(const XYZPsiState &s1,
                                            const XYZPsiState &s2,
                                            std::vector<XYZPsiState> &extended_path_out,
                                            const double *wind,
                                            double v,
                                            double max_kappa,
                                            bool exhaustive_solve_only,
                                            double waypoint_distance)
{
    trochoids::Trochoid trochoid;
    trochoid.problem.v = v;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {s1.x, s1.y, s1.psi};
    trochoid.problem.Xf = {s2.x, s2.y, s2.psi};
    Path path = trochoid.getTrochoidNumerical(exhaustive_solve_only, waypoint_distance);
    if (path.size() == 0)
    {
        return false;
    }
    XYZPsiState new_state;

    for (int i = 0; i < path.size(); i++)
    {
        new_state.x = std::get<0>(path[i]);
        new_state.y = std::get<1>(path[i]);
        new_state.psi = std::get<2>(path[i]);
        new_state.z = s1.z + ((static_cast<double>(i)) / static_cast<double>(path.size())) * (s2.z - s1.z);

        extended_path_out.push_back(new_state);
    }
    return true;
}

// If waypoint distance is 0, then it will use the default waypoint distance
bool trochoids::get_trochoid_path(const XYZPsiState &s1,
                                const XYZPsiState &s2,
                                std::vector<XYZPsiState> &extended_path_out,
                                const double *wind,
                                double v,
                                double max_kappa,
                                double waypoint_distance)
{
    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.v = v;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {s1.x, s1.y, s1.psi};
    trochoid.problem.Xf = {s2.x, s2.y, s2.psi};
    Path path = trochoid.getTrochoid(waypoint_distance);
    if (path.size() == 0)
    {
        return false;
    }
    XYZPsiState new_state;

    for (int i = 0; i < path.size(); i++)
    {
        new_state.x = std::get<0>(path[i]);
        new_state.y = std::get<1>(path[i]);
        new_state.psi = std::get<2>(path[i]);
        new_state.z = s1.z + ((static_cast<double>(i)) / static_cast<double>(path.size())) * (s2.z - s1.z);

        extended_path_out.push_back(new_state);
    }
    return true;
}

double trochoids::get_length(const XYZPsiState &s1,
                             const XYZPsiState &s2,
                             const double *wind,
                             double v,
                             double max_kappa)
{
    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.v = v;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.X0 = {s1.x, s1.y, s1.psi};
    trochoid.problem.Xf = {s2.x, s2.y, s2.psi};
    Path path = trochoid.getTrochoid();
    double length(0.0);
    //    std::cout<<path.size()<<std::endl;
    if (path.size() == 0)
    {
        //        std::cout<<"pathsizezero"<<std::endl;
        return 0.0;
    }
    for (int i = 0; i < path.size() - 1; i++)
    {
        double x = std::get<0>(path[i]);
        double y = std::get<1>(path[i]);
        double psi = std::get<2>(path[i]);
        double z = s1.z + ((static_cast<double>(i)) / static_cast<double>(path.size())) * (s2.z - s1.z);
        double x_ = std::get<0>(path[i + 1]);
        double y_ = std::get<1>(path[i + 1]);
        double z_ = s1.z + ((static_cast<double>(i+1)) / static_cast<double>(path.size())) * (s2.z - s1.z);

        length += sqrt(pow(x_ - x, 2) + pow(y_ - y, 2) + pow(z_ - z, 2));
    }
    return length;
}

double trochoids::get_length(std::vector<XYZPsiState> &path)
{
    double length = 0;
    for (int i = 0; i < path.size() - 1; i++)
    {
        double x = path[i].x;
        double y = path[i].y;
        double z = path[i].z;
        double x_ = path[i + 1].x;
        double y_ = path[i + 1].y;
        double z_ = path[i + 1].z;

        length += sqrt(pow(x_ - x, 2) + pow(y_ - y, 2) + pow(z_ - z, 2));
    }
    return length;
}
