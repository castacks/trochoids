/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  Copyright (c) 2023, AirLab
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Mark Moll, Brady Moon */


#include <trochoids/DubinsStateSpace.h>


namespace Dubins
{
    const double twopi = 2. * M_PI;
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;

    inline double mod2pi(double x)
    {
        double xm = x - twopi * floor(x / twopi);
        return xm;
    }

    DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha + t) - sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[0], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha - t) + sa - sb - d) < 2* DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[1], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[2], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[3], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
            assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[4], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
            assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[5], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubins(double d, double alpha, double beta)
    {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return {DubinsStateSpace::dubinsPathType[0], 0, d, 0};

        DubinsStateSpace::DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRSL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLSR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRLR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLRL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
            path = tmp;
        return path;
    }
}  // namespace Dubins

const Dubins::DubinsStateSpace::DubinsPathSegmentType Dubins::DubinsStateSpace::dubinsPathType[6][3] =
    {{DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},
    {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT},
    {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},
    {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}};

double Dubins::DubinsStateSpace::distance(const DubinsState state1, const DubinsState state2) const
{
    if (isSymmetric_)
        return rho_ * std::min(dubins(state1, state2).length(), dubins(state2, state1).length());
    return rho_ * dubins(state1, state2).length();
}


Dubins::DubinsStateSpace::DubinsPath Dubins::DubinsStateSpace::dubins(const DubinsState state1,
                                                                              const DubinsState state2) const
{
    double x1 = state1.x, y1 = state1.y, th1 = state1.theta;
    double x2 = state2.x, y2 = state2.y, th2 = state2.theta;
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    return Dubins::dubins(d, alpha, beta);
}

int Dubins::find_quadrant(double angle)
{
    angle = mod2pi(angle);
    if (angle >= 0 && angle < M_PI/2)
        return 1;  // first quadrant
    else if (angle >= M_PI/2 && angle < M_PI)
        return 2;  // second quadrant
    else if (angle >= M_PI && angle < 3*M_PI/2)
        return 3;  // third quadrant
    else
        return 4;  // fourth quadrant
}

/**
 * @brief Uses the Dubins set classification for solving a Dubins path
 *
 * @param state1 DubinsState of the starting pose
 * @param state2 DubinsState of the ending pose
 * @return DubinsPath
 */
Dubins::DubinsStateSpace::DubinsPath Dubins::DubinsStateSpace::dubins_matrix(const DubinsState state1,
                                                                              const DubinsState state2) const
{
    double x1 = state1.x, y1 = state1.y, th1 = state1.theta;
    double x2 = state2.x, y2 = state2.y, th2 = state2.theta;
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    if (d > (std::abs(std::sin(alpha)) + std::abs(std::sin(beta)) +
                std::sqrt(4 - std::pow(std::cos(alpha) + std::cos(beta), 2))))
    {
        int init_quadrant = find_quadrant(alpha);
        int final_quadrant = find_quadrant(beta);

        if (init_quadrant == 1 && final_quadrant == 1)
        {
            return dubinsRSL(d, alpha, beta);
        }
        else if (init_quadrant == 2 && final_quadrant == 3)
        {
            return dubinsRSR(d, alpha, beta);
        }
        else if (init_quadrant == 3 && final_quadrant == 2)
        {
            return dubinsLSL(d, alpha, beta);
        }
        else if (init_quadrant == 4 && final_quadrant == 4)
        {
            return dubinsLSR(d, alpha, beta);
        }
        // Checking for cells with 2 cases
        else if (init_quadrant == 1 && final_quadrant == 2)
        {
            DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);
            DubinsPath dubinsrsl = dubinsRSL(d, alpha, beta);
            double S13 = dubinsrsr.length_[0] - M_PI;

            if (S13 <= 0)
            {
                double S12 = dubinsrsr.length_[1] - dubinsrsl.length_[1] - 2*(dubinsrsl.length_[2] - M_PI);
                if (S12 > 0)
                {
                    return dubinsrsl;
                }
                else
                {
                    return dubinsrsr;
                }
            }
            else
            {
                DubinsPath dubinslsr = dubinsLSR(d, alpha, beta);
                if (dubinsrsl.length() < dubinslsr.length())
                {
                    return dubinsrsl;
                }
                else
                {
                    return dubinslsr;
                }
            }
        }
        else if (init_quadrant == 1 && final_quadrant == 3)
        {
            DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);
            double t_rsr = dubinsrsr.length_[0];
            double S13 = t_rsr - M_PI;
            if (S13 < 0)
            {
                return dubinsrsr;
            }
            else if (S13 > 0)
            {
                return dubinsLSR(d, alpha, beta);
            }
            else
            {
                return Dubins::dubins(d, alpha, beta);
            }
        }
        else if (init_quadrant == 2 && final_quadrant == 1)
        {
            DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
            DubinsPath dubinsrsl = dubinsRSL(d, alpha, beta);
            double S31 = dubinslsl.length_[2] - M_PI;

            if (S31 <= 0)
            {
                double S21 = dubinslsl.length_[1] - dubinsrsl.length_[1] - 2*(dubinsrsl.length_[0] - M_PI);
                if (S21 > 0)
                {
                    return dubinsrsl;
                }
                else
                {
                    return dubinslsl;
                }
            }
            else
            {
                DubinsPath dubinslsr = dubinsLSR(d, alpha, beta);
                if (dubinsrsl.length() < dubinslsr.length())
                {
                    return dubinsrsl;
                }
                else
                {
                    return dubinslsr;
                }
            }
        }
        else if (init_quadrant == 2 && final_quadrant == 4)
        {
            DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);
            double q_rsr = dubinsrsr.length_[2];
            double S24 = q_rsr - M_PI;
            if (S24 < 0)
            {
                return dubinsrsr;
            }
            else
            {
                return dubinsRSL(d, alpha, beta);
            }
        }
        else if (init_quadrant == 3 && final_quadrant == 1)
        {
            DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
            double q_lsl = dubinslsl.length_[2];
            double S31 = q_lsl - M_PI;
            if (S31 < 0)
            {
                return dubinslsl;
            }
            else
            {
                return dubinsLSR(d, alpha, beta);
            }
        }
        else if (init_quadrant == 3 && final_quadrant == 4)
        {
            DubinsPath dubinslsr = dubinsLSR(d, alpha, beta);
            DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);
            double S24 = dubinsrsr.length_[2] - M_PI;

            if (S24 <= 0)
            {
                double S34 = dubinsrsr.length_[1] - dubinslsr.length_[1] - 2*(dubinslsr.length_[0] - M_PI);
                if (S34 > 0)
                {
                    return dubinslsr;
                }
                else
                {
                    return dubinsrsr;
                }
            }
            else
            {
                DubinsPath dubinsrsl = dubinsRSL(d, alpha, beta);
                if (dubinsrsl.length() < dubinslsr.length())
                {
                    return dubinsrsl;
                }
                else
                {
                    return dubinslsr;
                }
            }
        }
        else if (init_quadrant == 4 && final_quadrant == 2)
        {
            DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
            double t_lsl = dubinslsl.length_[0];
            double S42 = t_lsl - M_PI;
            if (S42 < 0)
            {
                return dubinslsl;
            }
            else
            {
                return dubinsRSL(d, alpha, beta);
            }
        }
        else if (init_quadrant == 4 && final_quadrant == 3)
        {
            DubinsPath dubinslsr = dubinsLSR(d, alpha, beta);
            DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
            double S42 = dubinslsl.length_[0] - M_PI;

            if (S42 <= 0)
            {
                double S43 = dubinslsl.length_[1] - dubinslsr.length_[1] - 2*(dubinslsr.length_[2] - M_PI);
                if (S43 > 0)
                {
                    return dubinslsr;
                }
                else
                {
                    return dubinslsl;
                }
            }
            else
            {
                DubinsPath dubinsrsl = dubinsRSL(d, alpha, beta);
                if (dubinsrsl.length() < dubinslsr.length())
                {
                    return dubinsrsl;
                }
                else
                {
                    return dubinslsr;
                }
            }
        }
        else if (init_quadrant == 1 && final_quadrant == 4)
        {
            DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);

            double t_rsr = dubinsrsr.length_[0];
            double q_rsr = dubinsrsr.length_[2];
            double S114 = t_rsr - M_PI;
            double S214 = q_rsr - M_PI;
            if (S114 > 0)
            {
                return dubinsLSR(d, alpha, beta);
            }
            else if (S214 > 0)
            {
                return dubinsRSL(d, alpha, beta);
            }
            else
            {
                return dubinsrsr;
            }
        }
        else if (init_quadrant == 2 && final_quadrant == 2)
        {
            DubinsPath dubinsrsl = dubinsRSL(d, alpha, beta);
            double prsl = dubinsrsl.length_[1];
            
            if (alpha > beta)
            {
                DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
                double trsl = dubinsrsl.length_[0];
                double plsl = dubinslsl.length_[1];
                double S122 = plsl - prsl - 2*(trsl - M_PI);
                if (S122 < 0)
                {
                    return dubinslsl;
                }
                else if (S122 > 0)
                {
                    return dubinsrsl;
                }
                else
                {
                    return Dubins::dubins(d, alpha, beta);
                }
            }
            else
            {
                DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);
                double prsr = dubinsrsr.length_[1];
                double qrsl = dubinsrsl.length_[2];
                double S222 = prsr - prsl - 2*(qrsl - M_PI);
                if (S222 < 0)
                {
                    return dubinsrsr;
                }
                else if (S222 > 0)
                {
                    return dubinsrsl;
                }
                else
                {
                    return Dubins::dubins(d, alpha, beta);
                }
            }
        }
        else if (init_quadrant == 3 && final_quadrant == 3)
        {
            DubinsPath dubinslsr = dubinsLSR(d, alpha, beta);
            double plsr = dubinslsr.length_[1];
            
            if (alpha < beta)
            {
                DubinsPath dubinsrsr = dubinsRSR(d, alpha, beta);
                double prsr = dubinsrsr.length_[1];
                double tlsr = dubinslsr.length_[0];
                double S133 = prsr - plsr - 2*(tlsr - M_PI);
                if (S133 < 0)
                {
                    return dubinsrsr;
                }
                else if (S133 > 0)
                {
                    return dubinslsr;
                }
                else
                {
                    return Dubins::dubins(d, alpha, beta);
                }
            }
            else
            {
                DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
                double plsl = dubinslsl.length_[1];
                double qlsr = dubinslsr.length_[2];
                double S233 = plsl - plsr - 2*(qlsr - M_PI);
                if (S233 < 0)
                {
                    return dubinslsl;
                }
                else if (S233 > 0)
                {
                    return dubinslsr;
                }
                else
                {
                    return Dubins::dubins(d, alpha, beta);
                }
            }
        }
        else if (init_quadrant == 4 && final_quadrant == 1)
        {
            DubinsPath dubinslsl = dubinsLSL(d, alpha, beta);
            double tlsl = dubinslsl.length_[0];
            double qlsl = dubinslsl.length_[2];
            double S141 = tlsl - M_PI;
            double S241 = qlsl - M_PI;

            if (S141 > 0)
            {
                return dubinsRSL(d, alpha, beta);
            }
            else if (S241 > 0)
            {
                return dubinsLSR(d, alpha, beta);
            }
            else
            {
                return dubinslsl;
            }
        }
    }
    return Dubins::dubins(d, alpha, beta);
}



