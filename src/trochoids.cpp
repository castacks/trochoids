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

#include "trochoids/trochoids.h"

typedef std::vector<std::tuple<double, double, double>> Path;

int find_quadrant(double angle)
{
    angle = trochoids::WrapTo2Pi(angle);
    if (angle >= 0 && angle < M_PI/2)
        return 1;  // first quadrant
    else if (angle >= M_PI/2 && angle < M_PI)
        return 2;  // second quadrant
    else if (angle >= M_PI && angle < 3*M_PI/2)
        return 3;  // third quadrant
    else
        return 4;  // fourth quadrant
}

std::vector<std::pair<double, double>> del_picker(int init_quadrant, int final_quadrant)
{
        std::vector<std::pair<double, double>> del;

        if (init_quadrant == 1 && final_quadrant == 1)
        {
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            return del;
        }
        else if (init_quadrant == 2 && final_quadrant == 3)
        {
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            return del;
        }
        else if (init_quadrant == 3 && final_quadrant == 2)
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            return del;
        }
        else if (init_quadrant == 4 && final_quadrant == 4)
        {
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            return del;
        }
        // Checking for cells with 2 cases
        else if (init_quadrant == 1 && final_quadrant == 2)
        {
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            return del;
        }
        else if (init_quadrant == 1 && final_quadrant == 3)
        {
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            return del;
        }
        else if (init_quadrant == 2 && final_quadrant == 1)
        {
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            return del;
        }
        else if (init_quadrant == 2 && final_quadrant == 4)
        {
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            return del;
        }
        else if (init_quadrant == 3 && final_quadrant == 1)
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            return del;
        }
        else if (init_quadrant == 3 && final_quadrant == 4)
        {
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            return del;
        }
        else if (init_quadrant == 4 && final_quadrant == 2)
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            return del;
        }
        else if (init_quadrant == 4 && final_quadrant == 3)
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            return del;
        }
        else if (init_quadrant == 1 && final_quadrant == 4)
        {
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            del.push_back(std::pair<double, double>(1, -1));   // LSR
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            return del;
        }
        else if (init_quadrant == 2 && final_quadrant == 2)
        {
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            return del;
        }
        else if (init_quadrant == 3 && final_quadrant == 3)
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            return del;
        }
        else if (init_quadrant == 4 && final_quadrant == 1)
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            return del;
        }
        else
        {
            del.push_back(std::pair<double, double>(1, 1));  // LSL
            del.push_back(std::pair<double, double>(-1, 1));  // RSL
            del.push_back(std::pair<double, double>(1, -1));  // LSR
            del.push_back(std::pair<double, double>(-1, -1));  // RSR
            return del;
        }
}

void decision_pts_one_angle(std::vector<double> &decision_pts,
                            double angle,
                            double del_theta,
                            bool is_goal_above_start,
                            double x_diff,
                            double y_diff)
{
    angle = trochoids::WrapToPi(angle);
    angle += del_theta;

    if ((is_goal_above_start && angle < M_PI) || (!is_goal_above_start && angle > -M_PI))
    {
        double r = y_diff/tan(angle);
        double d = x_diff - r;
        decision_pts.push_back(d);
    }
    else
    {
        return;
    }

    if (is_goal_above_start)
        angle += M_PI_2;
    else
        angle -= M_PI_2;

    // Add 90 degrees and check if new decision point
    if ((is_goal_above_start && angle < M_PI) || (!is_goal_above_start && angle > -M_PI))
    {
        double r = y_diff/tan(angle);
        double d = x_diff - r;
        decision_pts.push_back(d);
    }
}

bool trochoids::Trochoid::check_within_four_r(double d2, double d1, double x0, double y0, double xf, double yf)
{
    // Check that segment isn't ever closer than 4R to starting point
    double four_R = 4.0/this->problem.max_kappa;
    if (abs(yf-y0) > four_R)
    {
        return false;
    }
    // then check if lowest d x 4r away from left side
    else if ((xf - d1) < (x0 - four_R))
    {
        return false;
    }
    // then check if highest d x 4r away from right side
    else if ((xf - d2) > (x0 + four_R))
    {
        return false;
    }
    // then check if within the two points bounding 4R (or could check angle)
    double delta_x = sqrt(four_R*four_R - pow(yf-y0, 2));
    double x_upper = x0 + delta_x;
    double x_lower = x0 - delta_x;
    double d_upper = (xf - d1);
    double d_lower = (xf - d2);
    if (d_upper < x_lower || d_lower > x_upper ||
       (d_upper > x_upper && d_lower > x_upper) ||
       (d_upper < x_lower && d_lower < x_lower))
    {
        return false;
    }

    // then check if within TODO
    return true;
}

bool trochoids::Trochoid::check_within_four_r(double d, double x0, double y0, double xf, double yf)
{
    // Check that segment isn't ever closer than 4R to starting point
    double four_R = 4.0/this->problem.max_kappa;
    if (abs(yf-y0) > four_R)
    {
        return false;
    }
    // then check if x 4r away
    else if ((xf - d) < (x0 - four_R))
    {
        return false;
    }
    // then check if within the two points bounding 4R
    double delta_x = sqrt(four_R*four_R - pow(yf-y0, 2));
    if ((xf - d) < x0 - delta_x)
    {
        return false;
    }

    return true;
}

std::vector<double> trochoids::Trochoid::decision_pts(double x0,
                                                      double y0,
                                                      double xf,
                                                      double yf,
                                                      double psi1_trochoidal,
                                                      double psi2_trochoidal)
{
    bool is_goal_above_start = yf > y0;
    std::vector<double> decision_pts;
    decision_pts.push_back(0);

    double theta = atan2(yf-y0, xf-x0);
    double del_theta1;
    double del_theta2;

    if (is_goal_above_start)
    {
        del_theta1 = fmod(trochoids::WrapTo2Pi(psi1_trochoidal - theta), M_PI_2);
        del_theta2 = fmod(trochoids::WrapTo2Pi(psi2_trochoidal - theta), M_PI_2);
    }
    else
    {
        del_theta1 = fmod(trochoids::WrapTo2Pi(psi1_trochoidal - theta)-M_2PI , -M_PI_2);
        del_theta2 = fmod(trochoids::WrapTo2Pi(psi2_trochoidal - theta)-M_2PI , -M_PI_2);
    }

    double y_diff = yf - y0;
    double x_diff = xf - x0;

    decision_pts_one_angle(decision_pts, theta, del_theta1, is_goal_above_start, x_diff, y_diff);
    decision_pts_one_angle(decision_pts, theta, del_theta2, is_goal_above_start, x_diff, y_diff);

    std::sort(decision_pts.begin(), decision_pts.end());
    auto last = std::unique(decision_pts.begin(), decision_pts.end(), [](double l, double r)
                            { return std::abs(l - r) < EPSILON; });
    decision_pts.erase(last, decision_pts.end());

    return decision_pts;
}

/*
x0, y0, xf, yf are the initial and final positions in the trochoidal frame
psi1, psi2 are the initial and final headings in intertial frame
*/
std::vector<std::pair<double, double>> trochoids::Trochoid::trochoid_classification(double x0,
                                                                                    double y0,
                                                                                    double xf,
                                                                                    double yf)
{
    if (this->use_trochoid_classification)
    {
        // First compute the psi angles in the trochoidal frame
        double psi1_trochoidal = trochoids::WrapTo2Pi(this->phi1 - this->psi_w);
        double psi2_trochoidal = trochoids::WrapTo2Pi(this->phi2 - this->psi_w);

        Path final_path;

        double time_to_decision_point_goal = 0;
        double time_to_decision_point_start = 0;  // Solve this with dubins

        std::vector<double> decision_points = decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);


        Dubins::DubinsStateSpace::DubinsState start = {x0, y0, psi1_trochoidal};

        Dubins::DubinsStateSpace::DubinsPath dubins_path;
        Dubins::DubinsStateSpace dubins_path_obj(1/problem.max_kappa);

        double turn_radius = (1.0/problem.max_kappa);
        double d_between_angle = NAN;
        for (int i = 1; i < decision_points.size(); ++i)
        {
            bool within_four_r = check_within_four_r(decision_points[i], decision_points[i-1], x0, y0, xf, yf);
            if (within_four_r)
            {
                std::vector<std::pair<double, double>> del;
                del.push_back(std::pair<double, double>(1, 1));
                del.push_back(std::pair<double, double>(-1, 1));
                del.push_back(std::pair<double, double>(1, -1));
                del.push_back(std::pair<double, double>(-1, -1));
                return del;
            }

            double best_length = std::numeric_limits<double>::infinity();
            double xd = xf - decision_points[i];

            // solve for the time to reach the decision point
            Dubins::DubinsStateSpace::DubinsState goal = {xd, yf, psi2_trochoidal};

            dubins_path = dubins_path_obj.dubins_matrix(start, goal);
            // dubins_path = dubins_path_obj.dubins(start, goal);

            double length = dubins_path.length()*turn_radius;

            if (length < best_length)
            {
                best_length = length;
            }
            // calculate time for the goal point movement (moves at the speed of wind)
            time_to_decision_point_goal = decision_points[i]/this->vw;
            // calculate time for the start point movement (moves at the speed of the vehicle)
            time_to_decision_point_start = best_length/this->v;

            if (time_to_decision_point_start < time_to_decision_point_goal)
            {
                double d_between = (decision_points[i]+decision_points[i-1])/2;
                d_between_angle = atan2(yf-y0, xf-d_between-x0);
                break;
            }
        }
        // Compute the quadrant between the decision points
        if (std::isnan(d_between_angle))  // No decision points or at last decision point
        {
            bool within_four_r = check_within_four_r(decision_points[decision_points.size()-1], x0, y0, xf, yf);
            if (within_four_r)
            {
                std::vector<std::pair<double, double>> del;
                del.push_back(std::pair<double, double>(1, 1));
                del.push_back(std::pair<double, double>(-1, 1));
                del.push_back(std::pair<double, double>(1, -1));
                del.push_back(std::pair<double, double>(-1, -1));
                return del;
            }
            double d_0_angle = atan2(yf-y0, xf-decision_points[decision_points.size()-1]-x0);
            if (d_0_angle > 0)
            {
                d_between_angle = (d_0_angle + M_PI)/2;
            }
            else
            {
                d_between_angle = (d_0_angle - M_PI)/2;
            }
        }

        int a1 = find_quadrant(psi1_trochoidal - d_between_angle);
        int a2 = find_quadrant(psi2_trochoidal - d_between_angle);

        std::vector<std::pair<double, double>> del = del_picker(a1, a2);
        return del;
    }
    std::vector<std::pair<double, double>> del;
    del.push_back(std::pair<double, double>(1, 1));  // ll
    del.push_back(std::pair<double, double>(1, -1));  // lr
    del.push_back(std::pair<double, double>(-1, 1));  // rl
    del.push_back(std::pair<double, double>(-1, -1));  // rr
    return del;
}

// For LSL and RSR, use analytical solution
Path trochoids::Trochoid::getTrochoid(double waypoint_distance)
{
    this->waypoint_distance = waypoint_distance;
    // Establishing some variables
    double best_time = std::numeric_limits<double>::infinity();
    Path final_path;
    Path temp_path;
    psi_w = trochoids::WrapTo2Pi(atan2(problem.wind[1], problem.wind[0]));
    // Transform to trochoidal frame
    double x0 = problem.X0[0] * cos(psi_w) + problem.X0[1] * sin(psi_w);
    double y0 = -problem.X0[0] * sin(psi_w) + problem.X0[1] * cos(psi_w);
    double xf = problem.Xf[0] * cos(psi_w) + problem.Xf[1] * sin(psi_w);
    double yf = -problem.Xf[0] * sin(psi_w) + problem.Xf[1] * cos(psi_w);
    v = problem.v;
    w = v / (1 / problem.max_kappa);
    vw = sqrt(pow(problem.wind[1], 2) + pow(problem.wind[0], 2));
    phi1 = trochoids::WrapTo2Pi(problem.X0[2]);
    phi2 = trochoids::WrapTo2Pi(problem.Xf[2]);

    if (abs(vw) < EPSILON)
    {
        dubins_solve(phi1, phi2,
                    x0, xf,
                    y0, yf, final_path);

        return final_path;
    }
    std::vector<std::pair<double, double>> del = trochoid_classification(x0, y0, xf, yf);

    for (int g = 0; g < del.size(); g++)
    {
        del1 = del[g].first;
        del2 = del[g].second;
        double t_2pi = 2 * M_PI / w;
        phi1 = fmod(problem.X0[2] - psi_w, M_2PI);
        phi2 = fmod(problem.Xf[2] - psi_w - del2 * 2 * M_PI, M_2PI);
        xt10 = x0 - (v / (del1 * w)) * sin(phi1);
        yt10 = y0 + (v / (del1 * w)) * cos(phi1);
        xt20 = xf - (v / (del2 * w)) * sin(phi2 + del2 * 2 * M_PI) - vw * t_2pi;
        yt20 = yf + (v / (del2 * w)) * cos(phi2 + del2 * 2 * M_PI);

        E = v * (((vw * (del1 - del2)) / (del1 * del2 * w)) - (yt20 - yt10));
        G = vw * (yt20 - yt10) + ((v * v * (del2 - del1)) / (del1 * del2 * w));

        if (abs(del1 - del2) < EPSILON)  // LSL or RSR condition
        {
            for (double k = -3; k < 3; k++)
            {
                double alpha = atan2((yt20 - yt10),
                                     (xt20 - xt10 + vw * (fmod(phi1 - phi2, M_2PI) + 2 * k * M_PI) /
                                     (del2 * w)));

                double t1_ = (t_2pi / (del1 * 2 * M_PI)) * (asin((vw / v) * sin(alpha)) + alpha - phi1);
                if (t1_ < 0 || t1_ > t_2pi)
                {
                    t1_ = t1_ - t_2pi*floor(t1_/t_2pi);
                }
                        double t2 = (t1_ + (fmod(phi1 - phi2, M_2PI) + 2 * k * M_PI)/(del2*w));
                        if (t2 <= -(2 * M_PI / w) || t2 > (2 * M_PI / w))
                        {
                            continue;
                        }

                        double x1t2 = (v / (del1 * w)) * sin(del1 * w * t1_ + phi1) + vw * t1_ + xt10;
                        double y1t2 = -(v / (del1 * w)) * cos(del1 * w * t1_ + phi1) + yt10;
                        double x2t2 = (v / (del2 * w)) * sin(del2 * w * t2 + phi2) + vw * t2 + xt20;
                        double y2t2 = -(v / (del2 * w)) * cos(del2 * w * t2 + phi2) + yt20;

                        if (abs(trochoids::WrapTo2Pi(atan2(y2t2 - y1t2, x2t2 - x1t2)) -
                                trochoids::WrapTo2Pi(alpha)) > M_PI/2.0)
                        {
                            continue;
                        }

                        double xt2dot = v*cos(del2*w*t2 + phi2) + vw;
                        double yt2dot = v*sin(del2*w*t2 + phi2);

                        double tBeta = t1_ + sqrt((x2t2-x1t2)*(x2t2-x1t2) + (y2t2-y1t2)*(y2t2-y1t2)) /
                                             sqrt((xt2dot*xt2dot + yt2dot*yt2dot));
                        double T = tBeta + (t_2pi - t2);


                        if (T < best_time)
                        {
                            best_time = T;
                            temp_path = get_path(t1_, t2);
                            // double length = get_length(temp_path);
                            // best_length = length;
                            final_path = temp_path;
                        }
            }
        }
        else
        {
            double step_size = (2 * t_2pi)/360.0;
            exhaustive_numerical_solve(del1, del2, phi1, phi2,
                                        vw, step_size, xt10, xt20,
                                        yt10, yt20, best_time, final_path);
        }
    }
    return final_path;
}


Path trochoids::Trochoid::getTrochoidNumerical(bool exhaustive_solve_only, double waypoint_distance)
{
    this->waypoint_distance = waypoint_distance;
    // Establishing some variables
    std::vector<std::pair<double, double>> del;
    del.push_back(std::pair<double, double>(1, 1));
    del.push_back(std::pair<double, double>(1, -1));
    del.push_back(std::pair<double, double>(-1, 1));
    del.push_back(std::pair<double, double>(-1, -1));
    double best_time = std::numeric_limits<double>::infinity();
    Path final_path;
    psi_w = trochoids::WrapTo2Pi(atan2(problem.wind[1], problem.wind[0]));
    double x0 = problem.X0[0]*cos(psi_w) + problem.X0[1]*sin(psi_w);
    double y0 = -problem.X0[0]*sin(psi_w) + problem.X0[1]*cos(psi_w);
    double xf = problem.Xf[0]*cos(psi_w) + problem.Xf[1]*sin(psi_w);
    double yf = -problem.Xf[0]*sin(psi_w) + problem.Xf[1]*cos(psi_w);
    v = problem.v;
    w = v / (1 / problem.max_kappa);
    vw = sqrt(pow(problem.wind[1], 2) + pow(problem.wind[0], 2));
    phi1 = trochoids::WrapTo2Pi(problem.X0[2]);
    phi2 = trochoids::WrapTo2Pi(problem.Xf[2]);
    if (!exhaustive_solve_only && abs(vw) < EPSILON)
    {
        dubins_solve(phi1, phi2,
                    x0, xf,
                    y0, yf, final_path);

        return final_path;
    }

    for (int g=0; g < del.size(); g++)
    {
        del1 = del[g].first;
        del2 = del[g].second;
        double t_2pi = 2 * M_PI / w;
        phi1 = trochoids::WrapTo2Pi(problem.X0[2] - atan2(problem.wind[1], problem.wind[0]));
        phi2 = trochoids::WrapTo2Pi(problem.Xf[2] - atan2(problem.wind[1], problem.wind[0]) - del2 * 2 * M_PI);
        xt10 = x0 - (v / (del1 * w)) * sin(phi1);
        yt10 = y0 + (v / (del1 * w)) * cos(phi1);
        xt20 = xf - (v / (del2 * w)) * sin(phi2 + del2 * 2 * M_PI) - vw * t_2pi;
        yt20 = yf + (v / (del2 * w)) * cos(phi2 + del2 * 2 * M_PI);

        E = v * (((vw * (del1 - del2)) / (del1 * del2 * w)) - (yt20 - yt10));
        G = vw * (yt20 - yt10) + ((v * v * (del2 - del1)) / (del1 * del2 * w));
            double step_size = (2 * 2 * M_PI / w)/360.0;
            exhaustive_numerical_solve(del1, del2, phi1, phi2,
                                        vw, step_size, xt10, xt20,
                                        yt10, yt20, best_time, final_path);
    }

    return final_path;
}



Path trochoids::Trochoid::get_path(double t1, double t2)
{
    Path path;
    // double step_size = std::max(0.1, t1/360.0);
    double turn_step_size = M_PI/(180*w);
    assert(waypoint_distance >= 0.0);
    // std::cout << "waypoint_distance: " << waypoint_distance << std::endl;
    if (waypoint_distance > 0.01)
    {
        turn_step_size = waypoint_distance/v;
        // std::cout << "turn_step_size: " << turn_step_size << std::endl;
    }
    // First curve
    for (double t = 0.0; t < t1 ; t += turn_step_size)
    {
        double x = (v/(del1*w))*sin(del1*w*t+phi1) + vw*t + xt10;
        double y = -(v/(del1*w))*cos(del1*w*t+phi1) + yt10;
        double psi = trochoids::WrapTo2Pi(del1*w*t+phi1);
        double xt = x*cos(psi_w) - y*sin(psi_w);
        double yt = x*sin(psi_w) + y*cos(psi_w);
        psi = trochoids::WrapTo2Pi(psi + psi_w);
        path.push_back(std::make_tuple(xt, yt, psi));
    }

    // Straight (Fixed at 100 points)
    double x1t2 = (v / (del1 * w)) * sin(del1 * w * t1 + phi1) + vw * t1 + xt10;
    double y1t2 = -(v / (del1 * w)) * cos(del1 * w * t1 + phi1) + yt10;
    double x2t2 = (v / (del2 * w)) * sin(del2 * w * t2 + phi2) + vw * t2 + xt20;
    double y2t2 = -(v / (del2 * w)) * cos(del2 * w * t2 + phi2) + yt20;
    double straight_step_size = 0.01;
    if (waypoint_distance > 0.01)
    {
        double straight_distance = sqrt(pow(x1t2 - x2t2, 2) + pow(y1t2 - y2t2, 2));
        straight_step_size = 1/(straight_distance/waypoint_distance);
    }
    for (double t = 0; t < 1 ; t += straight_step_size)
    {
        double x = x1t2 - t*(x1t2 - x2t2);
        double y = y1t2 - t*(y1t2 - y2t2);
        // double psi = trochoids::WrapTo2Pi(atan2(y2t2 - y1t2, x2t2 - x1t2));
        double psi = trochoids::WrapTo2Pi(del1*w*t1+phi1);
        double xt = x*cos(psi_w) - y*sin(psi_w);
        double yt = x*sin(psi_w) + y*cos(psi_w);
        psi = trochoids::WrapTo2Pi(psi + psi_w);

        path.push_back(std::make_tuple(xt, yt, psi));
    }
    // Second curve
    for (double t = t2; t < (2*M_PI/w) ; t += turn_step_size)
    {
        double x = (v/(del2*w))*sin(del2*w*t+phi2) + vw*t + xt20;
        double y = -(v/(del2*w))*cos(del2*w*t+phi2) + yt20;
        double psi = trochoids::WrapTo2Pi(del2*w*t+phi2);
        double xt = x*cos(psi_w) - y*sin(psi_w);
        double yt = x*sin(psi_w) + y*cos(psi_w);
        psi = trochoids::WrapTo2Pi(psi + psi_w);

        path.push_back(std::make_tuple(xt, yt, psi));
    }
    path.push_back(std::make_tuple(problem.Xf[0], problem.Xf[1], problem.Xf[2]));
    return path;
}

double trochoids::Trochoid::get_length(Path path)
{
    double length(0.0);
    for (int i=0; i < path.size()-1; i++)
    {
        // Compute the euclidean distance between two paths
        double dx = std::get<0>(path[i+1]) - std::get<0>(path[i]);
        double dy = std::get<1>(path[i+1]) - std::get<1>(path[i]);
        length += sqrt(dx*dx + dy*dy);
    }
    return length;
}

void trochoids::Trochoid::exhaustive_numerical_solve(double &del1, double &del2,
                                                    double &phi1, double &phi2,
                                                    double &vw, double &step_size,
                                                    double &xt10, double &xt20,
                                                    double &yt10, double &yt20,
                                                    double &best_time, Path &final_path)
{
    Path temp_path;
    for (double k = -3; k < 3; k++)
    {
        double t = 0;
        double t_2pi = (2 * M_PI / w);
        std::vector<double> t1;
        while (t < 2 * t_2pi)
        {
            double t1_ = newtonRaphson(t, k);
            t += step_size;
            if (t1_ >= 0.0 && t1_ < 2 * t_2pi && abs(func(t1_, k)) < EPSILON)  // Changed from 0.1
            {
                t1.push_back(t1_);
            }
        }
        std::sort(t1.begin(), t1.end());
        auto last = std::unique(t1.begin(), t1.end(), [](double l, double r)
                                { return std::abs(l - r) < EPSILON; });
        t1.erase(last, t1.end());
        for (int i = 0; i < t1.size(); i++)
        {
            double var = func(t1[i], k);
            double t2 = (del1 / del2) * t1[i] + ((trochoids::WrapTo2Pi(phi1 - phi2) + 2 * k * M_PI) / (del2 * w));
            if (t2 <= -t_2pi || t2 > t_2pi)
                continue;

            double x1t2 = (v / (del1 * w)) * sin(del1 * w * t1[i] + phi1) + vw * t1[i] + xt10;
            double y1t2 = -(v / (del1 * w)) * cos(del1 * w * t1[i] + phi1) + yt10;
            double x2t2 = (v / (del2 * w)) * sin(del2 * w * t2 + phi2) + vw * t2 + xt20;
            double y2t2 = -(v / (del2 * w)) * cos(del2 * w * t2 + phi2) + yt20;

            double alpha = atan2(v * sin(del1 * w * t1[i] + phi1), v * cos(del1 * w * t1[i] + phi1) + vw);
            if (abs(trochoids::WrapTo2Pi(atan2(y2t2 - y1t2, x2t2 - x1t2)) - trochoids::WrapTo2Pi(alpha)) > M_PI_2)
            {
                continue;
            }
            double xt1dot = v*cos(del1*w*t1[i] + phi1) + vw;
            double yt1dot = v*sin(del1*w*t1[i] + phi1);
            double xt2dot = v*cos(del2*w*t2 + phi2) + vw;
            double yt2dot = v*sin(del2*w*t2 + phi2);

            double tBeta = t1[i] + sqrt((x2t2-x1t2)*(x2t2-x1t2) + (y2t2-y1t2)*(y2t2-y1t2)) /
                                   sqrt((xt2dot*xt2dot + yt2dot*yt2dot));
            double T = tBeta + (t_2pi - t2);
            if (T < best_time)
            {
                best_time = T;
                temp_path = get_path(t1[i], t2);
                // double length = get_length(temp_path);
                // best_length = length;
                final_path = temp_path;
            }
        }
    }
}

void trochoids::Trochoid::dubins_solve(double &phi1, double &phi2,
                                        double &x0, double &xf,
                                        double &y0, double &yf,
                                        Path &final_path)
{
    Path temp_path;
    Dubins::DubinsStateSpace::DubinsState start = {x0, y0, phi1};
    Dubins::DubinsStateSpace::DubinsState goal = {xf, yf, phi2};

    double turn_radius = (1.0/problem.max_kappa);

    Dubins::DubinsStateSpace::DubinsPath dubins_path;
    Dubins::DubinsStateSpace dubins_path_obj(1/problem.max_kappa);

    dubins_path = dubins_path_obj.dubins_matrix(start, goal);

    Dubins::DubinsStateSpace::DubinsState s_state;
    s_state.x = 0;
    s_state.y = 0;
    s_state.theta = start.theta;

    std::vector<double> t1;
    t1.push_back(0.0);
    t1.push_back(dubins_path.length_[0]/dubins_path.length());
    t1.push_back((dubins_path.length_[0] + dubins_path.length_[1])/dubins_path.length());

    std::vector<double> arr1;
    arr1.push_back(dubins_path.length_[0]);
    arr1.push_back(dubins_path.length_[1]);
    arr1.push_back(dubins_path.length_[2]);

    for (unsigned int i = 0; i < 3; ++i)
    {
        double t_s = t1[i];
        double seg_length_in_radians = arr1[i];
        double step_size_in_rad = M_PI/180.0;

        assert(waypoint_distance >= 0.0);
        if (waypoint_distance > 0.01)
        {
            step_size_in_rad = waypoint_distance/turn_radius;
        }

        double num_points = seg_length_in_radians/step_size_in_rad;

        switch (dubins_path.type_[i])
        {
            case Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT:
            {
                for (int j = 0; j < num_points; ++j)
                {
                    temp_path.push_back(std::make_tuple(s_state.x*turn_radius + x0,
                                                        s_state.y*turn_radius + y0,
                                                        s_state.theta));
                    double phi = s_state.theta;

                    s_state.x = s_state.x + sin(phi + step_size_in_rad) - sin(phi);
                    s_state.y = s_state.y - cos(phi + step_size_in_rad) + cos(phi);
                    s_state.theta = phi + step_size_in_rad;
                }
                break;
            }
            case Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT:
            {
                for (int k = 0; k < num_points; ++k)
                {
                    temp_path.push_back(std::make_tuple(s_state.x*turn_radius + x0,
                                                        s_state.y*turn_radius + y0,
                                                        s_state.theta));
                    double phi = s_state.theta;

                    s_state.x = s_state.x - sin(phi - step_size_in_rad) + sin(phi);
                    s_state.y = s_state.y + cos(phi - step_size_in_rad) - cos(phi);
                    s_state.theta = phi - step_size_in_rad;
                }
                break;
            }
            case Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_STRAIGHT:
            {
                double phi = s_state.theta;

                if (waypoint_distance <= 0.01)
                {
                    num_points = 100;
                    step_size_in_rad = seg_length_in_radians/num_points;
                }

                for (int n = 0; n < num_points; ++n)
                {
                    temp_path.push_back(std::make_tuple(s_state.x*turn_radius + x0,
                                                        s_state.y*turn_radius + y0,
                                                        s_state.theta));

                    s_state.x = s_state.x + step_size_in_rad*cos(phi);
                    s_state.y = s_state.y + step_size_in_rad*sin(phi);
                }
                break;
            }
        }
        // set for next point and add goal point for the last one.
        if (i == 0)
        {
            if (dubins_path.type_[i] == Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT)
            {
                s_state.x = -sin(phi1 - arr1[i]) + sin(phi1);
                s_state.y = cos(phi1 - arr1[i]) - cos(phi1);
                s_state.theta = phi1 - arr1[i];
            }
            else if (dubins_path.type_[i] == Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT)
            {
                s_state.x = sin(phi1 + arr1[i]) - sin(phi1);
                s_state.y = -cos(phi1 + arr1[i]) + cos(phi1);
                s_state.theta = phi1 + arr1[i];
            }
            else
            {
                std::cout << "ERROR!!\n";
            }
        }
        else if (i == 1)
        {
            if (dubins_path.type_[2] == Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT)
            {
                s_state.x = -sin(phi2 + arr1[2]) + sin(phi2) + (xf-x0)/turn_radius;
                s_state.y = cos(phi2 + arr1[2]) - cos(phi2) + (yf-y0)/turn_radius;
                s_state.theta = phi2 + arr1[2];
            }
            else if (dubins_path.type_[2] == Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT)
            {
                s_state.x = sin(phi2 - arr1[2]) - sin(phi2) + (xf-x0)/turn_radius;
                s_state.y = -cos(phi2 - arr1[2]) + cos(phi2) + (yf-y0)/turn_radius;
                s_state.theta = phi2 - arr1[2];
            }
            else
            {
                std::cout << "ERROR!!\n";
            }
        }
        else if (i == 2)
        {
            temp_path.push_back(std::make_tuple(xf, yf, phi2));
        }
    }

    final_path = temp_path;
}

void trochoids::Trochoid::check_roots(double &del1, double &w,
                                      double &phi1, double &F,
                                      double &root1, double &root2, int &index)
{
    if (abs(E*cos(del1*w*root2+phi1) + F*sin(del1*w*root2+phi1)-G) > 0.1 ||
        abs(E*cos(del1*w*root1+phi1) + F*sin(del1*w*root1+phi1)-G) > 0.1)
        throw std::runtime_error("Not a root!");

    if (index == -1 && root1 >= 0.0 && root2 >= 0.0)
        throw std::runtime_error("Caught case where less than -1 works");
}

double trochoids::Trochoid::func(double t, double k)
{
    double F = v*((xt20-xt10)+vw*(t*((del1/del2)-1)+(((trochoids::WrapTo2Pi(phi1-phi2)+2*k*M_PI)/(del2*w)))));
    double val = E*cos(del1*w*t+phi1) + F*sin(del1*w*t+phi1)-G;

    return val;
}

double trochoids::Trochoid::derivfunc(double t, double k)
{
    double F = v*((xt20-xt10)+vw*(t*((del1/del2)-1)+(((trochoids::WrapTo2Pi(phi1-phi2)+2*k*M_PI)/(del2*w)))));
    double sin_val = sin(del1*w*t+phi1);

    return -E*del1*w*sin_val+F*del1*w*cos(del1*w*t + phi1)+ v*vw*((del1/del2)-1)*sin_val;
}

double trochoids::Trochoid::newtonRaphson(double x, double k, int idx_max)
{
    double h = func(x, k) / derivfunc(x, k);  // Line search
    int iter = 0;
    while (abs(h) >= EPSILON)
    {
        h = func(x, k)/derivfunc(x, k);

        iter++;
        if (iter > idx_max)
        {
            break;
        }
        x = x - h;
    }
    return x;
}
