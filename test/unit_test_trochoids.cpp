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

/* Authors: Sagar Sachdev, Brady Moon*/

#include <gtest/gtest.h>
#include <chrono>
#include <random>
#include <iostream>

#include "trochoids/trochoid_utils.h"
#include "unit_test_trochoid_classification.cpp"
#include "trochoids/trochoids.h"



std::vector<int> helper_fxn(std::vector<trochoids::XYZPsiState> &trochoid_path)
{
    std::vector<int> path_type;
    // std::cout << trochoid_path.getStateCount() << std::endl;
    int curr_seg = -1;
    for (int k = 0; k < trochoid_path.size()-1; ++k)
    {
        double psi1 = trochoid_path[k].psi;
        double psi2 = trochoid_path[k+1].psi;
        double difference = psi1 - psi2;

        if (abs(difference) < 1e-5)
        {
            // STRAIGHT = 1
            if (curr_seg == 1)
                continue;
            path_type.push_back(1);
            curr_seg = 1;   
        }
        else if (difference > M_PI)
        {
            // LEFT = 0
            if (curr_seg == 0)
                continue;
            path_type.push_back(0);
            curr_seg = 0;
        }
        else if (difference < -M_PI)
        {
            // RIGHT = 0
            if (curr_seg == 2)
                continue;
            path_type.push_back(2);
            curr_seg = 2;
        }
        else if (difference < 0)
        {
            // LEFT = 0
            if (curr_seg == 0)
                continue;
            path_type.push_back(0);
            curr_seg = 0;
        }
        else
        {
            // RIGHT = 2
            if (curr_seg == 2)
                continue;
            path_type.push_back(2);
            curr_seg = 2;
        }
    }
    // Print path type
    // for (int i = 0; i < path_type.size(); ++i)
    // {
    //     std::cout << path_type[i] << ", ";
    // }
    // std::cout << std::endl;

    return path_type;
}

// Straight Line test
TEST(TestTrochoids, trochoid_analytical_straight)
{
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1000) < 11);
}

TEST(TestTrochoids, trochoid_numerical_straight)
{
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1000) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1000) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

TEST(TestTrochoids, trochoid_analytical_straight_y)
{
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {0, 1000, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);
    
    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1085) < 11);
}

TEST(TestTrochoids, trochoid_numerical_straight_y)
{
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {0, 1000, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1085) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1085) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

// // Large radius Test
TEST(TestTrochoids, trochoid_large_radius_analytical)
{
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 6352) < 11);
    EXPECT_TRUE(valid);
}
TEST(TestTrochoids, trochoid_large_radius_numerical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 6353) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 6353) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}
TEST(TestTrochoids, trochoid_test_max_kappa_limit_analytical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 118) < 11);
    EXPECT_TRUE(valid);
}

TEST(TestTrochoids, trochoid_test_max_kappa_limit_numerical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 118) < 11);
    EXPECT_TRUE(valid);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 118) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);

}

TEST(TestTrochoids, trochoid_large_radius_slow_analytical)
{
    double wind[3] = {0, 0, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 6353) < 11);
}

TEST(TestTrochoids, trochoid_large_radius_slow_numerical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 6353) < 11);
     
    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 6353) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

TEST(TestTrochoids, trochoid_analytical_rsr)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1425) < 11);
}

TEST(TestTrochoids, trochoid_test_ccc_numerical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 28;
    double max_kappa = 0.00576283;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {-236.114, 541.739, 0, 1.29849};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);

    trochoid_path.clear();
    valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
}

TEST(TestTrochoids, trochoid_test_ccc_analytical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 28;
    double max_kappa = 0.00576283;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {-236.114, 541.739, 0, 1.29849};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
}

TEST(TestTrochoids, trochoid_rsr_numerical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1425) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1425) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

TEST(TestTrochoids, trochoid_analytical_rsl)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1425) < 11);
}

TEST(TestTrochoids, trochoid_numerical_rsl)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1425) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1425) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

// /*---------------------------------- Adding tests for different wind speeds ------------------------------------------------------------------*/

// // Check if wind speeds are lower than the desired speed
TEST(TestTrochoids, trochoid_large_radius_analytical_wind)
{
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 7394) < 11);
    EXPECT_TRUE(valid);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 7394) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}
TEST(TestTrochoids, trochoid_large_radius_numerical_wind)
{
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 7394) < 11);
    EXPECT_TRUE(valid);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 7394) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}
TEST(TestTrochoids, trochoid_test_max_kappa_limit_analytical_wind)
{
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 116) < 11);
}

TEST(TestTrochoids, trochoid_test_max_kappa_limit_numerical_wind)
{
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 116) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 116) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}
TEST(TestTrochoids, trochoid_large_radius_slow_analytical_wind)
{
    
    double wind[3] = {0.5, 0.75, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;

    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 8473) < 11);
}

TEST(TestTrochoids, trochoid_large_radius_slow_numerical_wind)
{
    
    double wind[3] = {0.5, 0.75, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 8473) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 8473) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

// // /*---------------------------------- Adding tests for different altitudes ------------------------------------------------------------------*/

// Straight Line test
TEST(TestTrochoids, trochoid_analytical_straight_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 50, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path; 
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1000) < 11);
}

TEST(TestTrochoids, trochoid_numerical_straight_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1000) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1000) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

// Large radius Test
TEST(TestTrochoids, trochoid_large_radius_analytical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 50, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 6352) < 11);
    EXPECT_TRUE(valid);
}
TEST(TestTrochoids, trochoid_large_radius_numerical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 50, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 6353) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 6353) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}
TEST(TestTrochoids, trochoid_test_max_kappa_limit_analytical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 50, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(dist - 130) < 11);
    EXPECT_TRUE(valid);
}

TEST(TestTrochoids, trochoid_test_max_kappa_limit_numerical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 50, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);
   
    EXPECT_TRUE(abs(dist - 130) < 11);
    EXPECT_TRUE(valid);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 130) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);

}

TEST(TestTrochoids, trochoid_large_radius_slow_analytical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 50, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;

    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 6353) < 11);
}

TEST(TestTrochoids, trochoid_large_radius_slow_numerical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 50, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 6353) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 6353) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

TEST(TestTrochoids, trochoid_analytical_rsr_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 50, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    // double dist = trochoids::get_length(start_n.get(), goal_n.get(), wind, desired_speed, max_kappa);
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1430) < 11);
}

TEST(TestTrochoids, trochoid_rsr_numerical_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 50, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1430) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1430) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

TEST(TestTrochoids, trochoid_analytical_rsl_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 50, 1.5707};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1431) < 11);
}

TEST(TestTrochoids, trochoid_numerical_rsl_altitude)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 50, 1.5707};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
    EXPECT_TRUE(abs(dist - 1431) < 11);

    trochoid_path.clear();
    bool valid_exhaustive_only = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, true);
    double dist_exhaustive_only = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_exhaustive_only);
    EXPECT_TRUE(abs(dist_exhaustive_only - 1431) < 11);
    EXPECT_TRUE(abs(dist - dist_exhaustive_only) < 1);
}

// /*---------------------------------------------------Random Methods Unit Tests--------------------------------------------------------------*/


TEST(TestTrochoids, trochoid_compare_methods_random_analytical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 28;
    double max_kappa = .05;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.001, 0.1);
    // std::uniform_real_distribution<> speedRange(-1000, 1000);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    for (int i = 0; i < 100; i++)
    {   
        if(i % 1000 == 0 && i != 0)
            std::cout << "Iteration number: " << i << std::endl;
        
        max_kappa = kappaRange(gen);
        Dubins::DubinsStateSpace::DubinsState start_state = {disRange(gen), disRange(gen), disPhi(gen)};
        Dubins::DubinsStateSpace::DubinsState goal_state = {disRange(gen), disRange(gen), disPhi(gen)};

        Dubins::DubinsStateSpace::DubinsPath dubins_path_matrix;
        Dubins::DubinsStateSpace::DubinsPath dubins_path;
        Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

        dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
        dubins_path = dubins_path_object.dubins(start_state, goal_state);

        double dubins_matrix_path_length = dubins_path_matrix.length()/max_kappa;
        double dubins_path_length = dubins_path.length()/max_kappa;
        EXPECT_TRUE(abs(dubins_matrix_path_length/dubins_path_length-1.0) < 0.05);

        if (dubins_path.type_[1] == Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT || 
            dubins_path.type_[1] == Dubins::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT)
        {
            continue;
        }

        trochoids::XYZPsiState start_stateT = {start_state.x, start_state.y, 0, start_state.theta};
        trochoids::XYZPsiState goal_stateT = {goal_state.x, goal_state.y, 0, goal_state.theta};
        // start_time = ompl::time::now();
        // double value = trochoids::get_length(start_n.get(), goal_n.get(), wind, desired_speed, max_kappa);
        std::vector<trochoids::XYZPsiState> trochoid_path;
        bool valid1 = trochoids::get_trochoid_path(start_stateT, goal_stateT, trochoid_path, wind, desired_speed, max_kappa);
        double value = trochoids::get_length(trochoid_path);
        // old_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        // start_time = ompl::time::now();
        std::vector<trochoids::XYZPsiState> trochoid_path2;
        bool valid2 = trochoids::get_trochoid_path_numerical(start_stateT, goal_stateT, trochoid_path2, wind, desired_speed, max_kappa);
        double value_new = trochoids::get_length(trochoid_path2);
        // new_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        std::vector<trochoids::XYZPsiState> trochoid_path3;
        bool valid3 = trochoids::get_trochoid_path_numerical(start_stateT, goal_stateT, trochoid_path3, wind, desired_speed, max_kappa, true);
        double value3 = trochoids::get_length(trochoid_path3);
        
        // std::cout << "Length is: " << value << " vs. " << value_new << std::endl;
        // std::cout << "Value1: " << value << " Value2: " << value_new << " Value3: " << value3 << std::endl;
        EXPECT_TRUE((value/dubins_path_length-1.0) < 0.05);
        if ((value/dubins_path_length-1.0) > 0.05)
        {
            std::cout << "Start: " << start_stateT.x << ", " << start_stateT.y << ", " << start_stateT.psi << std::endl;
            std::cout << "Goal: " << goal_stateT.x << ", " << goal_stateT.y << ", " << goal_stateT.psi << std::endl;
            std::cout << "Wind: " << wind[0] << ", " << wind[1] << ", " << wind[2] << std::endl;
            std::cout << "Speed: " << desired_speed << std::endl;
            std::cout << "Kappa: " << max_kappa << std::endl;
            std::cout << "Value1: " << value << " Value2: " << value_new << " Value3: " << value3 << std::endl;
            std::cout << "Dubins length: " << dubins_path_length << " dubins matrix length: " << dubins_matrix_path_length << std::endl;
            continue;
        }


        EXPECT_TRUE((value/value_new-1.0) < 0.05);
        if ((value/value_new-1.0) > 0.05)
        {
            std::cout << "Start: " << start_stateT.x << ", " << start_stateT.y << ", " << start_stateT.psi << std::endl;
            std::cout << "Goal: " << goal_stateT.x << ", " << goal_stateT.y << ", " << goal_stateT.psi << std::endl;
            std::cout << "Wind: " << wind[0] << ", " << wind[1] << ", " << wind[2] << std::endl;
            std::cout << "Speed: " << desired_speed << std::endl;
            std::cout << "Kappa: " << max_kappa << std::endl;
            std::cout << "Value1: " << value << " Value2: " << value_new << " Value3: " << value3 << std::endl;
            std::cout << "Dubins length: " << dubins_path_length << " dubins matrix length: " << dubins_matrix_path_length << std::endl;
            continue;
        }
        EXPECT_TRUE((value3/value_new-1.0) < 0.05);
        if ((value3/value_new-1.0) > 0.05)
        {
            std::cout << "Start: " << start_stateT.x << ", " << start_stateT.y << ", " << start_stateT.psi << std::endl;
            std::cout << "Goal: " << goal_stateT.x << ", " << goal_stateT.y << ", " << goal_stateT.psi << std::endl;
            std::cout << "Wind: " << wind[0] << ", " << wind[1] << ", " << wind[2] << std::endl;
            std::cout << "Speed: " << desired_speed << std::endl;
            std::cout << "Kappa: " << max_kappa << std::endl;
            std::cout << "Value1: " << value << " Value2: " << value_new << " Value3: " << value3 << std::endl;
            std::cout << "Dubins length: " << dubins_path_length << " dubins matrix length: " << dubins_matrix_path_length << std::endl;
            continue;
        }
        EXPECT_TRUE(valid1);
        EXPECT_TRUE(valid2);
        EXPECT_TRUE(valid3);
    }
    // std::cout << "Old method time: " << old_method_time << std::endl;
    // std::cout << "New method time: " << new_method_time << std::endl;
}

TEST(TestTrochoids, trochoid_compare_methods_random_wind)
{
    
    double desired_speed = 50;
    double max_kappa = .1;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.001, 0.1);
    std::uniform_real_distribution<> disWind(-35, 35);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    for (int i = 0; i < 100; i++)
    {   
        if(i % 1000 == 0 && i != 0)
            std::cout << "Iteration number: " << i << std::endl;
        
        max_kappa = kappaRange(gen);
        trochoids::XYZPsiState start_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        trochoids::XYZPsiState goal_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        double wind[3] = {disWind(gen), disWind(gen), 0};

        // start_time = ompl::time::now();
        std::vector<trochoids::XYZPsiState> trochoid_path;
        bool valid1 = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
        EXPECT_TRUE(valid1);
        if(!valid1)
        {
            std::cout << "Invalid path" << std::endl;
            std::cout << "Start: " << start_state.x << ", " << start_state.y << ", " << start_state.psi << std::endl;
            std::cout << "Goal: " << goal_state.x << ", " << goal_state.y << ", " << goal_state.psi << std::endl;
            std::cout << "Wind: " << wind[0] << ", " << wind[1] << ", " << wind[2] << std::endl;
            std::cout << "Speed: " << desired_speed << std::endl;
            std::cout << "Kappa: " << max_kappa << std::endl;
            continue;
        }
        double value = trochoids::get_length(trochoid_path);
        // old_method_time += ompl::time::seconds(ompl::time::now() - start_time);
        // start_time = ompl::time::now();
        std::vector<trochoids::XYZPsiState> trochoid_path2;
        bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
        EXPECT_TRUE(valid2);
        double value_new = trochoids::get_length(trochoid_path2);
        // new_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        std::vector<trochoids::XYZPsiState> trochoid_path3;
        bool valid3 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path3, wind, desired_speed, max_kappa, true);
        EXPECT_TRUE(valid3);
        double value3 = trochoids::get_length(trochoid_path3);

        EXPECT_TRUE((value/value_new-1.0) < 0.05);
        if ((value/value_new-1.0) > 0.05)
        {
            std::cout << "Start: " << start_state.x << ", " << start_state.y << ", " << start_state.psi << std::endl;
            std::cout << "Goal: " << goal_state.x << ", " << goal_state.y << ", " << goal_state.psi << std::endl;
            std::cout << "Wind: " << wind[0] << ", " << wind[1] << ", " << wind[2] << std::endl;
            std::cout << "Speed: " << desired_speed << std::endl;
            std::cout << "Kappa: " << max_kappa << std::endl;
            std::cout << "Value1: " << value << " Value2: " << value_new << " Value3: " << value3 << std::endl;
            continue;
        }
        EXPECT_TRUE((value3/value_new-1.0) < 0.05);
    }
}
typedef std::vector<std::tuple<double, double, double>> Path;
TEST(TestTrochoids, DISABLED_trochoid_compare_methods_random_wind_varkappa)
{
    
    double desired_speed = 20;
    double max_kappa = 0.01;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> kappaRange(0.01, 0.1);

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    double wind[3] = {5, -5, 0};

    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {wind[0], wind[1]};
    double ang_rate = desired_speed/(1.0/max_kappa);

    trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
    trochoid.problem.Xf = {goal_state.x , goal_state.y, goal_state.psi};

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++)
    {   
        // if(i % 1000 == 0)
        //     std::cout << "Iteration number: " << i << std::endl;
        
        max_kappa = kappaRange(gen);
        trochoid.problem.max_kappa = max_kappa;
        Path path = trochoid.getTrochoid();
    }
    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
    std::cout << "Total time: " << duration.count() << " ms" << std::endl;
}

TEST(TestTrochoids, DISABLED_trochoid_compare_methods_random_wind_varpos)
{
    
    double desired_speed = 20;
    double max_kappa = 0.01; // set max_kappa here
    
    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.v = desired_speed;
    
    double ang_rate = desired_speed/(1.0/max_kappa);
    

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    double wind[3] = {5,-5,0}; // to be edited
    trochoid.problem.wind = {wind[0], wind[1]};

    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++)
    {   
        // if(i % 1000 == 0)
        //     std::cout << "Iteration number: " << i << std::endl;
        trochoids::XYZPsiState start_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        trochoids::XYZPsiState goal_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};

        trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
        trochoid.problem.Xf = {goal_state.x, goal_state.y, goal_state.psi};

        Path path = trochoid.getTrochoid();
    }

    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
    std::cout << "Total time: " << duration.count() << " ms" << std::endl;
}

TEST(TestTrochoids, DISABLED_trochoid_compare_methods_random_wind_varwind)
{
    
    double desired_speed = 20;
    double max_kappa = 0.01;
    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.max_kappa = max_kappa;
    trochoid.problem.v = desired_speed;
    double ang_rate = desired_speed/(1.0/max_kappa);

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disWind(-35, 35);

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
    trochoid.problem.Xf = {goal_state.x, goal_state.y, goal_state.psi};

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++)
    {   
        // if(i % 1000 == 0)
        //     std::cout << "Iteration number: " << i << std::endl;
        
        double wind[3] = {disWind(gen), disWind(gen), 0};
        trochoid.problem.wind = {wind[0], wind[1]};
        Path path = trochoid.getTrochoid();

    }
    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
    std::cout << "Total time: " << duration.count() << " ms" << std::endl;
}

TEST(TestTrochoids, DISABLED_trochoid_compare_methods_random_wind_var)
{
    
    double desired_speed = 20;
    double max_kappa = 0.01;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.01, 0.1);
    std::uniform_real_distribution<> disWind(-35, 35);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);

    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.v = desired_speed;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 100; i++)
    {   
        // if(i % 1000 == 0)
        //     std::cout << "Iteration number: " << i << std::endl;
        
        max_kappa = kappaRange(gen);
        trochoid.problem.max_kappa = max_kappa;
        double ang_rate = desired_speed/(1.0/max_kappa);

        trochoids::XYZPsiState start_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        trochoids::XYZPsiState goal_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};

        trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
        trochoid.problem.Xf = {goal_state.x, goal_state.y, goal_state.psi};

         double wind[3] = {disWind(gen), disWind(gen), 0};
        trochoid.problem.wind = {wind[0], wind[1]};

        // start_time = ompl::time::now();
        Path path = trochoid.getTrochoid();
    }
    auto finish = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
    std::cout << "Total time: " << duration.count() << " ms" << std::endl;
}

TEST(TestTrochoids, test1)
{
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 50;
    double max_kappa = 0.1;

    trochoids::XYZPsiState start_state = {0, 0, 0, 2};
    trochoids::XYZPsiState goal_state = {1000, 0, 0, -1};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double value = trochoids::get_length(trochoid_path);

    trochoid_path.clear();
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double value2 = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(valid);
}

TEST(TestTrochoids, DISABLED_trochoid_compare_methods_random_numerical)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 28;
    double max_kappa = .05;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.001, 0.1);
    // std::uniform_real_distribution<> speedRange(-1000, 1000);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    for (int i = 0; i < 100; i++)
    {   
        max_kappa = kappaRange(gen);

        trochoids::XYZPsiState start_state = {0, 0, 0, 0};
        trochoids::XYZPsiState goal_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};

        // start_time = ompl::time::now();
        // double value = trochoids::get_length(start_n.get(), goal_n.get(), wind, desired_speed, max_kappa);
        std::vector<trochoids::XYZPsiState> trochoid_path1;
        bool valid1 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path1, wind, desired_speed, max_kappa);
        double value = trochoids::get_length(trochoid_path1);
        // old_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        // start_time = ompl::time::now();
        std::vector<trochoids::XYZPsiState> trochoid_path2;
        bool valid2 = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
        double value2 = trochoids::get_length(trochoid_path2);
        // new_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        std::vector<trochoids::XYZPsiState> trochoid_path3;
        bool valid3 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path3, wind, desired_speed, max_kappa, true);
        double value3 = trochoids::get_length(trochoid_path3);
        
        // std::cout << "Length is: " << value << " vs. " << value_new << std::endl;
        EXPECT_TRUE((value/value2-1.0) < 0.05);
        EXPECT_TRUE((value2/value3-1.0) < 0.05);
        EXPECT_TRUE(valid1);
        EXPECT_TRUE(valid2);
        EXPECT_TRUE(valid3);
    }
    // std::cout << "Old method time: " << old_method_time << std::endl;
    // std::cout << "New method time: " << new_method_time << std::endl;
}

TEST(TestTrochoids, DISABLED_trochoid_compare_methods_random_wahoo_random)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 28;
    double max_kappa = .05;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.005, 0.01);
    // std::uniform_real_distribution<> speedRange(-1000, 1000);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    // auto start_time = ompl::time::now();

    // double old_method_time = 0;
    // double new_method_time = 0;

    for (int i = 0; i < 100; i++)
    {   
        max_kappa = kappaRange(gen);
        trochoids::XYZPsiState start_state = {0, 0, 0, 0};
        trochoids::XYZPsiState goal_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        
        // start_time = ompl::time::now();
        std::vector<trochoids::XYZPsiState> trochoid_path1;
        bool valid1 = trochoids::get_trochoid_path(start_state,goal_state, trochoid_path1, wind, desired_speed, max_kappa);
        double value = trochoids::get_length(trochoid_path1);
        // std::cout << "Number of points in analytical (dubins): " << trochoid_path1.getStateCount() << std::endl;
        // std::cout << "Length_analytical_dubins: " << value << std::endl;
        // old_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        // start_time = ompl::time::now();
        std::vector<trochoids::XYZPsiState> trochoid_path2;
        bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
        double value2 = trochoids::get_length(trochoid_path2);
        // std::cout << "Number of points in numerical (dubins): " << trochoid_path2.getStateCount() << std::endl;
        // std::cout << "Length_numerical_dubins: " << value2 << std::endl;
        // new_method_time += ompl::time::seconds(ompl::time::now() - start_time);

        std::vector<trochoids::XYZPsiState> trochoid_path3;
        bool valid3 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path3, wind, desired_speed, max_kappa, true);
        double value3 = trochoids::get_length(trochoid_path3);
        // std::cout << "Number of points in numerical (exhaustive): " << trochoid_path3.getStateCount() << std::endl;
        // std::cout << "Length_numerical_exhaustive: " << value3 << std::endl;

        // if (abs((value2/value3)-1.0) > 0.05) // Finds CCC cases
        // {
        //     std::cout << "value2/value3 - 1 is greater than 0.05" << std::endl;
        //     std::cout << "condition output: " << abs((value2/value3)-1.0) << std::endl;
        //     std::cout << "Value1: " << value << std::endl;
        //     std::cout << "Value2: " << value2 << std::endl;
        //     std::cout << "Value3: " << value3 << std::endl;
        //     std::cout << "Kappa: " << max_kappa << std::endl;

        //     std::cout << "start: " << start_n->getX() << ", " << start_n->getY() << ", " << start_n->getPsi() << std::endl;
        //     std::cout << "goal: " << goal_n->getX() << ", " << goal_n->getY() << ", " << goal_n->getPsi() << std::endl;
        // }

        // if (abs((value/value3)-1.0) > 0.01) // Finds CCC cases
        // {
        //     std::cout << "value/value3 - 1 is greater than 0.01" << std::endl;
        //     std::cout << "condition output: " << abs((value2/value3)-1.0) << std::endl;
        //     std::cout << "Value1: " << value << std::endl;
        //     std::cout << "Value2: " << value2 << std::endl;
        //     std::cout << "Value3: " << value3 << std::endl;
        //     std::cout << "Kappa: " << max_kappa << std::endl;

        //     std::cout << "start: " << start_n->getX() << ", " << start_n->getY() << ", " << start_n->getPsi() << std::endl;
        //     std::cout << "goal: " << goal_n->getX() << ", " << goal_n->getY() << ", " << goal_n->getPsi() << std::endl;
        // }
        if ((abs(value/value3) - 1.0) > 0.05)
        {
            std::cout << "value/value3 - 1 = " << (abs(value/value3) - 1.0) << std::endl;
        } 

        if ((abs(value2/value3) - 1.0) > 0.05)
        {
            std::cout << "value2/value3 - 1 = " << (abs(value2/value3) - 1.0) << std::endl;
        } 

        EXPECT_TRUE(((value/value3)-1.0) < 0.05);
        EXPECT_TRUE(((value2/value3)-1.0) < 0.05);
        EXPECT_TRUE(valid1);
        EXPECT_TRUE(valid2);
        EXPECT_TRUE(valid3);
    }
}

TEST(TestTrochoids, trochoid_analytical_failure_case1)
{
    
    double wind[3] = {-13.3915, 24.8759, 0};
    double desired_speed = 50;
    double max_kappa = 0.0500199;

    trochoids::XYZPsiState start_state = {-344.497, 221.32, 0, 0.404871};
    trochoids::XYZPsiState goal_state = {-852.201, -2.55509, 0, 2.24239};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist = trochoids::get_length(trochoid_path);
    
    EXPECT_TRUE(valid);

    trochoid_path.clear();
    bool valid_numerical = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    double dist_numerical = 0;
    if (valid_numerical)
        dist_numerical = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(valid_numerical);
    EXPECT_TRUE((dist - dist_numerical) < 1);
}


TEST(TestTrochoidsDecimation, test_decimation_by_distance)
{
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 28;
    double max_kappa = 0.0075;

    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.001, 0.1);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);
    double distance_thresh = 10;

    auto euclidean_distance = [](trochoids::XYZPsiState state1, trochoids::XYZPsiState state2)
    {
        std::vector<double> diff = {state1.x - state2.x, state1.y - state2.y, state1.z - state2.z};
        return sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
    };

    
    for (int i = 0; i < 1000; i++)
    {   
        max_kappa = kappaRange(gen);
        
        trochoids::XYZPsiState start_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        trochoids::XYZPsiState goal_state = {disRange(gen), disRange(gen), 0, disPhi(gen)};
        
        
        std::vector<trochoids::XYZPsiState> trochoid_path;
        std::vector<trochoids::XYZPsiState> trochoid_path2;
        // Test without wind
        wind[0] = 0;
        bool valid1 = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, distance_thresh);
        double value = trochoids::get_length(trochoid_path);

        // std::cout << "Without wind" << std::endl;
        EXPECT_TRUE(valid1);
        for (int i = 0; i < trochoid_path.size() - 1; i++)
        {
            double dist = euclidean_distance(trochoid_path[i], 
                                             trochoid_path[i+1]);
            // std::cout << "dist: " << dist << std::endl;
            EXPECT_TRUE(dist <= distance_thresh + 0.001);
            if (dist > distance_thresh + 0.001)
            {
                std::cout << "Distance: " << dist << std::endl;
                std::cout << "Max kappa: " << max_kappa << std::endl;
                std::cout << "Start psi: " << start_state.psi << std::endl;
                std::cout << "Goal psi: " << goal_state.psi << std::endl;
            }
        }
        // Test the final point is the goal
        EXPECT_TRUE(euclidean_distance(trochoid_path[trochoid_path.size() - 1], goal_state) < 0.001);
        double final_dist =euclidean_distance(trochoid_path[trochoid_path.size() - 1], goal_state);
        EXPECT_TRUE(final_dist < 0.001);
        if (final_dist >= 0.001)
        {
            std::cout << "Max kappa: " << max_kappa << std::endl;
            std::cout << "Start: " << start_state.x << ", " << start_state.y << ", " << start_state.z << std::endl;
            std::cout << "Start psi: " << start_state.psi << std::endl;
            std::cout << "Goal: " << goal_state.x << ", " << goal_state.y << ", " << goal_state.z << std::endl;
            std::cout << "Goal psi: " << goal_state.psi << std::endl;
            std::cout << "Final dist: " << final_dist << std::endl;
            std::cout << "Final: " << trochoid_path2[trochoid_path2.size() - 1].x << ", " << trochoid_path2[trochoid_path2.size() - 1].y << ", " << trochoid_path2[trochoid_path2.size() - 1].z << ", " << trochoid_path2[trochoid_path2.size() - 1].psi << std::endl;
        }
        
        
        
        // Test with wind
        wind[0] = 0.001;
        valid1 = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa, distance_thresh);
        value = trochoids::get_length(trochoid_path2);
        // std::cout << "With wind" << std::endl;
        EXPECT_TRUE(valid1);
        for (int i = 0; i < trochoid_path2.size() - 1; i++)
        {
            double dist = euclidean_distance(trochoid_path2[i], 
                                             trochoid_path2[i+1]);
            // std::cout << "dist: " << dist << std::endl;
            EXPECT_TRUE(dist <= distance_thresh + 0.001);
            if (dist > distance_thresh + 0.001)
            {
                std::cout << "Distance: " << dist << std::endl;
                std::cout << "Max kappa: " << max_kappa << std::endl;
                std::cout << "Start: " << start_state.x << ", " << start_state.y << ", " << start_state.z << std::endl;
                std::cout << "Start psi: " << start_state.psi << std::endl;
                std::cout << "Goal: " << goal_state.x << ", " << goal_state.y << ", " << goal_state.z << std::endl;
                std::cout << "Goal psi: " << goal_state.psi << std::endl;
            }
        }
        // Test the final point is the goal
        double final_dist2 =(euclidean_distance(trochoid_path2[trochoid_path2.size() - 1], goal_state));
        EXPECT_TRUE(final_dist2 < 0.001);
        if (final_dist2 >= 0.001)
        {
                // std::cout << "Distance: " << dist << std::endl;
                std::cout << "Max kappa: " << max_kappa << std::endl;
                std::cout << "Start: " << start_state.x << ", " << start_state.y << ", " << start_state.z << std::endl;
                std::cout << "Start psi: " << start_state.psi << std::endl;
                std::cout << "Goal: " << goal_state.x << ", " << goal_state.y << ", " << goal_state.z << std::endl;
                std::cout << "Goal psi: " << goal_state.psi << std::endl;
            
            std::cout << "Final dist: " << final_dist2 << std::endl;
            std::cout << "Final: " << trochoid_path2[trochoid_path2.size() - 1].x << ", " << trochoid_path2[trochoid_path2.size() - 1].y << ", " << trochoid_path2[trochoid_path2.size() - 1].z << ", " <<  trochoid_path2[trochoid_path2.size() - 1].psi << std::endl;
        }

    }
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}