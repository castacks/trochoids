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
#include "trochoids/trochoid_utils.h"
#include <iostream>
#include <fstream>

#define HOME_PATH "/<path-to-where-you-want-to-save-files>"

void path_to_file(std::vector<trochoids::XYZPsiState> &trochoid_path, std::string path_name)
{
    std::ofstream myfile;
    myfile.open(path_name);
    for (int i = 0; i < trochoid_path.size(); i++)
    {
        myfile << trochoid_path[i].x << ", " << trochoid_path[i].y << ", " << trochoid_path[i].z << ", " << trochoid_path[i].psi << std::endl;
    }
    myfile.close();
}


class TrochoidTestFixture: public ::testing::Test { 
protected:
    double wind[3];
    Dubins::DubinsStateSpace::DubinsPath dubins_path;
    Dubins::DubinsStateSpace::DubinsPath dubins_path_matrix;
    double desired_speed;
    double max_kappa;
    double big_epsilon = 0.5;

public: 
   TrochoidTestFixture( ) { 
        // initialization code here
        wind[0] = 0;
        wind[1] = 0;
        wind[2] = 0;

        desired_speed = 50;
        max_kappa = 0.1;
   } 
};

class DubinsTestFixture: public ::testing::Test { 
protected:
    double wind[3];
    Dubins::DubinsStateSpace::DubinsPath dubins_path;
    Dubins::DubinsStateSpace::DubinsPath dubins_path_matrix;
    double desired_speed;
    double max_kappa;
    double big_epsilon = 0.5;

public: 
   DubinsTestFixture( ) { 
        // initialization code here
        wind[0] = 0;
        wind[1] = 0;
        wind[2] = 0;

        desired_speed = 50;
        max_kappa = 0.1;
   } 
};

// // No wind conditions
TEST_F(TrochoidTestFixture, DISABLED_no_wind)
{
    Dubins::DubinsStateSpace::DubinsState start_state = {-13, 5, 0};
    Dubins::DubinsStateSpace::DubinsState goal_state = {1010, -40, 0};
    Dubins::DubinsStateSpace::DubinsPath dubins_path;
    
    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa); 

    for (double i = -M_PI; i < M_PI; i+=1.05)
    {
        for (double j = -M_PI; j < M_PI; j+=1.05)
        {
            start_state.theta = i;
            goal_state.theta = j;

            dubins_path_matrix = dubins_path_obj.dubins_matrix(start_state, goal_state);
            dubins_path = dubins_path_obj.dubins(start_state, goal_state);
            EXPECT_FLOAT_EQ(dubins_path.length(), dubins_path_matrix.length());

            // dubins_path.clear();
            trochoids::XYZPsiState start_state1 = {start_state.x, start_state.y, 0, start_state.theta};
            trochoids::XYZPsiState goal_state1 = {goal_state.x, goal_state.y, 0, goal_state.theta};
            std::vector<trochoids::XYZPsiState> trochoid_path;
            trochoid_path.clear();
            bool valid = trochoids::get_trochoid_path(start_state1, goal_state1, trochoid_path, wind, desired_speed, max_kappa);
            EXPECT_TRUE(valid);

            EXPECT_TRUE(abs(dubins_path.length()/max_kappa - trochoids::get_length(trochoid_path)) < big_epsilon);
        }
    }
}


TEST_F(TrochoidTestFixture, DISABLED_wind_trochoid2)
{
    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 0, 0};
    std::vector<trochoids::XYZPsiState> trochoid_path;

    double wind[3] = {22, 44, 0};

    for (double i = -M_PI; i < M_PI; i+=1.05)
    {
        for (double j = -M_PI; j < M_PI; j+=1.05)
        {
            start_state.psi = i;
            goal_state.psi = j;

            trochoid_path.clear();
            bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
            EXPECT_TRUE(valid); 
        }
    }
}

TEST_F(TrochoidTestFixture, wind_trochoid_single)
{

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {100, 100, 0, M_PI/6};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {30, 0, 0};
    
    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
}

// Check for when the psi, parallel, or orthogonal so there would be duplicate
// Check below
// TEST(DecisionPoints, goal_above_right)
// {
//     double x0 = 0;
//     double y0 = 0;
//     double xf = 100;
//     double yf = 100;
//     double psi1_trochoidal = 0;
//     double psi2_trochoidal = M_PI/6;
    
//     std::vector<double> d_values = trochoids::Trochoid::decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);

//     EXPECT_EQ(d_values.size(), 3);
//     if(d_values.size() == 3)
//     {
//         EXPECT_FLOAT_EQ(d_values[0], 0);
//         EXPECT_FLOAT_EQ(d_values[1], 100);
//         EXPECT_FLOAT_EQ(d_values[2], 157.735026919);
//     }
// }

// TEST(DecisionPoints, goal_below_right)
// {
//     double x0 = 0;
//     double y0 = 0;
//     double xf = 100;
//     double yf = -100;
//     double psi1_trochoidal = 0;
//     double psi2_trochoidal = -M_PI/6;
    
//     std::vector<double> d_values = trochoids::Trochoid::decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);

//     EXPECT_EQ(d_values.size(), 3);
//     if(d_values.size() == 3)
//     {
//         EXPECT_FLOAT_EQ(d_values[0], 0);
//         EXPECT_FLOAT_EQ(d_values[1], 100);
//         EXPECT_FLOAT_EQ(d_values[2], 157.735026919);
//     }
// }

// TEST(DecisionPoints, goal_above_left)
// {
//     double x0 = 0;
//     double y0 = 0;
//     double xf = -100;
//     double yf = 100;
//     double psi1_trochoidal = 0;
//     double psi2_trochoidal = M_PI/6;
    
//     std::vector<double> d_values = trochoids::Trochoid::decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);

//     EXPECT_EQ(d_values.size(), 1);
//     if(d_values.size() == 1)
//     {
//         EXPECT_FLOAT_EQ(d_values[0], 0);
//     }
// }

// TEST(DecisionPoints, goal_below_left)
// {
//     double x0 = 0;
//     double y0 = 0;
//     double xf = -100;
//     double yf = -100;
//     double psi1_trochoidal = 0;
//     double psi2_trochoidal = -M_PI/6;
    
//     std::vector<double> d_values = trochoids::Trochoid::decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);

//     EXPECT_EQ(d_values.size(), 1);
//     if(d_values.size() == 1)
//     {
//         EXPECT_FLOAT_EQ(d_values[0], 0);
//     }
// }

// TEST(DecisionPoints, goal_above_start)
// {
//     double x0 = 0;
//     double y0 = 0;
//     double xf = 0;
//     double yf = 100;
//     double psi1_trochoidal = 0;
//     double psi2_trochoidal = M_PI/2;
    
//     std::vector<double> d_values = trochoids::Trochoid::decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);
//     EXPECT_EQ(d_values.size(), 1);
//     if(d_values.size() == 1)
//     {
//         EXPECT_TRUE(abs(d_values[0]) < EPSILON);
//     }
// }

// TEST(DecisionPoints, goal_below_start)
// {
//     double x0 = 0;
//     double y0 = 0;
//     double xf = 0;
//     double yf = -100;
//     double psi1_trochoidal = 0;
//     double psi2_trochoidal = -M_PI/2;
    
//     std::vector<double> d_values = trochoids::Trochoid::decision_pts(x0, y0, xf, yf, psi1_trochoidal, psi2_trochoidal);
//     EXPECT_EQ(d_values.size(), 1);
//     if(d_values.size() == 1)
//     {
//         EXPECT_TRUE(abs(d_values[0]) < EPSILON);
//     }
// }

TEST_F(DubinsTestFixture, dubins_matrix_test_random)
{
    // Call Random Start and Goal states:
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> kappaRange(0.005, 0.01);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);

    for (int i = 0; i < 100000; i++)
    {   
        // if(i % 10000 == 0 && i != 0)
        //     std::cout << "Iteration number: " << i << std::endl;

        max_kappa = kappaRange(gen);
        
        Dubins::DubinsStateSpace::DubinsState start_state = {disRange(gen), disRange(gen), disPhi(gen)};
        Dubins::DubinsStateSpace::DubinsState goal_state = {disRange(gen), disRange(gen), disPhi(gen)};

        Dubins::DubinsStateSpace::DubinsPath dubins_path_matrix;
        Dubins::DubinsStateSpace::DubinsPath dubins_path;
        Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

        dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
        dubins_path = dubins_path_object.dubins(start_state, goal_state);

        double dubins_matrix_path_length = dubins_path_matrix.length();
        double dubins_path_length = dubins_path.length();

        if (abs(dubins_matrix_path_length - dubins_path_length) > 1e-5)
        {
            std::cout << "Dubins Matrix Path Length: " << dubins_matrix_path_length << std::endl;
            std::cout << "Dubins Path Length: " << dubins_path_length << std::endl;
            std::cout << "Max_Kappa: " << max_kappa << std::endl;

            std::cout << "Start State: " << start_state.x << ", " << start_state.y << ", " << start_state.theta << std::endl;
            std::cout << "Goal State: " << goal_state.x << ", " << goal_state.y << ", " << goal_state.theta << std::endl;

            std::cout << "Dubins Matrix Path Type: " << dubins_path_matrix.type_[0] << ", " << dubins_path_matrix.type_[1] << ", " << dubins_path_matrix.type_[2] << std::endl;
            std::cout << "Dubins Path Type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;
        }
        EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) <= 1e-5);
    }
}

TEST_F(DubinsTestFixture, failure_case_dubins_matrix)
{
    // d = 5.5483172346109235
    // alpha = 0.27289972912311261
    // beta = 3.1388400363026987
    // a_1_2
    // LSR best

    trochoids::XYZPsiState start_state = {-661.176239, 236.542484, 0, 0.330952};
    trochoids::XYZPsiState goal_state = {-67.757473, 271.030542, 0, -3.086293};
    // std::vector<trochoids::XYZPsiState> trochoid_path;

    Dubins::DubinsStateSpace::DubinsState start_n = {start_state.x, start_state.y, start_state.psi};
    Dubins::DubinsStateSpace::DubinsState goal_n = {goal_state.x, goal_state.y, goal_state.psi};

    // double wind[3] = {20.424, -20.0901, 0};
    
    desired_speed = 50;
    max_kappa = 0.009334; // RADIUS = 111.11

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_n, goal_n);
    dubins_path = dubins_path_object.dubins(start_n, goal_n);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
    // std::cout << "Dubins Matrix value: " << dubins_matrix_path_length << std::endl;
    // std::cout << "Dubins value: " << dubins_path_length << std::endl;
}

TEST_F(DubinsTestFixture, failure_case_dubins_matrix2)
{
    // d = 4.3796238531375771
    // alpha = 3.1557258885631878
    // beta = 5.8465128885631881
    // a_3_4
    // RSL

    Dubins::DubinsStateSpace::DubinsState start_n = {707.809344, -865.235911, -0.194369};
    Dubins::DubinsStateSpace::DubinsState goal_n = {209.058309, -759.711596, 2.496418};
    
    desired_speed = 50;
    max_kappa = 0.008591; // RADIUS = 111.11

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_n, goal_n);
    dubins_path = dubins_path_object.dubins(start_n, goal_n);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
}

TEST_F(DubinsTestFixture, failure_case_dubins_matrix3)
{
    double d = 283.768; 
    double alpha = 0;
    double beta = 0.0100429;

    max_kappa = 0.068106897435311786; // 16.67
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {0,0, alpha};
    Dubins::DubinsStateSpace::DubinsState goal_state = {d*(1/max_kappa), 0, beta};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);
    
    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
    // std::cout << "dubins_path_length: " << dubins_path.length() << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;
}   

TEST_F(DubinsTestFixture, failure_case_dubins_matrix4)
{
    double d = 146.718732682216; 
    double alpha = 0;
    double beta = 0.0060383633327356634;

    max_kappa = 0.068106897435311786;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {528.31640768858404,-229.49382213258539, 2.3532546119395792};
    Dubins::DubinsStateSpace::DubinsState goal_state = {-990.47782251535614, 1298.2569051412538, 2.3592929752723149};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
    // std::cout << "dubins_path_length: " << dubins_path.length() << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;
}   

TEST_F(DubinsTestFixture, failure_case_dubins_four_r1_1)
{
    /*
    Dubins Matrix Path Length: 8.26153
    Dubins Path Length: 8.07563
    Max_Kappa: 0.00572078
    Start State: -51.074, -287.227, 0.888931
    Goal State: -607.936, -471.39, 1.94105
    Dubins Matrix Path Type: 2, 1, 2
    Dubins Path Type: 2, 0, 2
    */
    max_kappa = 0.00572078;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {-51.074, -287.227, 0.888931};
    Dubins::DubinsStateSpace::DubinsState goal_state = {-607.936, -471.39, 1.94105};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
} 

TEST_F(DubinsTestFixture, failure_case_dubins_four_r1_2)
{
    /*
    Dubins Matrix Path Length: 1.79769e+308
    Dubins Path Length: 6.60756
    Max_Kappa: 0.00982667
    Start State: -347.852, 202.189, 3.46437
    Goal State: -505.575, 148.801, 0.333584
    Dubins Matrix Path Type: 0, 1, 0
    Dubins Path Type: 2, 0, 2
    */
    max_kappa = 0.00982667;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {-347.852, 202.189, 3.46437};
    Dubins::DubinsStateSpace::DubinsState goal_state = {-505.575, 148.801, 0.333584};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
} 

TEST_F(DubinsTestFixture, dubins_a12)
{
    /*
    Dubins Matrix Path Length: 1.79769e+308
    Dubins Path Length: 6.60756
    Max_Kappa: 0.00982667
    Start State: -347.852, 202.189, 3.46437
    Goal State: -505.575, 148.801, 0.333584
    Dubins Matrix Path Type: 0, 1, 0
    Dubins Path Type: 2, 0, 2
    */
    max_kappa = 0.01;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {0, 0, 0.5};
    Dubins::DubinsStateSpace::DubinsState goal_state = {800, 0, 2.4};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
} 

TEST_F(DubinsTestFixture, dubins_a22)
{
    /*
    Dubins Matrix Path Length: 1.79769e+308
    Dubins Path Length: 6.60756
    Max_Kappa: 0.00982667
    Start State: -347.852, 202.189, 3.46437
    Goal State: -505.575, 148.801, 0.333584
    Dubins Matrix Path Type: 0, 1, 0
    Dubins Path Type: 2, 0, 2
    */
    max_kappa = 0.01;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {0, 0, 2.8};
    Dubins::DubinsStateSpace::DubinsState goal_state = {800, 0, 2.4};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
} 

TEST_F(DubinsTestFixture, dubins_a22_equal_alpha_beta)
{
    /*
    Dubins Matrix Path Length: 1.79769e+308
    Dubins Path Length: 6.60756
    Max_Kappa: 0.00982667
    Start State: -347.852, 202.189, 3.46437
    Goal State: -505.575, 148.801, 0.333584
    Dubins Matrix Path Type: 0, 1, 0
    Dubins Path Type: 2, 0, 2
    */
    max_kappa = 0.01;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {0, 0, 2.4};
    Dubins::DubinsStateSpace::DubinsState goal_state = {800, 0, 2.4};

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    dubins_path_matrix = dubins_path_object.dubins_matrix(start_state, goal_state);
    dubins_path = dubins_path_object.dubins(start_state, goal_state);

    double dubins_matrix_path_length = dubins_path_matrix.length();
    double dubins_path_length = dubins_path.length();

    EXPECT_TRUE(abs(dubins_matrix_path_length - dubins_path_length) < 1e-5);
} 

TEST_F(DubinsTestFixture, fail_case_8_tests)
{
    max_kappa = 0.066222699999999995;
    // Run dubins on this (should choose dubinsLSL and segfault)

    Dubins::DubinsStateSpace::DubinsState start_state = {-458.78368113158683, 382.34142866860554, 2.3950568527085956};
    Dubins::DubinsStateSpace::DubinsState goal_state = {-370, -167.7736361091923, 4.7975558527085962};

    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa);

    // goal_n->setXYZ(Eigen::Vector3d(-150, -167.7736361091923, 0));
    // goal_state = goal_n.get();
    dubins_path = dubins_path_obj.dubins(start_state, goal_state);
    // std::cout << "X position is: " << goal_n->getX() << std::endl;
    // std::cout << "dubins_time: " << dubins_path.length()*Rho/v << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;
    
    // goal_n->setXYZ(Eigen::Vector3d(-370, -167.7736361091923, 0));
    // goal_state = goal_n.get();
    // dubins_path = dubins_path_obj.dubins(start_state, goal_state);
    // std::cout << "X position is: " << goal_n->getX() << std::endl;
    // std::cout << "dubins_time: " << dubins_path.length()*Rho/v << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;

    // goal_n->setXYZ(Eigen::Vector3d(-372.241454, -167.7736361091923, 0));
    // goal_state = goal_n.get();
    // dubins_path = dubins_path_obj.dubins(start_state, goal_state);
    // std::cout << "X position is: " << goal_n->getX() << std::endl;
    // std::cout << "dubins_time: " << dubins_path.length()*Rho/v << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;

    // goal_n->setXYZ(Eigen::Vector3d(-374, -167.7736361091923, 0));
    // goal_state = goal_n.get();
    // dubins_path = dubins_path_obj.dubins(start_state, goal_state);
    // std::cout << "X position is: " << goal_n->getX() << std::endl;
    // std::cout << "dubins_time: " << dubins_path.length()*Rho/v << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;

    // goal_n->setXYZ(Eigen::Vector3d(-400, -167.7736361091923, 0));
    // goal_state = goal_n.get();
    // dubins_path = dubins_path_obj.dubins(start_state, goal_state);
    // std::cout << "X position is: " << goal_n->getX() << std::endl;
    // std::cout << "dubins_time: " << dubins_path.length()*Rho/v << std::endl;
    // std::cout << "dubins_path_type: " << dubins_path.type_[0] << ", " << dubins_path.type_[1] << ", " << dubins_path.type_[2] << std::endl;

}   

TEST_F(TrochoidTestFixture, failure_case1)
{
    trochoids::XYZPsiState start_state = {-196.653, 837.578, 0, 1.06121};
    trochoids::XYZPsiState goal_state = {-19.2448, -424.688, 0, -2.01684};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = -2.77071;
    wind[1] = -23.9548;
    
    desired_speed = 50;
    max_kappa = 0.0746302;

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
}

TEST_F(TrochoidTestFixture, failure_case2)
{
    trochoids::XYZPsiState start_state = {-319.815, -750.816, 0, -2.36801};
    trochoids::XYZPsiState goal_state = {-143.296, 398.456, 0, -0.445093};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {8.83431, 19.2025, 0};
    
    desired_speed = 50;
    max_kappa = 0.0562675;

    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 1194) < 2);
    // std::cout << "value_new: " << value_new << std::endl;


    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 1194) < 2);
    // std::cout << "value_new: " << value_new << std::endl;
}

TEST_F(TrochoidTestFixture, failure_case3)
{
    trochoids::XYZPsiState start_state = {663.51, -793.769, 0, -2.15149};
    trochoids::XYZPsiState goal_state = {-780.297, -536.396, 0, 1.78873};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {6.27431, -13.9293, 0};

    desired_speed = 50;
    max_kappa = 0.0626979;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    // EXPECT_TRUE(abs(value_new - 1194) < 2);
    // std::cout << "value_new: " << value_new << std::endl;


    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);
    // std::cout << "value_new: " << value_new << std::endl;
    // EXPECT_TRUE(abs(value_new - 1194) < 2);
}

TEST_F(TrochoidTestFixture, failure_case4)
{
    trochoids::XYZPsiState start_state = {555.573, 655.098, 0, -2.92817};
    trochoids::XYZPsiState goal_state = {-260.486, -382.695, 0, -2.49624};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {3.69043, 7.17727, 0};    
    
    desired_speed = 50;
    max_kappa = 0.0672958;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    EXPECT_TRUE(abs(value_new - 1321.19) < 2);
    // std::cout << "value_numerical: " << value_new << std::endl;


    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);
    // std::cout << "value_new: " << value_new << std::endl;
    EXPECT_TRUE(abs(value_new - 1321.19) < 2);
}

TEST_F(TrochoidTestFixture, failure_case5)
{
    trochoids::XYZPsiState start_state = {559.691, -923.666, 0, 2.10367};
    trochoids::XYZPsiState goal_state = {222.806, 471.913, 0, -2.88327};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {20.424, -20.0901, 0};

    desired_speed = 50;
    max_kappa = 0.0333166;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    EXPECT_TRUE(abs(value_new - 1455.07) < 2);
    // std::cout << "value_numerical: " << value_new << std::endl;
    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 1455.07) < 2);
}

TEST_F(TrochoidTestFixture, failure_case7)
{
    trochoids::XYZPsiState start_state = {957.782, -202.044, 0, 2.389};
    trochoids::XYZPsiState goal_state = {402.109, 650.92, 0, -0.91143};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {21.4002, -8.15052, 0}; 
    
    desired_speed = 50;
    max_kappa = 0.0491396;

    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 1102.54) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);
    
    EXPECT_TRUE(abs(value_new - 1102.54) < 2);
}

TEST_F(TrochoidTestFixture, failure_case8)
{
    trochoids::XYZPsiState start_state = {533.007, 269.39, 0, 0.416201};
    trochoids::XYZPsiState goal_state = {-113.278, 160.766, 0, 2.8187};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {-9.49159, -21.9547, 0};

    desired_speed = 50;
    max_kappa = 0.0662227;

    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 675.463) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 675.463) < 2);
}

TEST_F(TrochoidTestFixture, failure_case9)
{
    trochoids::XYZPsiState start_state = {980.632, -49.9126, 0, 2.35961};
    trochoids::XYZPsiState goal_state = {240.781, -160.296, 0, 2.83716};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {30.4434, -27.7792, 0};
    
    desired_speed = 50;
    max_kappa = 0.0935017;

    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 748.1) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 748.1) < 2);
}

TEST_F(TrochoidTestFixture, failure_case10)
{
    // Quadrant 3,4 
    // Best is LSR
    trochoids::XYZPsiState start_state = {506.645, 681.799, 0, -2.91607};
    trochoids::XYZPsiState goal_state = {807.936, -680.674, 0, -1.60349};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = -23.2288;
    wind[1] = 4.51207;
    double wind[3] = {-23.2288, 4.51207, 0};

    desired_speed = 50;
    max_kappa = 0.0232594;

    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 1453) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 1453) < 2);
}


TEST_F(TrochoidTestFixture, DISABLED_decision_point_func_save8)
{
    // Failure case 8
    trochoids::XYZPsiState start_state = {-458.78368113158683, 382.34142866860554, 0, 2.3950568527085956};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    double wind[3] = {0, 0, 0};
    
    desired_speed = 50;
    max_kappa = 0.066222699999999995;
    
    // std::ofstream myfile;

    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa);

    // myfile.open(std::string(HOME_PATH) + "decision_point_func_save.csv");

    for (int i = 0; i < 1500; i++)
    {
        // goal_n->setXYZ(Eigen::Vector3d(-i*2-102.6139321276615, -167.7736361091923, 0));
        trochoids::XYZPsiState goal_state = {-i*2-102.6139321276615, -167.7736361091923, 0, 4.7975558527085962};
        trochoid_path.clear();
        bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
        Dubins::DubinsStateSpace::DubinsState start_n = {start_state.x, start_state.y, start_state.psi};
        Dubins::DubinsStateSpace::DubinsState goal_n = {goal_state.x, goal_state.y, goal_state.psi};
        dubins_path = dubins_path_obj.dubins(start_n, goal_n);
        EXPECT_TRUE(valid); 
        // myfile << -i*2-102.6139321276615 << ", " << trochoid_path.length() << std::endl;
        // myfile << -i*2-102.6139321276615 << ", " << dubins_path.length()/max_kappa << std::endl;
    }
    // myfile.close();
}



TEST_F(TrochoidTestFixture, failure_case11)
{
    trochoids::XYZPsiState start_state = {-693.799, -685.417, 0, 3.02215};
    trochoids::XYZPsiState goal_state = {936.204, 222.328, 0, 2.21198};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 27.6369;
    wind[1] = -32.3822;
    double wind_scale = 0.96;
    wind[0] *= wind_scale;
    wind[1] *= wind_scale;
    // std::cout << "Wind magnitude: " << sqrt(wind[0]*wind[0] + wind[1]*wind[1]) << std::endl; 
    desired_speed = 50;
    max_kappa = 0.0329675;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);
}

TEST_F(TrochoidTestFixture, DISABLED_decision_point_func_save11)
{
    // Failure case 11
    trochoids::XYZPsiState start_state = {70.95818489435112, -972.68620442900249, 0, 3.8864472025592836};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 0;
    wind[1] = 0;
    
    desired_speed = 50;
    max_kappa = 0.032967499999999997;
    
    // std::ofstream myfile;

    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa);

    // myfile.open(std::string(HOME_PATH) + "decision_point_func_save11.csv");

    for (int i = 0; i < 5000; i++)
    {
        trochoids::XYZPsiState goal_state = {-i*6+438.6486974769295, 856.44322018554919, 0, 3.0762772025592837};
        trochoid_path.clear();
        bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
        Dubins::DubinsStateSpace::DubinsState start_n = {start_state.x, start_state.y, start_state.psi};
        Dubins::DubinsStateSpace::DubinsState goal_n = {goal_state.x, goal_state.y, goal_state.psi};
        dubins_path = dubins_path_obj.dubins(start_n, goal_n);
        EXPECT_TRUE(valid); 
        // myfile << -i*2-102.6139321276615 << ", " << trochoid_path.length() << std::endl;
        // myfile << -i*6+438.6486974769295 << ", " << dubins_path.length()/max_kappa << std::endl;
    }
    // myfile.close();
}

TEST_F(TrochoidTestFixture, failure_case12)
{
    trochoids::XYZPsiState start_state = {434.384, 604.645, 0, -2.43885};
    trochoids::XYZPsiState goal_state = {661.368, 590.385, 0, 0.568076};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = -15.1501;
    wind[1] = -33.6004;

    desired_speed = 50;
    max_kappa = 0.0620349;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    EXPECT_TRUE(abs(value_new - 311.074935) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 311.074935) < 2);
}

TEST_F(TrochoidTestFixture, failure_case13)
{
    trochoids::XYZPsiState start_state = {-619.979, 454.972, 0, 0.178383};
    trochoids::XYZPsiState goal_state = {176.546, 668.405, 0, 2.58302};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] =-27.0432;
    wind[1] = 31.0691;
    
    desired_speed = 50;
    max_kappa = 0.0300606;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa, true);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    EXPECT_TRUE(abs(value_new - 1021.797280) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 1021.797280) < 2);
}

// // CCC Trajectory
TEST_F(TrochoidTestFixture, DISABLED_dubins_v_numerical1)
{
    trochoids::XYZPsiState start_state = {210.711, 540.281, 0, 2.68493};
    trochoids::XYZPsiState goal_state = {67.1478, 895.458, 0, -0.365924};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 0;
    wind[1] = 0;

    desired_speed = 28;
    max_kappa = 0.00392925;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa, true);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    EXPECT_TRUE(abs(value_new - 1470.25) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 1470.25) < 2);
}

TEST_F(TrochoidTestFixture, DISABLED_dubins_v_numerical2)
{
    trochoids::XYZPsiState start_state = {561.629, -138.353, 0, 0.136536};
    trochoids::XYZPsiState goal_state = {809.895, -429.822, 0, 3.12781};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 0;
    wind[1] = 0;    
    
    desired_speed = 28;
    max_kappa = 0.00472367;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa, true);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);

    EXPECT_TRUE(abs(value_new - 1012.38) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 1012.38) < 2);
}

TEST_F(TrochoidTestFixture, failure_case14)
{
    trochoids::XYZPsiState start_state = {639.565, 192.115, 0, 2.89124};
    trochoids::XYZPsiState goal_state = {-84.0098, 965.034, 0, 2.38807};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = -33.9482;
    wind[1] = 33.8088;
    
    desired_speed = 50;
    max_kappa = 0.00358801;

    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path);
    EXPECT_TRUE(abs(value_new - 1064.37) < 2);
    std::string csv_save = std::string(HOME_PATH) + "failure14_numerical.csv";

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 1064.37) < 2);
    std::string csv_save1 = std::string(HOME_PATH) + "failure14.csv";
}

TEST_F(TrochoidTestFixture, DISABLED_decision_point_func_save14)
{
    // Failure case 14
    trochoids::XYZPsiState start_state = {-317.60415816254476, -587.43438881960378, 0, 0.53298816070245314};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 0;
    wind[1] = 0;
    
    desired_speed = 50;
    max_kappa = 0.00358801;
    
    std::ofstream myfile;

    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa);

    myfile.open(std::string(HOME_PATH) + "decision_point_func_save14.csv");

    for (int i = 0; i < 10000; i++)
    {
        trochoids::XYZPsiState goal_state = {-i+740.50273215912853, -624.50297942996713, 0, 0.029818160702453245};
        trochoid_path.clear();
        bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
        
        Dubins::DubinsStateSpace::DubinsState start_n = {start_state.x, start_state.y, start_state.psi};
        Dubins::DubinsStateSpace::DubinsState goal_n = {goal_state.x, goal_state.y, goal_state.psi};
        dubins_path = dubins_path_obj.dubins(start_n, goal_n);
        EXPECT_TRUE(valid); 
        // myfile << -i*2-102.6139321276615 << ", " << trochoid_path.length() << std::endl;
        myfile << -i+740.50273215912853 << ", " << dubins_path.length()/max_kappa << std::endl;
    }
    myfile.close();
}

TEST_F(TrochoidTestFixture, failure_case15)
{
    trochoids::XYZPsiState start_state = {126.355, -218.411, 0, 0.768467};
    trochoids::XYZPsiState goal_state = {462.019, 139.714, 0, 1.04642};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 32.4644;
    wind[1] = 32.8011;

    desired_speed = 50;
    max_kappa = 0.00822863;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);
    EXPECT_TRUE(abs(value_new - 490.034) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 490.034) < 2);
}

TEST_F(TrochoidTestFixture, DISABLED_decision_point_func_save15)
{
    // Failure case 15
    trochoids::XYZPsiState start_state = {-66.350215221095311, -243.44732466411361, 0, 6.2610952617846101};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 0;
    wind[1] = 0;    
    
    desired_speed = 50;
    max_kappa = 0.0082286300000000007;
    
    std::ofstream myfile;

    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa);

    myfile.open(std::string(HOME_PATH) + "decision_point_func_save15.csv");

    for (int i = 0; i < 10000; i++)
    {
        trochoids::XYZPsiState goal_state = {-i+424.30809788292231, -230.09605869718064, 0, 0.25586295460502384};
        trochoid_path.clear();
        bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
        Dubins::DubinsStateSpace::DubinsState start_n = {start_state.x, start_state.y, start_state.psi};
        Dubins::DubinsStateSpace::DubinsState goal_n = {goal_state.x, goal_state.y, goal_state.psi};

        dubins_path = dubins_path_obj.dubins(start_n, goal_n);
        EXPECT_TRUE(valid); 
        // myfile << -i*2-102.6139321276615 << ", " << trochoid_path.length() << std::endl;
        myfile << -i+424.30809788292231 << ", " << dubins_path.length()/max_kappa << std::endl;
    }
    myfile.close();
}

TEST_F(TrochoidTestFixture, failure_case16)
{
    trochoids::XYZPsiState start_state = {-461.475, 33.0206, 0, -2.15338};
    trochoids::XYZPsiState goal_state = {-737.541, -189.144, 0, -2.88763};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = -22.6279;
    wind[1] = -29.5127;

    desired_speed = 50;
    max_kappa = 0.00507459;

    std::vector<trochoids::XYZPsiState> trochoid_path2;
    bool valid2 = trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path2, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid2);
    double value_new = trochoids::get_length(trochoid_path2);

    EXPECT_TRUE(abs(value_new - 358) < 2);

    trochoid_path.clear();
    bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
    EXPECT_TRUE(valid); 
    if (valid)
        value_new = trochoids::get_length(trochoid_path);

    EXPECT_TRUE(abs(value_new - 358) < 2);
}

TEST_F(TrochoidTestFixture, DISABLED_decision_point_func_save16)
{
    // Failure case 16
    trochoids::XYZPsiState start_state = {254.58292679179445, -386.31213938405278, 0, 0.071530989802925049};
    std::vector<trochoids::XYZPsiState> trochoid_path;
    
    wind[0] = 0;
    wind[1] = 0;
    
    desired_speed = 50;
    max_kappa = 0.0050745900000000004;
    
    std::ofstream myfile;

    Dubins::DubinsStateSpace dubins_path_obj(1/max_kappa);

    myfile.open(std::string(HOME_PATH) + "decision_point_func_save16.csv");

    for (int i = 0; i < 10000; i++)
    {
        trochoids::XYZPsiState goal_state = {-i+598.86410911342648, -470.21692678250321, 0, 5.6204662969825119};
        trochoid_path.clear();
        bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa);
        
        Dubins::DubinsStateSpace::DubinsState start_n = {start_state.x, start_state.y, start_state.psi};
        Dubins::DubinsStateSpace::DubinsState goal_n = {goal_state.x, goal_state.y, goal_state.psi};
        dubins_path = dubins_path_obj.dubins(start_n, goal_n);
        EXPECT_TRUE(valid); 
        // myfile << -i*2-102.6139321276615 << ", " << trochoid_path.length() << std::endl;
        myfile << -i+598.86410911342648 << ", " << dubins_path.length()/max_kappa << std::endl;
    }
    myfile.close();
}

/*
To test
1. Infeasible paths
2. Account for short and long paths. Would need to account for that. Would this 
also make it so k and m could be reduced?
3. What if no decision points, except for start


*/


