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

#include <benchmark/benchmark.h>
#include <random>
#include "trochoids/trochoid_utils.h"


/*
Check trochoids
*/

static void BM_Get_Path_Extend_Straight_Numerical(benchmark::State& state) {
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Extend_Straight_Numerical);

static void BM_Get_Path_Extend_Straight_Analytical(benchmark::State& state) {
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 110, 0};
    trochoids::XYZPsiState goal_state = {1000, 0, 110, 0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Extend_Straight_Analytical);


static void BM_Get_Path_Extend_Circle_Numerical(benchmark::State& state) {
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Extend_Circle_Numerical);

static void BM_Get_Path_Extend_Circle_Analytical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Extend_Circle_Analytical);

static void BM_Get_Path_Max_Kappa_Limit_Numerical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Max_Kappa_Limit_Numerical);

static void BM_Get_Path_Max_Kappa_Limit_Analytical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 100;
    double max_kappa = .07;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Max_Kappa_Limit_Analytical);


static void BM_Get_Path_Large_Radius_Slow_Numerical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Large_Radius_Slow_Numerical);

static void BM_Get_Path_Large_Radius_Slow_Analytical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 1;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Large_Radius_Slow_Analytical);

static void BM_Get_Path_RSR_Numerical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 0.0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_RSR_Numerical);

static void BM_Get_Path_RSR_Analytical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 0.0};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_RSR_Analytical);

static void BM_Get_Path_RSL_Numerical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.570};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_RSL_Numerical);

static void BM_Get_Path_RSL_Analytical(benchmark::State& state) {
    
    double wind[3] = {0, 0, 0};
    double desired_speed = 50;
    double max_kappa = .015;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.570};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_RSL_Analytical);

/******************************************************WIND BENCHMARKS BELOW********************************************************************************************/
static void BM_Get_Path_Large_Radius_Analytical_Wind(benchmark::State& state) {
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Large_Radius_Analytical_Wind);

static void BM_Get_Path_Max_Kappa_Limit_Analytical_Wind(benchmark::State& state) {
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Max_Kappa_Limit_Analytical_Wind);

static void BM_Get_Path_Large_Radius_Slow_Analytical_Wind(benchmark::State& state) {
    
    double wind[3] = {10, 20, 0};
    double desired_speed = 50;
    double max_kappa = .001;

    trochoids::XYZPsiState start_state = {0, 0, 0, 0};
    trochoids::XYZPsiState goal_state = {50, 100, 0, 0.2};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Get_Path_Large_Radius_Slow_Analytical_Wind);

// static void BM_Get_Path_Large_Radius_Slow_Analytical_Wind(benchmark::State& state) {
//     ompl::base::SpaceInformationPtr si_xyzpsi = GetStandardXYZPsiSpacePtr();
//     double wind[3] = {10, 20, 0};
//     double desired_speed = 50;
//     double max_kappa = .001;

//     ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
//     start_n->setXYZ(Eigen::Vector3d(0, 0, 0));
//     start_n->setPsi(0);

//     ob::ScopedState<XYZPsiStateSpace> goal_n(si_xyzpsi);
//     goal_n->setXYZ(Eigen::Vector3d(50,100,0));
//     goal_n->setPsi(0.2);

//     std::vector<trochoids::XYZPsiState> trochoid_path;
//     for (auto _ : state)
//     {
//         benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
//         benchmark::ClobberMemory();
//     }
// }
// BENCHMARK(BM_Get_Path_Large_Radius_Slow_Numerical_Wind);

// /******************************************************INIT_WIND BENCHMARKS BELOW********************************************************************************************/

// // static void BM_Get_Path_Large_Radius_Numerical_Wind(benchmark::State& state) {
// //     
// //     double wind[3] = {10, 20, 0};
// //     double desired_speed = 50;
// //     double max_kappa = .001;

// //     ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
// //     start_n->setXYZ(Eigen::Vector3d(0, 0, 0));
// //     start_n->setPsi(0);

// //     ob::ScopedState<XYZPsiStateSpace> goal_n(si_xyzpsi);
// //     goal_n->setXYZ(Eigen::Vector3d(50,100,0));
// //     goal_n->setPsi(0.2);

// //     std::vector<trochoids::XYZPsiState> trochoid_path;
// //     for (auto _ : state)
// //     {
// //         benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
// //         benchmark::ClobberMemory();
// //     }
// // }
// // BENCHMARK(BM_Get_Path_Large_Radius_Numerical_Wind);

// // static void BM_Get_Path_Max_Kappa_Limit_Numerical_Wind(benchmark::State& state) {
// //     
// //     double wind[3] = {10, 20, 0};
// //     double desired_speed = 50;
// //     double max_kappa = .001;

// //     ob::ScopedState<XYZPsiStateSpace> start_n(si_xyzpsi);
// //     start_n->setXYZ(Eigen::Vector3d(0, 0, 0));
// //     start_n->setPsi(0);

// //     ob::ScopedState<XYZPsiStateSpace> goal_n(si_xyzpsi);
// //     goal_n->setXYZ(Eigen::Vector3d(50,100,0));
// //     goal_n->setPsi(0.2);

// //     std::vector<trochoids::XYZPsiState> trochoid_path;
// //     for (auto _ : state)
// //     {
// //         benchmark::DoNotOptimize(trochoids::get_trochoid_path_numerical(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa));
// //         benchmark::ClobberMemory();
// //     }
// // }
// // BENCHMARK(BM_Get_Path_Max_Kappa_Limit_Numerical_Wind);


static void BM_GetTrochoid_no_classification(benchmark::State& state) {
    
    double desired_speed = 20;
    double max_kappa = 0.01;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    double wind[3] = {5, -5, 0};

    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = false;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    double ang_rate = desired_speed/(1.0/max_kappa);

    trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
    trochoid.problem.Xf = {goal_state.x, goal_state.y, goal_state.psi};

    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoid.getTrochoid());
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_GetTrochoid_no_classification);

static void BM_GetTrochoid_with_classification(benchmark::State& state) {
    
    double desired_speed = 20;
    double max_kappa = 0.01;

    trochoids::XYZPsiState start_state = {0, 0, 0, 1.5707};
    trochoids::XYZPsiState goal_state = {1000, 1000, 0, 1.5707};

    double wind[3] = {5, -5, 0};

    trochoids::Trochoid trochoid;
    trochoid.use_trochoid_classification = true;
    trochoid.problem.v = desired_speed;
    trochoid.problem.wind = {wind[0], wind[1]};
    trochoid.problem.max_kappa = max_kappa;
    double ang_rate = desired_speed/(1.0/max_kappa);

    trochoid.problem.X0 = {start_state.x, start_state.y, start_state.psi};
    trochoid.problem.Xf = {goal_state.x, goal_state.y, goal_state.psi};


    std::vector<trochoids::XYZPsiState> trochoid_path;
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(trochoid.getTrochoid());
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_GetTrochoid_with_classification);

static void BM_Dubins_Random_Exhaustive(benchmark::State& state) {
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);

    double max_kappa = 0.01;

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    for (auto _ : state){
        benchmark::DoNotOptimize(dubins_path_object.dubins({disRange(gen), disRange(gen), disPhi(gen)}, {disRange(gen), disRange(gen), disPhi(gen)}));
        benchmark::ClobberMemory();
    }
        
}
BENCHMARK(BM_Dubins_Random_Exhaustive);

static void BM_Dubins_Random_Classification(benchmark::State& state) {
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disRange(-1000, 1000);
    std::uniform_real_distribution<> disPhi(0.0, 2.0 * M_PI);

    double max_kappa = 0.01;

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    for (auto _ : state){
        benchmark::DoNotOptimize(dubins_path_object.dubins_matrix({disRange(gen), disRange(gen), disPhi(gen)}, {disRange(gen), disRange(gen), disPhi(gen)}));
        benchmark::ClobberMemory();
    }
        
}
BENCHMARK(BM_Dubins_Random_Classification);

static void BM_Dubins_a12(benchmark::State& state) {
    double max_kappa = 0.01;

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    for (auto _ : state){
        benchmark::DoNotOptimize(dubins_path_object.dubins({0, 0, 0.5}, {800, 0, 2.4}));
        benchmark::ClobberMemory();
    }
        
}
BENCHMARK(BM_Dubins_a12);

static void BM_Dubins_a12_random(benchmark::State& state) {
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disPhi1(0.0, M_PI_2);
    std::uniform_real_distribution<> disPhi2(M_PI_2, M_PI);
    double max_kappa = 0.01;

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    for (auto _ : state){
        benchmark::DoNotOptimize(dubins_path_object.dubins_matrix({0, 0, disPhi1(gen)}, {800, 0, disPhi2(gen)}));
        benchmark::ClobberMemory();
    }
        
}
BENCHMARK(BM_Dubins_a12_random);

static void BM_Dubins_a22(benchmark::State& state) {
    double max_kappa = 0.01;

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    for (auto _ : state){
        benchmark::DoNotOptimize(dubins_path_object.dubins({0, 0, 2.8}, {800, 0, 2.4}));
        benchmark::ClobberMemory();
    }
        
}
BENCHMARK(BM_Dubins_a22);

static void BM_Dubins_a22_random(benchmark::State& state) {
    std::random_device rd;
    std::mt19937 gen = std::mt19937(rd());
    std::uniform_real_distribution<> disPhi2(M_PI_2, M_PI);
    double max_kappa = 0.01;

    Dubins::DubinsStateSpace dubins_path_object(1/max_kappa);

    for (auto _ : state){
        benchmark::DoNotOptimize(dubins_path_object.dubins_matrix({0, 0, disPhi2(gen)}, {800, 0, disPhi2(gen)}));
        benchmark::ClobberMemory();
    }
        
}
BENCHMARK(BM_Dubins_a22_random);


BENCHMARK_MAIN();