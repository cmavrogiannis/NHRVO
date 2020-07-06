#pragma once

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#if _OPENMP
#include <omp.h>
#endif

#include "RVO.h"
#include "dubins.h"
#include "dubins.c"

#ifndef M_PI
constexpr float M_PI = 3.14159265358979323846f;
#endif

struct Car
{
    double x, y, theta;

    double goal_x, goal_y, goal_theta;
    // TODO: Ensure these values are correct
    // Radians
    static constexpr double MaxSteeringAngle = M_PI / 4.0;

    // Meters
    static constexpr double Length = 0.3;
    static constexpr double TurnRadius = Car::Length / std::tan(Car::MaxSteeringAngle);

    // Meters / sec
    static constexpr double Speed = 1.0;
};


constexpr double SIM_DT = 0.1;
constexpr float MAX_TIMESTEP = 10000.0f;

constexpr int NUM_AGENTS = 2;

std::vector<Car> agents;

bool car_is_at_goal(const Car &c)
{
    double dist_x_sq = (c.goal_x - c.x) * (c.goal_x - c.x);
    double dist_y_sq = (c.goal_y - c.y) * (c.goal_y - c.y);
    double dist_sq = dist_x_sq + dist_y_sq;
    return dist_sq <= Car::Length * Car::Length;
}

void car_kinematics(Car &c, double steering_angle, double speed, double dt)
{
    double L_tand = Car::Length / std::tan(steering_angle);
    double theta_old = c.theta;
    c.theta += speed / Car::Length * std::tan(steering_angle) * dt;
    c.x += L_tand * (std::sin(c.theta) - std::sin(theta_old));
    c.y += L_tand * (-std::cos(c.theta) + std::cos(theta_old));
}

void setup(RVO::RVOSimulator &sim, int num_agents)
{
    sim.setTimeStep(SIM_DT);
    // Initial velocity is 0
    sim.setAgentDefaults(2.0f, // neighborDist
                         static_cast<size_t>(10), // maxNeighbors
                         10.0f, // timeHorizon
                         10.0f, // timeHorizonObst
                         static_cast<float>(Car::Speed * SIM_DT + 0.1), // radius
                         static_cast<float>(Car::Speed)); // maxSpeed
    

    for(int i = 0; i < num_agents; i++)
    {
        RVO::Vector2 start_pos = 10.0f * RVO::Vector2(std::cos(i * 2.0f * M_PI / NUM_AGENTS),
                                                      std::sin(i * 2.0f * M_PI / NUM_AGENTS));
        sim.addAgent(start_pos);
        
        Car new_agent = { start_pos.x(), start_pos.y(), 0.0f, -start_pos.x(), -start_pos.y(), M_PI };
        agents.push_back(new_agent);
    }
}

void output_positions(RVO::RVOSimulator &sim)
{
    std::cout << sim.getGlobalTime();
    for(const Car &c : agents)
        std::cout << " (" << c.x << "," << c.y << "," << c.theta << ")";
    std::cout << std::endl;
}

void update_positions_in_sim(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < agents.size(); i++)
    {
        RVO::Vector2 pos(agents[i].x, agents[i].y);
        sim.setAgentPosition(i, pos);
    }
}

bool goals_reached(RVO::RVOSimulator &sim)
{
    for(const Car &c : agents)
    {
        if(!car_is_at_goal(c))
            return false;
    }
    return true;
}
