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


constexpr double SIM_DT = 0.16;
constexpr float MAX_TIMESTEP = 10000.0f;

constexpr int NUM_AGENTS = 2;

std::vector<Car> agents;

void car_kinematics(Car &c, double steering_angle, double speed, double dt)
{

    c.x += (Car::Length / std::tan(steering_angle))*(-sin(c.theta));
    c.y += (Car::Length / std::tan(steering_angle))*cos(c.theta);
    c.theta += (speed / Car::Length) * std::tan(steering_angle) * dt;
    c.x += (Car::Length / std::tan(steering_angle))*sin(c.theta);
    c.y += (Car::Length / std::tan(steering_angle))*(-cos(c.theta));
}

void setup(RVO::RVOSimulator &sim, int num_agents)
{
    sim.setTimeStep(SIM_DT);
    // Initial velocity is 0
    sim.setAgentDefaults(15.0f, // neighborDist
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

RVO::Vector2 compute_preferred_velocity(const Car &c)
{
    RVO::Vector2 curr(c.x, c.y);
    RVO::Vector2 goal(c.goal_x, c.goal_y);
    RVO::Vector2 vel = goal - curr;
    if(RVO::absSq(vel) > 1.0f)
        vel = RVO::normalize(vel);
    return vel;
}

void set_preferred_velocities(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < agents.size(); i++)
    {
        RVO::Vector2 preferred_velocity = compute_preferred_velocity(agents[i]);
        sim.setAgentPrefVelocity(i, preferred_velocity);
    }
}

void perform_actual_movement(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < agents.size(); i++)
    {
        RVO::Vector2 new_pos = sim.getAgentPosition(i);
        double p1[3] = { agents[i].x, agents[i].y, agents[i].theta };
        // TODO: Make target orientation be towards goal
        double p2[3] = { new_pos.x(), new_pos.y(), agents[i].theta };
        DubinsPath path;
        if(int err = dubins_shortest_path(&path, p1, p2, Car::TurnRadius))
        {
            //std::cerr << "Failed to compute path, error number " << err << '\n';
            car_kinematics(agents[i], 0, Car::Speed, SIM_DT);
            return;
        }

        // TODO: Capture case where first segment has length 0. 
        switch(path.type)
        {
        case LSL:
        case LSR:
        case LRL:
            car_kinematics(agents[i], -Car::MaxSteeringAngle, Car::Speed, SIM_DT);
            break;
        case RSL:
        case RSR:
        case RLR:
            car_kinematics(agents[i], Car::MaxSteeringAngle, Car::Speed, SIM_DT);
            break;
        }
    }
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
        double dist_x_sq = (c.goal_x - c.x) * (c.goal_x - c.x);
        double dist_y_sq = (c.goal_y - c.y) * (c.goal_y - c.y);
        double dist_sq = dist_x_sq + dist_y_sq;
        if(dist_sq > Car::Length * Car::Length)
            return false;
    }
    return true;
}

int main()
{
    RVO::RVOSimulator sim;
    setup(sim, NUM_AGENTS);
    
    do
    {
        output_positions(sim);
        set_preferred_velocities(sim);
        sim.doStep();
        perform_actual_movement(sim);
        update_positions_in_sim(sim);
    } while(!goals_reached(sim) && sim.getGlobalTime() < MAX_TIMESTEP);

    return 0;
}


