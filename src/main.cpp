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

constexpr double TURN_RADIUS = 10.0f;
constexpr double PATH_LOOKAHEAD = 10.0f;
constexpr double SPEED = 10.0f;

constexpr float MAX_TIMESTEP = 10000.0f;

constexpr int NUM_AGENTS = 2;
std::vector< std::pair<RVO::Vector2, float> > goals;
std::vector<RVO::Vector2> prev_positions;
std::vector<float> facing;

void setup(RVO::RVOSimulator &sim, int num_agents)
{
    sim.setTimeStep(0.16f);
    sim.setAgentDefaults(15.0, // neighborDist
                         10.0, // maxNeighbors
                         10.0f, // timeHorizon
                         10.0f, // radius
                         1.5, // maxSpeed
                         2.0f); // velocity

    for(int i = 0; i < num_agents; i++)
    {
        RVO::Vector2 start_pos = 200.0f * RVO::Vector2(std::cos(i * 2.0f * M_PI / NUM_AGENTS),
                                                       std::sin(i * 2.0f * M_PI / NUM_AGENTS));
        sim.addAgent(start_pos);
        goals.push_back(std::make_pair(-start_pos, 180.0f));
        prev_positions.push_back(start_pos);
        facing.push_back(0.0f); // Make everyone facing upwards
    }
}

void output_positions(RVO::RVOSimulator &sim)
{
    std::cout << sim.getGlobalTime();
    for(size_t i = 0; i < sim.getNumAgents(); i++)
        std::cout << " " << sim.getAgentPosition(i);

    std::cout << std::endl;
}

RVO::Vector2 compute_preferred_velocity(RVO::Vector2 current_position, int agent)
{
    double p1[3] = {
        static_cast<double>(current_position.x()),
        static_cast<double>(current_position.y()),
        static_cast<double>(facing[agent]),
    };

    double p2[3] = {
        static_cast<double>(goals[agent].first.x()),
        static_cast<double>(goals[agent].first.y()),
        static_cast<double>(goals[agent].second),
    };

    DubinsPath path;
    if(int err = dubins_shortest_path(&path, p1, p2, TURN_RADIUS))
    {
        RVO::Vector2 default_vel = SPEED * RVO::Vector2(std::cos(facing[agent]), std::sin(facing[agent]));
        std::cerr << "Failed to compute path, error number " << err << ", returning " << default_vel << '\n';
        return default_vel;
    }

    if(dubins_path_length(&path) <= PATH_LOOKAHEAD)
        return RVO::Vector2(0, 0);
    
    double q[3];
    if(dubins_path_sample(&path, PATH_LOOKAHEAD, q))
    {
        std::cerr << "Failed to sample path\n";
        return RVO::Vector2(0, 0);
    }

    RVO::Vector2 sampled_pos(q[0], q[1]);
    RVO::Vector2 velocity = sampled_pos - current_position;
    velocity = SPEED * RVO::normalize(velocity);
    return velocity;
}

void set_preferred_velocities(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < sim.getNumAgents(); i++)
    {
        RVO::Vector2 current_position = sim.getAgentPosition(i);
        RVO::Vector2 preferred_velocity = compute_preferred_velocity(current_position, i);
        prev_positions[i] = current_position;
        sim.setAgentPrefVelocity(i, preferred_velocity);
    }
}

void update_facing(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < sim.getNumAgents(); i++)
    {
        RVO::Vector2 new_position = sim.getAgentPosition(i);
        RVO::Vector2 old_position = prev_positions[i];
        RVO::Vector2 old_facing_vec(std::cos(facing[i]), std::sin(facing[i]));
        RVO::Vector2 actual_velocity = RVO::normalize(new_position - old_position);
        float dot_product = std::max(-1.0f, std::min(1.0f, old_facing_vec * actual_velocity));
        float dtheta = std::acos(dot_product);
        if(actual_velocity.y() > old_facing_vec.y())
            facing[i] += dtheta;
        else
            facing[i] -= dtheta;
    }
}

bool goals_reached(RVO::RVOSimulator &sim)
{
    for(int i = 0; i < sim.getNumAgents(); i++)
    {
        RVO::Vector2 delta = sim.getAgentPosition(i) - goals[i].first;
        float radius_squared = sim.getAgentRadius(i) * sim.getAgentRadius(i);
        if(RVO::absSq(delta) > radius_squared)
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
        update_facing(sim);
    } while(!goals_reached(sim) && sim.getGlobalTime() < MAX_TIMESTEP);

    return 0;
}

