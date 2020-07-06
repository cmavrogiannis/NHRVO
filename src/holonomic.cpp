#include "NHRVO.h"

void set_preferred_velocities(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < agents.size(); i++)
    {
        RVO::Vector2 curr(agents[i].x, agents[i].y);
        RVO::Vector2 goal(agents[i].goal_x, agents[i].goal_y);
        RVO::Vector2 preferred_velocity = goal - curr;
        if(RVO::absSq(preferred_velocity) > 1.0f)
            preferred_velocity = RVO::normalize(preferred_velocity);
        sim.setAgentPrefVelocity(i, preferred_velocity);
    }
}

void update_agent_positions(RVO::RVOSimulator &sim)
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(int i = 0; i < agents.size(); i++)
    {
        RVO::Vector2 new_pos = sim.getAgentPosition(i);
        agents[i].x = new_pos.x();
        agents[i].y = new_pos.y();
    }
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
        update_agent_positions(sim);
    } while(!goals_reached(sim) && sim.getGlobalTime() < MAX_TIMESTEP);

    return 0;
}
