#include "NHRVO.h"

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
        if(car_is_at_goal(agents[i]))
        {
            sim.setAgentPrefVelocity(i, RVO::Vector2(0, 0));
            continue;
        }
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
        if(car_is_at_goal(agents[i]))
            continue;

        RVO::Vector2 new_pos = sim.getAgentPosition(i);
        double p1[3] = { agents[i].x, agents[i].y, agents[i].theta };
        RVO::Vector2 goal(agents[i].goal_x, agents[i].goal_y);
        RVO::Vector2 q_orca(new_pos.x(), new_pos.y());
        RVO::Vector2 vec2goal = goal - q_orca;
	double p2[3] = { new_pos.x(), new_pos.y(), std::atan2(vec2goal.y(), vec2goal.x()) };

        DubinsPath path;
        if(int err = dubins_shortest_path(&path, p1, p2, Car::TurnRadius))
        {
            //std::cerr << "Failed to compute path, error number " << err << '\n';
            car_kinematics(agents[i], 0, Car::Speed, SIM_DT);
            return;
        }

        DubinsSegmentType dir = get_first_nonzero_segment(path);
        switch(dir)
        {
        case L:
            car_kinematics(agents[i], Car::MaxSteeringAngle, Car::Speed, SIM_DT);
            break;
        case R:
            car_kinematics(agents[i], -Car::MaxSteeringAngle, Car::Speed, SIM_DT);
            break;
        case S:
        default:
            car_kinematics(agents[i], 0.0, Car::Speed, SIM_DT);
            break;
        }
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
        perform_actual_movement(sim);
        update_positions_in_sim(sim);
    } while(!goals_reached(sim) && sim.getGlobalTime() < MAX_TIMESTEP);

    return 0;
}
