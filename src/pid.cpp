#include "NHRVO.h"

double compute_steering_angle(const RVO::RVOSimulator &sim, int i)
{
    constexpr double P = 0.12;
    constexpr double D = 0.1;

    RVO::Vector2 ref = sim.getAgentPosition(i);
    RVO::Vector2 curr(agents[i].x, agents[i].y);
    
    double c = std::cos(agents[i].theta);
    double s = std::sin(agents[i].theta);

    RVO::Vector2 p = ref - curr;
    double error_at = c * p.x() + -s * p.y();
    double error_ct = s * p.x() + c * p.y();

    double gain_p = error_ct;
    double gain_d = std::sin(std::atan(error_ct/error_at));
    return P * gain_p + D * gain_d;
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
        double steering_angle = compute_steering_angle(sim, i);
        car_kinematics(agents[i], steering_angle, Car::Speed, SIM_DT);
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
