#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

//#define MOTION_PLAN_DEBUG

struct trajectory_point {
    float position;
    float velocity;
    float acceleration;
};

struct motion_constraints {
    float max_vel;
    float max_accel;
    float max_deccel;
    float max_jerk;
    float max_jerk_deccel;
};

struct st_curve_parameters {
    float jerk_k1;
    float jerk_k2;
    float t_phases[7];
    float t_phase_start_times[7];
    float t_phase_end_times[7];
    trajectory_point phase_points[8];
    float dir = 1.0;
    motion_constraints feasible;
    float execution_time;
};


class MotionPlanner {
public:
    MotionPlanner();
    MotionPlanner(float max_vel, float max_accel, float max_deccel, float max_jerk, float max_jerk_deccel);
    bool planMotion(float start_pos, float end_pos);
    bool planMotion(float start_pos, float end_pos, motion_constraints constraints);
    bool planMotion(float start_pos, float end_pos, float max_vel, float max_accel = 0, float max_jerk = 0);

    trajectory_point sequenceMotion(float t);

    trajectory_point current_point;

    motion_constraints constraints;
    motion_constraints feasible;
    motion_constraints feasible_next;

    float internal_time = 0.0;

    bool executing_traj_flag;


private:
    trajectory_point phase1_acceleration_buildup(float t, trajectory_point start, st_curve_parameters curve);
    trajectory_point phase2_upper_acc_limit(float t, trajectory_point start, st_curve_parameters curve);
    trajectory_point phase3_acceleration_ramp_down(float t, trajectory_point start, st_curve_parameters curve);
    trajectory_point phase4_const_velocity(float t, trajectory_point start, st_curve_parameters curve);
    trajectory_point phase5_deceleration_buildup(float t, trajectory_point start, st_curve_parameters curve);
    trajectory_point phase6_low_deceleration_limit(float t, trajectory_point start, st_curve_parameters curve);
    trajectory_point phase7_deceleration_ramp_down(float t, trajectory_point start, st_curve_parameters curve);

    motion_constraints gen_feasible_constraints(float start, float end, motion_constraints constr);

    st_curve_parameters current_curve;

};

#endif // !MOTION_PLANNER_H